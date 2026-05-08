#!/usr/bin/env python3
"""
OSKI Ground Station

Connects to the Feather V2 ground radio, displays parsed telemetry in a
split-screen terminal, accepts commands on a >> prompt, and logs every
packet to InfluxDB.

Environment variables:
  INFLUX_URL    default: http://localhost:8086
  INFLUX_TOKEN  required for InfluxDB logging (omit to disable)
  INFLUX_ORG    default: oski
"""

import os
import sys
import math
import time
import struct
import curses
import threading
import queue
from dataclasses import dataclass
from datetime import datetime
from dotenv import load_dotenv

load_dotenv()

import serial
import serial.tools.list_ports
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# ── Config ──────────────────────────────────────────────────────────────────────

INFLUX_URL    = os.getenv("INFLUX_URL",   "http://localhost:8086")
INFLUX_TOKEN  = os.getenv("INFLUX_TOKEN", "")
INFLUX_ORG    = os.getenv("INFLUX_ORG",  "oski")
INFLUX_BUCKET = "oskisat_telem"
BAUD          = 115200
AUTO_PING_ENABLED = False

FEATHER_V2_VID = 0x1A86  # WCH CH340 (Feather ESP32 V2)
ESP32_S3_VID   = 0x303A  # Espressif native USB (FC S3 DevKitC)

PING_INTERVAL_S  = 5.0
PING_TIMEOUT_S   = 4.0
DEPLOY_TIMEOUT_S = 6.0
RESCAN_INTERVAL  = 3.0

# Packet IDs — must match Common.h
CMD_PING            = 0
CMD_TAKE_PHOTO      = 1
CMD_ADCS_SETPOINT   = 2
CMD_DEPLOY          = 3
CMD_SET_CAMERA_RES  = 4
CMD_CTRL_5V         = 5
CMD_ADCS_ENABLE     = 6
CMD_ADCS_ZERO       = 7
CMD_ADCS_SET_PID    = 8
CMD_RESET           = 9
CMD_ADCS_WHEEL_VEL  = 10
CMD_ADCS_POWER_ON   = 11
CMD_ADCS_POWER_OFF  = 12
CMD_ADCS_PING       = 13
BMS_TELEMETRY  = 101
ADCS_TELEMETRY = 102
ADCS_PARAMS    = 103
CAM_IMAGE_META = 200
CAM_IMAGE_DATA = 201
CAM_IMAGE_DONE = 202
CAM_NACK       = 203
CAM_IMAGE_ACK  = 204

FRAME_SYNC = b"\xA5\x5A"
MAX_MISSING_SEQS_PER_NACK = 125

MEAS_NAME = {
    CMD_PING:           "ping",
    CMD_TAKE_PHOTO:     "take_photo",
    CMD_ADCS_SETPOINT:  "adcs_setpoint",
    CMD_ADCS_ENABLE:    "adcs_enable",
    CMD_ADCS_ZERO:      "adcs_zero",
    CMD_ADCS_SET_PID:   "adcs_set_pid",
    CMD_ADCS_WHEEL_VEL: "adcs_wheel_vel",
    CMD_ADCS_POWER_ON:  "adcs_power_on",
    CMD_ADCS_POWER_OFF: "adcs_power_off",
    CMD_ADCS_PING:      "adcs_ping",
    ADCS_TELEMETRY:     "adcs_telem",
    ADCS_PARAMS:        "adcs_params",
    CMD_SET_CAMERA_RES: "set_camera_res",
    CMD_DEPLOY:         "deploy",
    CMD_CTRL_5V:        "ctrl_5v",
    CMD_RESET:          "reset",
    BMS_TELEMETRY:      "bms",
}

CAMERA_RES_CODES = {
    "qvga": 0,
    "vga": 1,
    "svga": 2,
    "xga": 3,
    "hd": 4,
    "sxga": 5,
    "uxga": 6,
    "qxga": 7,
}

CAMERA_RES_NAMES = {value: key for key, value in CAMERA_RES_CODES.items()}

RESET_REASON_NAMES = {
    0: "unknown",
    1: "poweron",
    2: "ext",
    3: "software",
    4: "panic",
    5: "int_wdt",
    6: "task_wdt",
    7: "wdt",
    8: "deepsleep",
    9: "brownout",
    10: "sdio",
    11: "usb",
    12: "jtag",
    13: "efuse",
    14: "pwr_glitch",
    15: "cpu_lockup",
}

# LoRa addressing — must match ground station FW
FC_ADDR    = 0xAA
LOCAL_ADDR = 0xBB

# ── Pending command tracker ──────────────────────────────────────────────────────

@dataclass
class PendingCmd:
    log_idx:     int
    sent_at:     float
    timeout:     float = PING_TIMEOUT_S
    timeout_msg: str   = " → NO ACK"

# ── Helpers ──────────────────────────────────────────────────────────────────────

def quat_to_euler(w, x, y, z):
    """ZYX convention inverse of euler_to_quat — returns (roll, pitch, yaw) in degrees."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = 2.0 * (w * y - z * x)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))

    return roll, pitch, yaw

def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    """ZYX convention: yaw applied first, then pitch, then roll."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w =  cr * cp * cy + sr * sp * sy
    x =  sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z =  cr * cp * sy - sr * sp * cy
    return w, x, y_, z

def fmt_uptime(total_seconds):
    hours, rem = divmod(int(total_seconds), 3600)
    minutes, seconds = divmod(rem, 60)
    return f"{hours:d}:{minutes:02d}:{seconds:02d}"

def reset_reason_name(code):
    return RESET_REASON_NAMES.get(int(code), f"code={int(code)}")

def build_uplink(pkt_id, data=b""):
    """Return a hex string the ground station FW will relay verbatim over LoRa."""
    return (bytes([FC_ADDR, LOCAL_ADDR, pkt_id, len(data)]) + data).hex()

# ── Serial port detection ───────────────────────────────────────────────────────

def find_feather():
    for p in serial.tools.list_ports.comports():
        if p.vid == FEATHER_V2_VID:
            return p.device
    return None

# ── Packet parsing ──────────────────────────────────────────────────────────────

def parse_data_line(line):
    if not line.startswith("DATA "):
        return None
    parts = line.split()
    try:
        pkt_id = int(parts[1])
        rssi   = int(parts[-2].split("=")[1])
        snr    = float(parts[-1].split("=")[1])
        raw    = bytes(int(b, 16) for b in parts[3:-2])
        return pkt_id, raw, rssi, snr
    except (IndexError, ValueError):
        return None

def extract_binary_frames(rx_buf):
    frames = []
    min_frame_len = 8

    while True:
        sync_idx = rx_buf.find(FRAME_SYNC)
        if sync_idx < 0:
            if len(rx_buf) > 1:
                del rx_buf[:-1]
            break
        if sync_idx > 0:
            del rx_buf[:sync_idx]
        if len(rx_buf) < min_frame_len:
            break

        payload_len = rx_buf[3]
        frame_len = min_frame_len + payload_len
        if len(rx_buf) < frame_len:
            break

        frame = bytes(rx_buf[:frame_len])
        checksum = sum(frame[2:-1]) & 0xFF
        if checksum != frame[-1]:
            del rx_buf[0]
            continue

        pkt_id = frame[2]
        raw_len = frame[3]
        rssi = struct.unpack_from("<h", frame, 4)[0]
        snr = struct.unpack_from("<b", frame, 6)[0] / 4.0
        raw = frame[7:7 + raw_len]
        frames.append((pkt_id, raw, rssi, snr))
        del rx_buf[:frame_len]

    return frames

def fmt_packet(pkt_id, raw, rssi, snr):
    ts = datetime.now().strftime("%H:%M:%S")
    if pkt_id == BMS_TELEMETRY and len(raw) >= 22:
        c1, c2, c3, c4, stack = struct.unpack_from("<HHHHH", raw, 0)
        current  = struct.unpack_from("<i", raw, 10)[0]
        int_temp = struct.unpack_from("<f", raw, 14)[0]
        ts_temp  = struct.unpack_from("<f", raw, 18)[0]
        msg = (f"[{ts}] BMS — "
               f"{c1}/{c2}/{c3}/{c4} mV  stack={stack} mV  "
               f"current={current} mA  "
               f"intTemp={int_temp:.1f}°C  tsTemp={ts_temp:.1f}°C")
        if len(raw) >= 27:
            uptime_s = struct.unpack_from("<I", raw, 22)[0]
            reset_reason = raw[26]
            msg += f"  uptime={fmt_uptime(uptime_s)}  reset={reset_reason_name(reset_reason)}"
        return msg
    return f"[{ts}] PKT id={pkt_id} len={len(raw)} {raw.hex()}"

_TS_PAD = " " * 11  # aligns continuation lines under content after "[HH:MM:SS] "

def fmt_adcs_telem(ts, raw, rssi, snr):
    v = struct.unpack_from("<14f", raw, 0)
    sp_r,  sp_p,  sp_y  = quat_to_euler(v[0], v[1], v[2], v[3])
    cur_r, cur_p, cur_y = quat_to_euler(v[4], v[5], v[6], v[7])
    return [
        f"[{ts}] ADCS TELEM  RSSI={rssi} SNR={snr:.1f}",
        f"{_TS_PAD}  setpoint  roll={sp_r:+8.3f}°  pitch={sp_p:+8.3f}°  yaw={sp_y:+8.3f}°",
        f"{_TS_PAD}  current   roll={cur_r:+8.3f}°  pitch={cur_p:+8.3f}°  yaw={cur_y:+8.3f}°",
        f"{_TS_PAD}  gyro r/s  x={v[8]:+.4f} y={v[9]:+.4f} z={v[10]:+.4f}",
        f"{_TS_PAD}  integral  x={v[11]:+.6f} y={v[12]:+.6f} z={v[13]:+.6f}",
    ]

def fmt_adcs_params(ts, raw, rssi, snr):
    v = struct.unpack_from("<9f", raw, 0)
    en = ["ON " if raw[36 + i] else "OFF" for i in range(3)]
    return [
        f"[{ts}] ADCS PARAMS  RSSI={rssi} SNR={snr:.1f}",
        f"{_TS_PAD}  X [{en[0]}]  Kp={v[0]:.3e}  Ki={v[1]:.3e}  Kd={v[2]:.3e}",
        f"{_TS_PAD}  Y [{en[1]}]  Kp={v[3]:.3e}  Ki={v[4]:.3e}  Kd={v[5]:.3e}",
        f"{_TS_PAD}  Z [{en[2]}]  Kp={v[6]:.3e}  Ki={v[7]:.3e}  Kd={v[8]:.3e}",
    ]

def fmt_img_progress(ts, received, total, expected_len, seq=None, chunk_len=None):
    msg = f"[{ts}] IMG RX — {received}/{total} chunks received"
    if expected_len:
        msg += f"  ({expected_len} B expected)"
    if seq is not None:
        msg += f"  last seq={seq}"
    if chunk_len is not None:
        msg += f"  payload={chunk_len} B"
    return msg

def influx_fields(pkt_id, raw):
    if pkt_id == CMD_PING and len(raw) >= 8:
        echoed, val = struct.unpack_from("<II", raw, 0)
        return {"echoed": int(echoed), "val": int(val)}
    if pkt_id == BMS_TELEMETRY and len(raw) >= 22:
        c1, c2, c3, c4, stack = struct.unpack_from("<HHHHH", raw, 0)
        current  = struct.unpack_from("<i", raw, 10)[0]
        int_temp = struct.unpack_from("<f", raw, 14)[0]
        ts_temp  = struct.unpack_from("<f", raw, 18)[0]
        fields = {
            "cell1_mV": int(c1), "cell2_mV": int(c2),
            "cell3_mV": int(c3), "cell4_mV": int(c4),
            "stack_mV": int(stack), "current_mA": int(current),
            "intTemp_C": float(int_temp), "tsTemp_C": float(ts_temp),
        }
        if len(raw) >= 27:
            fields["uptime_s"] = int(struct.unpack_from("<I", raw, 22)[0])
            fields["reset_reason"] = int(raw[26])
        return fields
    if pkt_id == ADCS_TELEMETRY and len(raw) >= 56:
        v = struct.unpack_from("<14f", raw, 0)
        des_roll, des_pitch, des_yaw = quat_to_euler(v[0], v[1], v[2], v[3])
        cur_roll, cur_pitch, cur_yaw = quat_to_euler(v[4], v[5], v[6], v[7])
        return {
            "des_qw": v[0],  "des_qx": v[1],  "des_qy": v[2],  "des_qz": v[3],
            "des_roll": des_roll, "des_pitch": des_pitch, "des_yaw": des_yaw,
            "qw":     v[4],  "qx":     v[5],  "qy":     v[6],  "qz":     v[7],
            "roll":   cur_roll, "pitch": cur_pitch, "yaw": cur_yaw,
            "wx":     v[8],  "wy":     v[9],  "wz":     v[10],
            "int_x":  v[11], "int_y":  v[12], "int_z":  v[13],
        }
    if pkt_id == ADCS_PARAMS and len(raw) >= 39:
        v = struct.unpack_from("<9f", raw, 0)
        return {
            "kp_x": v[0], "ki_x": v[1], "kd_x": v[2],
            "kp_y": v[3], "ki_y": v[4], "kd_y": v[5],
            "kp_z": v[6], "ki_z": v[7], "kd_z": v[8],
            "x_en": int(raw[36]), "y_en": int(raw[37]), "z_en": int(raw[38]),
        }
    return None

# ── InfluxDB helpers ────────────────────────────────────────────────────────────

def _is_connection_refused(exc):
    s = str(exc)
    return "Connection refused" in s or "NewConnectionError" in s

def ensure_bucket(client):
    api = client.buckets_api()
    existing = [b.name for b in api.find_buckets().buckets]
    if INFLUX_BUCKET not in existing:
        api.create_bucket(bucket_name=INFLUX_BUCKET, org=INFLUX_ORG)
        return True
    return False

def write_point(write_api, pkt_id, raw, rssi, snr):
    fields = influx_fields(pkt_id, raw)
    if fields is None:
        return
    meas = MEAS_NAME.get(pkt_id, f"pkt_{pkt_id}")
    p = Point(meas).tag("source", "fc")
    for k, v in fields.items():
        p = p.field(k, v)
    p = p.field("rssi", rssi).field("snr", snr)
    write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=p)

# ── Curses UI ───────────────────────────────────────────────────────────────────
# Color pairs: 1=green (data), 2=cyan (system), 3=yellow (attention), 4=red (error), 5=magenta (user commands)

def run_ui(stdscr, initial_port, write_api, influx_status):
    curses.curs_set(1)
    stdscr.nodelay(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,    curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    log           = []
    input_buf     = ""
    pkt_q         = queue.Queue()
    cmd_q         = queue.Queue()
    serial_status = ["scanning…"]  # mutable so serial_thread can update it

    # pending: keyed by command name, each entry expires after its timeout
    pending: dict[str, PendingCmd] = {}
    # pending_pings is keyed by ping arg rather than a singleton, so kept separate
    pending_pings  = {}
    ping_arg       = 0
    last_ping_time = 0.0
    pending_burst_request = None
    active_burst = None

    last_adcs_cur_quat = None  # (qw, qx, qy, qz) from last ADCS_TELEMETRY current attitude

    # image reassembly state
    img_chunks       = {}
    img_transfer_id  = None
    img_total        = 0
    img_expected_len = 0
    img_last_missing = None
    img_have_new_chunks = False
    img_last_nack_at = 0.0
    img_progress_idx = None

    # ── Shared helpers ────────────────────────────────────────────────────────

    def resolve(key, msg, rssi, snr, color=None):
        """Update a pending command's log line with an ACK message."""
        p = pending.pop(key, None)
        if p is None:
            return
        ts = log[p.log_idx][0][1:9]
        log[p.log_idx][0] = f"[{ts}] {msg}  RSSI={rssi} SNR={snr}"
        if color is not None:
            log[p.log_idx][1] = color

    def send_cmd(pkt_id, payload, log_msg, pending_key=None,
                 timeout=PING_TIMEOUT_S, timeout_msg=" → NO ACK"):
        """Enqueue an uplink, append a log line, and register a pending entry."""
        cmd_q.put(build_uplink(pkt_id, payload))
        ts = datetime.now().strftime("%H:%M:%S")
        log.append([f"[{ts}] {log_msg}", 5])
        if pending_key is not None:
            pending[pending_key] = PendingCmd(len(log) - 1, time.time(), timeout, timeout_msg)

    def show_help(ts):
        valid_res = ", ".join(CAMERA_RES_CODES.keys())
        help_lines = [
            f"[{ts}] Commands:",
            f"{_TS_PAD}  help | commands | ?",
            f"{_TS_PAD}  ping",
            f"{_TS_PAD}  deploy",
            f"{_TS_PAD}  setres <{valid_res}>",
            f"{_TS_PAD}  photo <count> <spacing_s>",
            f"{_TS_PAD}  adcs zero",
            f"{_TS_PAD}  adcs enable <on|off>  (Z axis)",
            f"{_TS_PAD}  adcs enable <on|off> <on|off> <on|off>",
            f"{_TS_PAD}  adcs set <yaw_deg>  (roll=0 pitch=0)",
            f"{_TS_PAD}  adcs set <roll_deg> <pitch_deg> <yaw_deg>",
            f"{_TS_PAD}  adcs set current",
            f"{_TS_PAD}  adcs pid <kp> <ki> <kd>  (Z axis)",
            f"{_TS_PAD}  adcs pid <x|y|z> <kp> <ki> <kd>",
            f"{_TS_PAD}  adcs vel <vel_rad_s>  (Z axis)",
            f"{_TS_PAD}  adcs vel <x|y|z> <vel_rad_s>",
            f"{_TS_PAD}  adcs poweron",
            f"{_TS_PAD}  adcs poweroff",
            f"{_TS_PAD}  adcs ping",
            f"{_TS_PAD}  ctrl5v <on|off>",
            f"{_TS_PAD}  reset",
        ]
        for line in help_lines:
            log.append([line, 2])

    # ── ACK handlers (one per packet ID) ──────────────────────────────────────

    def on_deploy(raw, rssi, snr):
        if len(raw) >= 1 and raw[0] == 0:
            resolve("deploy", "DEPLOY ACK — sequence triggered", rssi, snr)
        else:
            resolve("deploy", "DEPLOY — invalid command", rssi, snr, color=4)

    def on_ctrl5v(raw, rssi, snr):
        state_str = "ON" if (raw[0] if len(raw) >= 1 else 0) else "OFF"
        resolve("ctrl5v", f"CTRL5V ACK — set {state_str}", rssi, snr)

    def on_adcs_setpoint(raw, rssi, snr):
        resolve("adcs", "ADCS ACK — setpoint confirmed", rssi, snr)

    def on_adcs_zero(raw, rssi, snr):
        resolve("adcs_zero", "ADCS ZERO ACK — wheels zeroed", rssi, snr)

    def on_adcs_enable(raw, rssi, snr):
        if len(raw) >= 3:
            xs, ys, zs = ["ON" if raw[i] else "OFF" for i in range(3)]
            msg = f"ADCS ENABLE ACK — X={xs} Y={ys} Z={zs}"
        else:
            msg = "ADCS ENABLE ACK"
        resolve("adcs_enable", msg, rssi, snr)

    def on_reset(raw, rssi, snr):
        resolve("reset", "RESET ACK — FC restarting", rssi, snr)

    def on_adcs_wheel_vel(raw, rssi, snr):
        if len(raw) >= 5:
            axis_n, vel = struct.unpack_from("<Bf", raw, 0)
            axis_s = ("X", "Y", "Z")[axis_n] if axis_n < 3 else str(axis_n)
            msg = f"ADCS WHEEL VEL ACK — {axis_s}={vel:.3f} rad/s (ADCS {axis_s} disabled)"
        else:
            msg = "ADCS WHEEL VEL ACK"
        resolve("adcs_wheel_vel", msg, rssi, snr)

    def on_adcs_power_on(raw, rssi, snr):
        resolve("adcs_power_on", "ADCS POWER ON ACK", rssi, snr)

    def on_adcs_power_off(raw, rssi, snr):
        resolve("adcs_power_off", "ADCS POWER OFF ACK", rssi, snr)

    def on_adcs_ping(raw, rssi, snr):
        alive = len(raw) >= 1 and raw[0] == 1
        msg = f"ADCS PING — {'YES (alive)' if alive else 'NO (no response)'}  RSSI={rssi} SNR={snr}"
        resolve("adcs_ping", msg, rssi, snr, color=1 if alive else 4)

    def on_adcs_pid(raw, rssi, snr):
        if len(raw) >= 13:
            axis_n, kp, ki, kd = struct.unpack_from("<Bfff", raw, 0)
            axis_s = ("X", "Y", "Z")[axis_n] if axis_n < 3 else str(axis_n)
            msg = f"ADCS PID ACK — {axis_s} Kp={kp:.6f} Ki={ki:.6f} Kd={kd:.6f}"
        else:
            msg = "ADCS PID ACK"
        resolve("adcs_pid", msg, rssi, snr)

    def on_take_photo(raw, rssi, snr):
        nonlocal pending_burst_request, active_burst

        status = raw[0] if len(raw) >= 1 else 255
        if status == 0:
            resolve("photo", "PHOTO ACK — triggered", rssi, snr)
            if pending_burst_request is not None:
                if pending_burst_request["count"] > 1:
                    active_burst = {
                        "folder": pending_burst_request["folder"],
                        "count": pending_burst_request["count"],
                        "saved": 0,
                    }
                    ts = datetime.now().strftime("%H:%M:%S")
                    log.append([f"[{ts}] PHOTO BURST — saving to {active_burst['folder']}", 2])
                pending_burst_request = None
        elif status == 2:
            resolve("photo", "PHOTO NACK — invalid photo count", rssi, snr, color=4)
            pending_burst_request = None
        elif status == 3:
            resolve("photo", "PHOTO NACK — camera buffer full", rssi, snr, color=4)
            pending_burst_request = None
        elif status == 4:
            resolve("photo", "PHOTO NACK — camera comms error", rssi, snr, color=4)
            pending_burst_request = None
        else:
            resolve("photo", "PHOTO NACK — busy/error", rssi, snr, color=4)
            pending_burst_request = None

    def on_camera_res(raw, rssi, snr):
        status = raw[0] if len(raw) >= 1 else 255
        resolution = raw[1] if len(raw) >= 2 else 255
        res_name = CAMERA_RES_NAMES.get(resolution, f"code={resolution}")

        if status == 0:
            resolve("cam_res", f"CAM RES ACK — {res_name}", rssi, snr)
        elif status == 1:
            resolve("cam_res", f"CAM RES NACK — busy  current={res_name}", rssi, snr, color=4)
        elif status == 5:
            resolve("cam_res", f"CAM RES NACK — unsupported  current={res_name}", rssi, snr, color=4)
        else:
            resolve("cam_res", f"CAM RES NACK — error status={status} current={res_name}", rssi, snr, color=4)

    ack_dispatch = {
        CMD_TAKE_PHOTO:      on_take_photo,
        CMD_SET_CAMERA_RES:  on_camera_res,
        CMD_DEPLOY:        on_deploy,
        CMD_CTRL_5V:       on_ctrl5v,
        CMD_ADCS_SETPOINT: on_adcs_setpoint,
        CMD_ADCS_ZERO:     on_adcs_zero,
        CMD_ADCS_ENABLE:   on_adcs_enable,
        CMD_ADCS_SET_PID:  on_adcs_pid,
        CMD_ADCS_WHEEL_VEL: on_adcs_wheel_vel,
        CMD_ADCS_POWER_ON:  on_adcs_power_on,
        CMD_ADCS_POWER_OFF: on_adcs_power_off,
        CMD_ADCS_PING:      on_adcs_ping,
        CMD_RESET:         on_reset,
    }

    # ── Serial thread ─────────────────────────────────────────────────────────

    def serial_thread():
        port = initial_port
        disconnected = False
        rx_buf = bytearray()

        while True:
            if port is None:
                if not disconnected:
                    pkt_q.put(("# No Feather V2 connected — scanning…", 2))
                    serial_status[0] = "scanning…"
                    disconnected = True
                time.sleep(RESCAN_INTERVAL)
                port = find_feather()
                if port:
                    pkt_q.put((f"# Feather V2 found on {port}", 2))
                    disconnected = False
                continue

            try:
                with serial.Serial(port, BAUD, timeout=1) as ser:
                    serial_status[0] = f"connected ({port})"
                    while True:
                        while not cmd_q.empty():
                            ser.write((cmd_q.get_nowait() + "\n").encode())
                            ser.flush()
                        chunk = ser.read(max(1, ser.in_waiting or 1))
                        if chunk:
                            rx_buf.extend(chunk)
                            for parsed in extract_binary_frames(rx_buf):
                                pkt_q.put((parsed, None))
            except PermissionError:
                pkt_q.put((f"# Cannot claim {port} — port in use or permission denied. Retrying…", 4))
                serial_status[0] = "permission denied"
                time.sleep(RESCAN_INTERVAL)
            except serial.SerialException:
                serial_status[0] = "disconnected"
                port = None

    threading.Thread(target=serial_thread, daemon=True).start()

    while True:
        h, w = stdscr.getmaxyx()
        log_start = 1
        log_h = h - 2 - log_start

        # ── Drain packet queue ────────────────────────────────────────────────
        try:
            while True:
                line, color = pkt_q.get_nowait()

                if color is not None:
                    log.append([line, color])

                else:
                    parsed = line if isinstance(line, tuple) else parse_data_line(line)
                    if parsed:
                        pkt_id, raw, rssi, snr = parsed

                        if pkt_id == CMD_PING and len(raw) >= 8:
                            echoed, val = struct.unpack_from("<II", raw, 0)
                            if echoed in pending_pings:
                                idx, _ = pending_pings.pop(echoed)
                                ts = log[idx][0][1:9]
                                log[idx][0] = (f"[{ts}] PING #{echoed} → "
                                               f"val={val}  RSSI={rssi} SNR={snr}")
                                log[idx][1] = 5
                            else:
                                log.append([fmt_packet(pkt_id, raw, rssi, snr), 1])
                        elif pkt_id == ADCS_TELEMETRY and len(raw) >= 56:
                            ts = datetime.now().strftime("%H:%M:%S")
                            v = struct.unpack_from("<14f", raw, 0)
                            last_adcs_cur_quat = (v[4], v[5], v[6], v[7])
                            for line in fmt_adcs_telem(ts, raw, rssi, snr):
                                log.append([line, 1])
                        elif pkt_id == ADCS_PARAMS and len(raw) >= 39:
                            ts = datetime.now().strftime("%H:%M:%S")
                            for line in fmt_adcs_params(ts, raw, rssi, snr):
                                log.append([line, 2])
                        elif pkt_id in ack_dispatch:
                            ack_dispatch[pkt_id](raw, rssi, snr)
                        elif pkt_id == CAM_IMAGE_META and len(raw) >= 7:
                            img_transfer_id  = raw[0]
                            img_expected_len = struct.unpack_from("<I", raw, 1)[0]
                            img_total        = struct.unpack_from("<H", raw, 5)[0]
                            img_chunks       = {}
                            img_last_missing = None
                            img_have_new_chunks = False
                            img_last_nack_at = 0.0
                            ts = datetime.now().strftime("%H:%M:%S")
                            log.append([f"[{ts}] IMG META — id={img_transfer_id} {img_total} chunks, {img_expected_len} B expected", 3])
                            log.append([fmt_img_progress(ts, 0, img_total, img_expected_len), 3])
                            img_progress_idx = len(log) - 1
                        elif pkt_id == CAM_IMAGE_DATA and len(raw) >= 5:
                            transfer_id = raw[0]
                            seq   = struct.unpack_from("<H", raw, 1)[0]
                            total = struct.unpack_from("<H", raw, 3)[0]
                            chunk = raw[5:]
                            if img_transfer_id == transfer_id and total == img_total and seq < img_total:
                                prev = img_chunks.get(seq)
                                img_chunks[seq] = chunk
                                if prev != chunk:
                                    img_have_new_chunks = True
                                received = len(img_chunks)
                                ts = datetime.now().strftime("%H:%M:%S")
                                progress_msg = fmt_img_progress(ts, received, total, img_expected_len, seq, len(chunk))
                                if img_progress_idx is not None and img_progress_idx < len(log):
                                    log[img_progress_idx][0] = progress_msg
                                    log[img_progress_idx][1] = 3
                                else:
                                    log.append([progress_msg, 3])
                                    img_progress_idx = len(log) - 1
                            else:
                                ts = datetime.now().strftime("%H:%M:%S")
                                log.append([f"[{ts}] IMG DROP — id={transfer_id} seq={seq}/{total} ignored", 4])
                        elif pkt_id == CAM_IMAGE_DONE and len(raw) >= 1:
                            transfer_id = raw[0]
                            if transfer_id != img_transfer_id:
                                ts = datetime.now().strftime("%H:%M:%S")
                                log.append([f"[{ts}] IMG DONE — stale id={transfer_id}, current id={img_transfer_id}", 4])
                                continue

                            missing = [i for i in range(img_total) if i not in img_chunks]
                            ts = datetime.now().strftime("%H:%M:%S")
                            if not missing:
                                assembled = b"".join(img_chunks[i] for i in range(img_total))
                                save_dir = "../images"
                                if active_burst is not None and active_burst["saved"] < active_burst["count"]:
                                    save_dir = active_burst["folder"]
                                os.makedirs(save_dir, exist_ok=True)
                                timestamp = datetime.now().strftime('%H%M%S')
                                if active_burst is not None and active_burst["saved"] < active_burst["count"]:
                                    index = active_burst["saved"] + 1
                                    count = active_burst["count"]
                                    fname = os.path.join(save_dir, f"image_{index:02d}_of_{count:02d}_{timestamp}.jpg")
                                else:
                                    fname = os.path.join(save_dir, f"image_{timestamp}.jpg")
                                try:
                                    with open(fname, "wb") as f:
                                        f.write(assembled)
                                    if active_burst is not None and active_burst["saved"] < active_burst["count"]:
                                        active_burst["saved"] += 1
                                        if active_burst["saved"] >= active_burst["count"]:
                                            active_burst = None
                                    ts = datetime.now().strftime("%H:%M:%S")
                                    log.append([f"[{ts}] IMG SAVED — all {img_total}/{img_total} chunks received, "
                                                f"{len(assembled)} B written to {fname}", 1])
                                except OSError as e:
                                    ts = datetime.now().strftime("%H:%M:%S")
                                    log.append([f"[{ts}] IMG FAIL — could not save image: {e}", 4])
                                cmd_q.put(build_uplink(CAM_IMAGE_ACK, bytes([img_transfer_id])))
                                img_chunks = {}
                                img_transfer_id = None
                                img_total = 0
                                img_expected_len = 0
                                img_last_missing = None
                                img_have_new_chunks = False
                                img_last_nack_at = 0.0
                                img_progress_idx = None
                            else:
                                missing_key = tuple(missing)
                                should_send_nack = (
                                    img_last_missing != missing_key
                                    or img_have_new_chunks
                                    or time.time() - img_last_nack_at > 1.0
                                )
                                if should_send_nack:
                                    nack_data = bytes([img_transfer_id]) + b"".join(
                                        struct.pack("<H", s) for s in missing[:MAX_MISSING_SEQS_PER_NACK]
                                    )
                                    cmd_q.put(build_uplink(CAM_NACK, nack_data))
                                    log.append([f"[{ts}] IMG NACK — missing seq={missing}", 4])
                                    img_last_missing = missing_key
                                    img_have_new_chunks = False
                                    img_last_nack_at = time.time()
                        else:
                            log.append([fmt_packet(pkt_id, raw, rssi, snr), 1])

                        if write_api:
                            try:
                                write_point(write_api, pkt_id, raw, rssi, snr)
                            except Exception as e:
                                if _is_connection_refused(e):
                                    influx_status = "no influxd running"
                                    write_api = None
                                else:
                                    log.append([f"# InfluxDB error: {e}", 2])
                    else:
                        log.append([line, 2])

        except queue.Empty:
            pass

        # ── Timeouts ──────────────────────────────────────────────────────────
        now = time.time()
        if AUTO_PING_ENABLED and now - last_ping_time >= PING_INTERVAL_S:
            cmd_q.put(build_uplink(CMD_PING, struct.pack("<I", ping_arg)))
            ts = datetime.now().strftime("%H:%M:%S")
            log.append([f"[{ts}] PING #{ping_arg} → …", 3])
            pending_pings[ping_arg] = (len(log) - 1, now)
            ping_arg += 1
            last_ping_time = now
        for arg, (idx, sent_at) in list(pending_pings.items()):
            if now - sent_at > PING_TIMEOUT_S:
                log[idx][0] = log[idx][0].replace(" → …", " → NO ACK")
                log[idx][1] = 4
                del pending_pings[arg]
        for key, p in list(pending.items()):
            if now - p.sent_at > p.timeout:
                log[p.log_idx][0] = log[p.log_idx][0].replace(" → …", p.timeout_msg)
                log[p.log_idx][1] = 4
                del pending[key]

        # ── Draw ──────────────────────────────────────────────────────────────
        stdscr.erase()
        try:
            stdscr.addstr(0, 0, " " * (w - 1), curses.color_pair(2))
            serial_cp = 1 if serial_status[0].startswith("connected") else (3 if serial_status[0] == "scanning…" else 4)
            serial_text = f" Serial: {serial_status[0]}"
            stdscr.addstr(0, 0, serial_text[:w - 1], curses.color_pair(serial_cp))
            if influx_status:
                influx_cp = 1 if influx_status == "connected" else 4
                influx_text = f"  InfluxDB: {influx_status}"
                col = len(serial_text)
                if col + len(influx_text) < w - 1:
                    stdscr.addstr(0, col, influx_text, curses.color_pair(influx_cp))
        except curses.error:
            pass
        for i, (text, cp) in enumerate(log[-log_h:]):
            try:
                stdscr.addstr(log_start + i, 0, text[:w - 1], curses.color_pair(cp))
            except curses.error:
                pass

        try:
            stdscr.addstr(h - 2, 0, "─" * (w - 1), curses.color_pair(2))
            prompt = f">> {input_buf}"
            stdscr.addstr(h - 1, 0, prompt[:w - 1], curses.color_pair(1))
            stdscr.move(h - 1, min(len(prompt), w - 1))
        except curses.error:
            pass
        stdscr.refresh()

        # ── Input ─────────────────────────────────────────────────────────────
        ch = stdscr.getch()
        if ch == curses.KEY_RESIZE:
            pass
        elif ch in (curses.KEY_ENTER, 10, 13):
            cmd = input_buf.strip()
            if cmd:
                ts = datetime.now().strftime("%H:%M:%S")
                if cmd.lower() in ("help", "commands", "?"):
                    show_help(ts)
                elif cmd.lower() == "ping":
                    cmd_q.put(build_uplink(CMD_PING, struct.pack("<I", ping_arg)))
                    log.append([f"[{ts}] PING #{ping_arg} → …", 5])
                    pending_pings[ping_arg] = (len(log) - 1, time.time())
                    ping_arg += 1
                elif cmd.lower() == "deploy":
                    send_cmd(CMD_DEPLOY, b"", "DEPLOY → …",
                             "deploy", DEPLOY_TIMEOUT_S, " → NO RETURN PACKET")
                elif cmd.lower().startswith("setres") or cmd.lower().startswith("resolution"):
                    parts = cmd.lower().split()
                    if len(parts) != 2 or parts[1] not in CAMERA_RES_CODES:
                        valid = ", ".join(CAMERA_RES_CODES.keys())
                        log.append([f"[{ts}] CAM RES CMD invalid — use `setres <{valid}>`", 4])
                    else:
                        resolution = parts[1]
                        send_cmd(CMD_SET_CAMERA_RES, bytes([CAMERA_RES_CODES[resolution]]),
                                 f"CAM RES CMD sent — {resolution} → …", "cam_res")
                elif cmd.lower().startswith("photo"):
                    parts = cmd.split()
                    count = 1
                    spacing_s = 0.0
                    try:
                        if len(parts) >= 2:
                            count = int(parts[1])
                        if len(parts) >= 3:
                            spacing_s = float(parts[2])
                    except ValueError:
                        log.append([f"[{ts}] PHOTO CMD invalid — use `photo <count> <spacing_s>`", 4])
                        input_buf = ""
                        continue

                    spacing_ms = int(round(spacing_s * 1000.0))
                    if count <= 0 or spacing_ms < 0 or spacing_ms > 65535:
                        log.append([f"[{ts}] PHOTO CMD invalid — count must be > 0 and spacing <= 65.535 s", 4])
                    else:
                        payload = bytes([count & 0xFF]) + struct.pack("<H", spacing_ms)
                        if count > 1:
                            burst_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            pending_burst_request = {
                                "count": count,
                                "folder": os.path.join("../images", f"burst_{burst_stamp}"),
                            }
                        else:
                            pending_burst_request = None
                        send_cmd(CMD_TAKE_PHOTO, payload,
                                 f"PHOTO CMD sent — count={count} spacing={spacing_s:.3f} s → …",
                                 "photo")
                elif cmd.lower().startswith("adcs"):
                    parts = cmd.split()
                    sub = parts[1].lower() if len(parts) >= 2 else ""
                    if sub == "zero":
                        send_cmd(CMD_ADCS_ZERO, b"", "ADCS ZERO CMD sent → …", "adcs_zero")
                    elif sub == "enable":
                        valid = ("on", "off", "1", "0")
                        flags_raw = [p.lower() for p in parts[2:]]
                        if len(flags_raw) == 1 and flags_raw[0] in valid:
                            z_val = 1 if flags_raw[0] in ("on", "1") else 0
                            flags = [0, 0, z_val]
                        elif len(flags_raw) == 3 and all(p in valid for p in flags_raw):
                            flags = [1 if p in ("on", "1") else 0 for p in flags_raw]
                        else:
                            log.append([f"[{ts}] ADCS ENABLE CMD invalid — use `adcs enable <on|off>` (Z axis) or `adcs enable <x> <y> <z>`", 4])
                            input_buf = ""
                            continue
                        xs, ys, zs = ["ON" if f else "OFF" for f in flags]
                        send_cmd(CMD_ADCS_ENABLE, bytes(flags),
                                 f"ADCS ENABLE CMD sent — X={xs} Y={ys} Z={zs} → …", "adcs_enable")
                    elif sub == "set":
                        if len(parts) == 3 and parts[2].lower() == "current":
                            if last_adcs_cur_quat is None:
                                log.append([f"[{ts}] ADCS SET CURRENT — no ADCS telemetry received yet", 4])
                            else:
                                qw, qx, qy, qz = last_adcs_cur_quat
                                r, p, y = quat_to_euler(qw, qx, qy, qz)
                                send_cmd(CMD_ADCS_SETPOINT, struct.pack("<ffff", qw, qx, qy, qz),
                                         f"ADCS SET CURRENT CMD sent — "
                                         f"roll={r:.1f}° pitch={p:.1f}° yaw={y:.1f}° → …",
                                         "adcs")
                        else:
                            try:
                                if len(parts) == 3:
                                    roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, float(parts[2])
                                elif len(parts) == 5:
                                    roll_deg  = float(parts[2])
                                    pitch_deg = float(parts[3])
                                    yaw_deg   = float(parts[4])
                                else:
                                    raise ValueError
                            except ValueError:
                                log.append([f"[{ts}] ADCS SET CMD invalid — use `adcs set <yaw_deg>` or `adcs set <roll_deg> <pitch_deg> <yaw_deg>` or `adcs set current`", 4])
                            else:
                                qw, qx, qy, qz = euler_to_quat(roll_deg, pitch_deg, yaw_deg)
                                send_cmd(CMD_ADCS_SETPOINT, struct.pack("<ffff", qw, qx, qy, qz),
                                         f"ADCS SET CMD sent — "
                                         f"roll={roll_deg:.1f}° pitch={pitch_deg:.1f}° yaw={yaw_deg:.1f}° → …",
                                         "adcs")
                    elif sub == "pid":
                        AXIS_MAP = {"x": 0, "y": 1, "z": 2, "0": 0, "1": 1, "2": 2}
                        try:
                            if len(parts) == 5:
                                axis_n = 2  # default Z
                                kp = float(parts[2])
                                ki = float(parts[3])
                                kd = float(parts[4])
                            elif len(parts) == 6:
                                axis_s = parts[2].lower()
                                if axis_s not in AXIS_MAP:
                                    raise ValueError
                                axis_n = AXIS_MAP[axis_s]
                                kp = float(parts[3])
                                ki = float(parts[4])
                                kd = float(parts[5])
                            else:
                                raise ValueError
                        except ValueError:
                            log.append([f"[{ts}] ADCS PID CMD invalid — use `adcs pid <kp> <ki> <kd>` (Z) or `adcs pid <x|y|z> <kp> <ki> <kd>`", 4])
                        else:
                            axis_label = ("X", "Y", "Z")[axis_n]
                            send_cmd(CMD_ADCS_SET_PID, struct.pack("<Bfff", axis_n, kp, ki, kd),
                                     f"ADCS PID CMD sent — {axis_label} "
                                     f"Kp={kp:.6f} Ki={ki:.6f} Kd={kd:.6f} → …",
                                     "adcs_pid")
                    elif sub == "vel":
                        AXIS_MAP = {"x": 0, "y": 1, "z": 2, "0": 0, "1": 1, "2": 2}
                        try:
                            if len(parts) == 3:
                                axis_n = 2  # default Z
                                vel = float(parts[2])
                            elif len(parts) == 4:
                                axis_s = parts[2].lower()
                                if axis_s not in AXIS_MAP:
                                    raise ValueError
                                axis_n = AXIS_MAP[axis_s]
                                vel = float(parts[3])
                            else:
                                raise ValueError
                        except ValueError:
                            log.append([f"[{ts}] ADCS VEL CMD invalid — use `adcs vel <vel_rad_s>` (Z) or `adcs vel <x|y|z> <vel_rad_s>`", 4])
                        else:
                            axis_label = ("X", "Y", "Z")[axis_n]
                            send_cmd(CMD_ADCS_WHEEL_VEL, struct.pack("<Bf", axis_n, vel),
                                     f"ADCS WHEEL VEL CMD sent — {axis_label}={vel:.3f} rad/s → …",
                                     "adcs_wheel_vel")
                    elif sub == "poweron":
                        send_cmd(CMD_ADCS_POWER_ON, b"", "ADCS POWER ON CMD sent → …", "adcs_power_on")
                    elif sub == "poweroff":
                        send_cmd(CMD_ADCS_POWER_OFF, b"", "ADCS POWER OFF CMD sent → …", "adcs_power_off")
                    elif sub == "ping":
                        send_cmd(CMD_ADCS_PING, b"", "ADCS PING CMD sent → …", "adcs_ping")
                    else:
                        log.append([f"[{ts}] ADCS CMD invalid — use `adcs set/set current/enable/zero/pid/vel/poweron/poweroff/ping`", 4])
                elif cmd.lower().startswith("ctrl5v"):
                    parts = cmd.lower().split()
                    if len(parts) != 2 or parts[1] not in ("on", "off", "1", "0"):
                        log.append([f"[{ts}] CTRL5V CMD invalid — use `ctrl5v on/off`", 4])
                    else:
                        state = 1 if parts[1] in ("on", "1") else 0
                        send_cmd(CMD_CTRL_5V, bytes([state]),
                                 f"CTRL5V CMD sent — {'ON' if state else 'OFF'} → …", "ctrl5v")
                elif cmd.lower() == "reset":
                    send_cmd(CMD_RESET, b"", "RESET CMD sent → …",
                             "reset", PING_TIMEOUT_S, " → NO ACK")
                else:
                    log.append([f">> {cmd}", 5])
            input_buf = ""
        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            input_buf = input_buf[:-1]
        elif 32 <= ch < 127:
            input_buf += chr(ch)

        curses.napms(20)

# ── Entry point ─────────────────────────────────────────────────────────────────

def main():
    port = find_feather()
    if not port:
        print("Feather V2 not found. Plug in the ground station radio and retry.")
        sys.exit(1)

    print(f"Feather V2 on {port}")
    print(f"Auto-ping {'enabled' if AUTO_PING_ENABLED else 'disabled'} "
          f"(set AUTO_PING_ENABLED=0 to disable).")

    write_api = None
    influx_status = ""
    if INFLUX_TOKEN:
        try:
            client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
            created = ensure_bucket(client)
            print(f"InfluxDB bucket '{INFLUX_BUCKET}' {'created' if created else 'exists'}.")
            write_api = client.write_api(write_options=SYNCHRONOUS)
            influx_status = "connected"
        except Exception as e:
            if _is_connection_refused(e):
                print("no influxd running — InfluxDB logging disabled.")
                influx_status = "no influxd running"
            else:
                raise
    else:
        print("INFLUX_TOKEN not set — InfluxDB logging disabled.")

    input("Press Enter to launch terminal UI…")
    curses.wrapper(run_ui, port, write_api, influx_status)

if __name__ == "__main__":
    main()
