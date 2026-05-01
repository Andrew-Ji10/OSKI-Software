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
import time
import struct
import curses
import threading
import queue
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

PING_INTERVAL_S  = 5.0   # seconds between auto-pings
PING_TIMEOUT_S   = 4.0   # seconds before a sent ping is marked NO ACK
DEPLOY_TIMEOUT_S = 6.0   # seconds before a sent deploy is marked NO RETURN PACKET
RESCAN_INTERVAL = 3.0   # seconds between port scans when disconnected

# Packet IDs — must match Common.h
CMD_PING       = 0
CMD_TAKE_PHOTO = 1
CMD_DEPLOY     = 3
BMS_TELEMETRY  = 101
CAM_IMAGE_META = 200
CAM_IMAGE_DATA = 201
CAM_IMAGE_DONE = 202
CAM_NACK       = 203
CAM_IMAGE_ACK  = 204

FRAME_SYNC = b"\xA5\x5A"
MAX_MISSING_SEQS_PER_NACK = 125

MEAS_NAME = {
    CMD_PING:       "ping",
    CMD_TAKE_PHOTO: "take_photo",
    CMD_DEPLOY:     "deploy",
    BMS_TELEMETRY:  "bms",
}

# LoRa addressing — must match ground station FW
FC_ADDR    = 0xAA
LOCAL_ADDR = 0xBB

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
        return (f"[{ts}] BMS — "
                f"{c1}/{c2}/{c3}/{c4} mV  stack={stack} mV  "
                f"current={current} mA  "
                f"intTemp={int_temp:.1f}°C  tsTemp={ts_temp:.1f}°C")
    return f"[{ts}] PKT id={pkt_id} len={len(raw)} {raw.hex()}"

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
        return {
            "cell1_mV": int(c1), "cell2_mV": int(c2),
            "cell3_mV": int(c3), "cell4_mV": int(c4),
            "stack_mV": int(stack), "current_mA": int(current),
            "intTemp_C": float(int_temp), "tsTemp_C": float(ts_temp),
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
# Color pairs: 1=green (data), 2=cyan (system), 3=yellow (ping/cmd), 4=red (no ack)

def run_ui(stdscr, initial_port, write_api, influx_status):
    curses.curs_set(1)
    stdscr.nodelay(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,    curses.COLOR_BLACK)

    # log is a list of [text, color_pair] — mutable so ping lines can be updated
    log           = []
    input_buf     = ""
    pkt_q         = queue.Queue()
    cmd_q         = queue.Queue()
    serial_status = ["scanning…"]  # mutable so serial_thread can update it

    # pending_pings: arg -> (log_idx, sent_time)
    pending_pings  = {}
    # pending_deploy: (log_idx, sent_time) or None
    pending_deploy = None
    ping_arg       = 0
    last_ping_time = 0.0  # trigger first ping immediately
    pending_burst_request = None
    active_burst = None

    # image reassembly state
    img_chunks       = {}   # seq (int) -> bytes
    img_transfer_id  = None
    img_total        = 0    # expected chunk count
    img_expected_len = 0    # expected byte count from META packet
    img_last_missing = None
    img_have_new_chunks = False
    img_last_nack_at = 0.0

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

        # drain queue
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
                                ts = log[idx][0][1:9]  # reuse original HH:MM:SS
                                log[idx][0] = (f"[{ts}] PING #{echoed} → "
                                               f"val={val}  RSSI={rssi} SNR={snr}")
                                log[idx][1] = 1  # green on ack
                            else:
                                log.append([fmt_packet(pkt_id, raw, rssi, snr), 1])
                        elif pkt_id == CMD_DEPLOY and pending_deploy is not None:
                            idx, _ = pending_deploy
                            pending_deploy = None
                            ts = log[idx][0][1:9]
                            if len(raw) >= 1 and raw[0] == 0:
                                log[idx][0] = f"[{ts}] DEPLOY ACK — sequence triggered  RSSI={rssi} SNR={snr}"
                                log[idx][1] = 1  # green
                            else:
                                log[idx][0] = f"[{ts}] DEPLOY — invalid command  RSSI={rssi} SNR={snr}"
                                log[idx][1] = 4  # red
                        elif pkt_id == CMD_TAKE_PHOTO:
                            ts = datetime.now().strftime("%H:%M:%S")
                            if len(raw) >= 1 and raw[0] == 0:
                                log.append([f"[{ts}] PHOTO ACK — triggered  RSSI={rssi} SNR={snr}", 1])
                                if pending_burst_request is not None:
                                    if pending_burst_request["count"] > 1:
                                        active_burst = {
                                            "folder": pending_burst_request["folder"],
                                            "count": pending_burst_request["count"],
                                            "saved": 0,
                                        }
                                        log.append([f"[{ts}] PHOTO BURST — saving to {active_burst['folder']}", 2])
                                    pending_burst_request = None
                            elif len(raw) >= 1 and raw[0] == 2:
                                log.append([f"[{ts}] PHOTO NACK — invalid photo count  RSSI={rssi} SNR={snr}", 4])
                                pending_burst_request = None
                            else:
                                log.append([f"[{ts}] PHOTO ACK — busy/error  RSSI={rssi} SNR={snr}", 4])
                                pending_burst_request = None
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
                                log.append([fmt_img_progress(ts, received, total, img_expected_len, seq, len(chunk)), 3])
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

        # auto-ping + expire timed-out pings / deploys
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
                log[idx][1] = 4  # red
                del pending_pings[arg]
        if pending_deploy is not None:
            idx, sent_at = pending_deploy
            if now - sent_at > DEPLOY_TIMEOUT_S:
                log[idx][0] = log[idx][0].replace(" → …", " → NO RETURN PACKET")
                log[idx][1] = 4  # red
                pending_deploy = None

        # draw status bar + log
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

        # draw separator + input
        try:
            stdscr.addstr(h - 2, 0, "─" * (w - 1), curses.color_pair(2))
            prompt = f">> {input_buf}"
            stdscr.addstr(h - 1, 0, prompt[:w - 1], curses.color_pair(1))
            stdscr.move(h - 1, min(len(prompt), w - 1))
        except curses.error:
            pass
        stdscr.refresh()

        # input
        ch = stdscr.getch()
        if ch == curses.KEY_RESIZE:
            pass
        elif ch in (curses.KEY_ENTER, 10, 13):
            cmd = input_buf.strip()
            if cmd:
                ts = datetime.now().strftime("%H:%M:%S")
                if cmd.lower() == "ping":
                    cmd_q.put(build_uplink(CMD_PING, struct.pack("<I", ping_arg)))
                    log.append([f"[{ts}] PING #{ping_arg} → …", 3])
                    pending_pings[ping_arg] = (len(log) - 1, time.time())
                    ping_arg += 1
                elif cmd.lower() == "deploy":
                    cmd_q.put(build_uplink(CMD_DEPLOY))
                    log.append([f"[{ts}] DEPLOY → …", 3])
                    pending_deploy = (len(log) - 1, time.time())
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
                        cmd_q.put(build_uplink(CMD_TAKE_PHOTO, payload))
                        if count > 1:
                            burst_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            pending_burst_request = {
                                "count": count,
                                "folder": os.path.join("../images", f"burst_{burst_stamp}"),
                            }
                        else:
                            pending_burst_request = None
                        log.append([f"[{ts}] PHOTO CMD sent — count={count} spacing={spacing_s:.3f} s", 3])
                else:
                    log.append([f">> {cmd}", 3])
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
