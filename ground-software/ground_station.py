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
import re
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

FEATHER_V2_VID = 0x1A86  # WCH CH340 (Feather ESP32 V2)
ESP32_S3_VID   = 0x303A  # Espressif native USB (FC S3 DevKitC)

PING_TIMEOUT_S  = 4.0   # seconds before a sent ping is marked NO ACK
RESCAN_INTERVAL = 3.0   # seconds between port scans when disconnected

# Packet IDs — must match Common.h
CMD_PING      = 0
BMS_TELEMETRY = 101

MEAS_NAME = {
    CMD_PING:      "ping",
    BMS_TELEMETRY: "bms",
}

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

SENT_PING_RE = re.compile(r"# Sent CMD_PING arg=(\d+)")

def run_ui(stdscr, initial_port, write_api):
    curses.curs_set(1)
    stdscr.nodelay(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,    curses.COLOR_BLACK)

    # log is a list of [text, color_pair] — mutable so ping lines can be updated
    log       = []
    input_buf = ""
    pkt_q     = queue.Queue()
    cmd_q     = queue.Queue()

    # pending_pings: arg -> (log_idx, sent_time)
    pending_pings = {}

    def serial_thread():
        port = initial_port
        disconnected = False

        while True:
            if port is None:
                if not disconnected:
                    pkt_q.put(("# No Feather V2 connected — scanning…", 2))
                    disconnected = True
                time.sleep(RESCAN_INTERVAL)
                port = find_feather()
                if port:
                    pkt_q.put((f"# Feather V2 found on {port}", 2))
                    disconnected = False
                continue

            try:
                with serial.Serial(port, BAUD, timeout=1) as ser:
                    while True:
                        line = ser.readline().decode("utf-8", errors="replace").strip()
                        if line:
                            pkt_q.put((line, None))
                        while not cmd_q.empty():
                            ser.write((cmd_q.get_nowait() + "\n").encode())
            except PermissionError:
                pkt_q.put((f"# Cannot claim {port} — port in use or permission denied. Retrying…", 4))
                time.sleep(RESCAN_INTERVAL)
            except serial.SerialException:
                port = None

    threading.Thread(target=serial_thread, daemon=True).start()

    while True:
        h, w = stdscr.getmaxyx()
        log_h = h - 2

        # drain queue
        try:
            while True:
                line, color = pkt_q.get_nowait()

                if color is not None:
                    log.append([line, color])

                else:
                    # check for sent-ping line from Arduino
                    m = SENT_PING_RE.match(line)
                    if m:
                        arg = int(m.group(1))
                        ts  = datetime.now().strftime("%H:%M:%S")
                        log.append([f"[{ts}] PING #{arg} → …", 3])
                        pending_pings[arg] = (len(log) - 1, time.time())

                    else:
                        parsed = parse_data_line(line)
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
                            else:
                                log.append([fmt_packet(pkt_id, raw, rssi, snr), 1])

                            if write_api:
                                try:
                                    write_point(write_api, pkt_id, raw, rssi, snr)
                                except Exception as e:
                                    log.append([f"# InfluxDB error: {e}", 2])
                        else:
                            log.append([line, 2])

        except queue.Empty:
            pass

        # expire timed-out pings
        now = time.time()
        for arg, (idx, sent_at) in list(pending_pings.items()):
            if now - sent_at > PING_TIMEOUT_S:
                log[idx][0] = log[idx][0].replace(" → …", " → NO ACK")
                log[idx][1] = 4  # red
                del pending_pings[arg]

        # draw log
        stdscr.erase()
        for row, (text, cp) in enumerate(log[-log_h:]):
            try:
                stdscr.addstr(row, 0, text[:w - 1], curses.color_pair(cp))
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
            if input_buf.strip():
                cmd_q.put(input_buf.strip())
                log.append([f">> {input_buf}", 3])
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

    write_api = None
    if INFLUX_TOKEN:
        client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
        created = ensure_bucket(client)
        print(f"InfluxDB bucket '{INFLUX_BUCKET}' {'created' if created else 'exists'}.")
        write_api = client.write_api(write_options=SYNCHRONOUS)
    else:
        print("INFLUX_TOKEN not set — InfluxDB logging disabled.")

    input("Press Enter to launch terminal UI…")
    curses.wrapper(run_ui, port, write_api)

if __name__ == "__main__":
    main()
