"""
ESP32 Channel Hopper — Experiment Data Collector
=================================================
Reads binary serial packets from the ESP32 (both RR and AD formats),
decodes them, and logs structured data for downstream analysis.

Experiment protocol:
  Phase A — Round-Robin baseline (configurable duration)
  Phase B — Adaptive w/ 3 s cycle  (configurable duration)
  Phase C — Adaptive w/ 5 s cycle  (configurable duration)
  Phase D — Adaptive w/ 7 s cycle  (configurable duration)

The host-side runner sends a single-byte command to the ESP32 to switch
modes and cycle times. (Requires the firmware experiment patch.)

Output files:
  data/raw_packets.jsonl   — one JSON line per decoded packet
  data/cycles.csv          — one row per complete cycle
  data/channels.csv        — one row per channel per cycle (long format)
  data/experiment_log.txt  — human-readable event log
"""

import serial
import serial.tools.list_ports
import struct
import json
import csv
import time
import os
import sys
import argparse
import threading
import queue
from datetime import datetime, timezone
from dataclasses import dataclass, asdict
from typing import Optional
from enum import IntEnum

# ─── Constants matching firmware ─────────────────────────────────────────────

SERIAL_MAGIC   = 0xAB
PKT_TYPE_RR    = 0x01
PKT_TYPE_AD    = 0x02
NUM_CHANNELS   = 13

# Experiment control bytes sent TO the ESP32
CMD_MODE_RR    = 0x10
CMD_MODE_AD    = 0x20
CMD_CYCLE_3S   = 0x31
CMD_CYCLE_5S   = 0x35
CMD_CYCLE_7S   = 0x37

# ─── Packet structures ────────────────────────────────────────────────────────

# RR_SerialPacket layout (little-endian, packed):
#   uint8  magic
#   uint8  type
#   uint32 cycleNum
#   uint32 cycleTotal
#   uint32 allTimeTotal
#   [13 x (uint32 packets, float pps)]
RR_HEADER_FMT  = "<BBIII"
RR_HEADER_SIZE = struct.calcsize(RR_HEADER_FMT)       # 14 bytes
RR_ENTRY_FMT   = "<If"
RR_ENTRY_SIZE  = struct.calcsize(RR_ENTRY_FMT)        # 8 bytes
RR_TOTAL_SIZE  = RR_HEADER_SIZE + NUM_CHANNELS * RR_ENTRY_SIZE   # 118 bytes

@dataclass
class RRPacket:
    ts:           float
    mode:         str
    cycle_time_ms: int
    cycle_num:    int
    cycle_total:  int
    all_time_total: int
    channels:     list   # list of dicts: {ch, packets, pps}

# AD packet: nanopb-encoded protobuf. We decode it manually using the
# known wire format (field numbers from CycleStruct.proto).
# For robustness we use a length-prefixed framing: the firmware sends
# a 2-byte LE uint16 length before the protobuf payload.
AD_LEN_FMT  = "<H"
AD_LEN_SIZE = struct.calcsize(AD_LEN_FMT)   # 2 bytes

@dataclass
class ADPacket:
    ts:            float
    mode:          str
    cycle_time_ms: int
    cycle_num:     int
    cycle_total:   int
    all_time_total: int
    global_mean:   float
    global_std_dev: float
    channels:      list   # list of dicts: {ch, avg_pps, dwell_ms, z_score}


# ─── Protobuf varint / wire-type decoder (minimal) ───────────────────────────

def _read_varint(data: bytes, pos: int):
    result, shift = 0, 0
    while pos < len(data):
        b = data[pos]; pos += 1
        result |= (b & 0x7F) << shift
        shift += 7
        if not (b & 0x80):
            break
    return result, pos

def _decode_fixed32(data: bytes, pos: int):
    val = struct.unpack_from("<I", data, pos)[0]
    return val, pos + 4

def _decode_float(data: bytes, pos: int):
    val = struct.unpack_from("<f", data, pos)[0]
    return val, pos + 4

def decode_ad_protobuf(payload: bytes) -> dict:
    """Minimal hand-rolled protobuf decoder for AD_SerialPacket."""
    pos = 0
    msg = {
        "magic": 0, "type": 0, "cycle_num": 0,
        "cycle_total": 0, "all_time_total": 0,
        "global_mean": 0.0, "global_std_dev": 0.0,
        "channels": []
    }
    field_map = {
        1: "magic", 2: "type", 3: "cycle_num",
        4: "cycle_total", 5: "all_time_total",
        6: "global_mean", 7: "global_std_dev"
    }
    while pos < len(payload):
        tag_wire, pos = _read_varint(payload, pos)
        field_num = tag_wire >> 3
        wire_type = tag_wire & 0x07

        if field_num in (1, 2, 3, 4, 5) and wire_type == 0:
            val, pos = _read_varint(payload, pos)
            msg[field_map[field_num]] = val
        elif field_num in (6, 7) and wire_type == 5:
            val, pos = _decode_float(payload, pos)
            msg[field_map[field_num]] = val
        elif field_num == 8 and wire_type == 2:
            # Embedded AD_ChannelEntry message
            sub_len, pos = _read_varint(payload, pos)
            sub_data = payload[pos:pos + sub_len]; pos += sub_len
            ch_entry = {"avg_pps": 0.0, "dwell_ms": 0, "z_score": 0.0}
            sp = 0
            while sp < len(sub_data):
                stag, sp = _read_varint(sub_data, sp)
                sf = stag >> 3; sw = stag & 7
                if sf == 1 and sw == 5:
                    ch_entry["avg_pps"], sp = _decode_float(sub_data, sp)
                elif sf == 2 and sw == 0:
                    ch_entry["dwell_ms"], sp = _read_varint(sub_data, sp)
                elif sf == 3 and sw == 5:
                    ch_entry["z_score"], sp = _decode_float(sub_data, sp)
                else:
                    break
            msg["channels"].append(ch_entry)
        else:
            # Unknown field — skip
            if wire_type == 0:
                _, pos = _read_varint(payload, pos)
            elif wire_type == 5:
                pos += 4
            elif wire_type == 1:
                pos += 8
            elif wire_type == 2:
                sub_len, pos = _read_varint(payload, pos)
                pos += sub_len
            else:
                break  # can't recover
    return msg


# ─── Serial packet reader ─────────────────────────────────────────────────────

class PacketReader:
    """
    Reads from the serial port, identifies packet boundaries by the
    SERIAL_MAGIC byte, and dispatches decoded packets to a queue.

    RR packets are fixed-size structs (118 bytes).
    AD packets are preceded by a 2-byte length field after the magic+type.
    """

    def __init__(self, port: str, baud: int, pkt_queue: queue.Queue,
                 mode: str = "rr", cycle_time_ms: int = 10000):
        self.ser          = serial.Serial(port, baud, timeout=0.1)
        self.pkt_queue    = pkt_queue
        self.mode         = mode            # "rr" | "ad"
        self.cycle_time_ms = cycle_time_ms
        self._buf         = b""
        self._stop        = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)

    def start(self): self._thread.start()
    def stop(self):  self._stop.set(); self._thread.join(timeout=2)

    def send_cmd(self, cmd_byte: int):
        self.ser.write(bytes([cmd_byte]))
        self.ser.flush()

    def _run(self):
        while not self._stop.is_set():
            chunk = self.ser.read(256)
            if chunk:
                self._buf += chunk
                self._parse()

    def _parse(self):
        while len(self._buf) >= 2:
            idx = self._buf.find(bytes([SERIAL_MAGIC]))
            if idx == -1:
                self._buf = b""
                return
            if idx > 0:
                self._buf = self._buf[idx:]

            # Need at least magic + type
            if len(self._buf) < 2:
                return

            pkt_type = self._buf[1]

            if pkt_type == PKT_TYPE_RR:
                if len(self._buf) < RR_TOTAL_SIZE:
                    return   # wait for more
                raw = self._buf[:RR_TOTAL_SIZE]
                self._buf = self._buf[RR_TOTAL_SIZE:]
                self._decode_rr(raw)

            elif pkt_type == PKT_TYPE_AD:
                # magic(1) + type(1) + length(2) + payload(length)
                if len(self._buf) < 4:
                    return
                ad_len = struct.unpack_from("<H", self._buf, 2)[0]
                total  = 4 + ad_len
                if len(self._buf) < total:
                    return
                payload    = self._buf[4:total]
                self._buf  = self._buf[total:]
                self._decode_ad(payload)

            else:
                # Unknown type — skip this magic byte and retry
                self._buf = self._buf[1:]

    def _decode_rr(self, raw: bytes):
        ts = time.time()
        magic, ptype, cycle_num, cycle_total, all_time_total = \
            struct.unpack_from(RR_HEADER_FMT, raw)
        channels = []
        for ch in range(NUM_CHANNELS):
            offset = RR_HEADER_SIZE + ch * RR_ENTRY_SIZE
            pkts, pps = struct.unpack_from(RR_ENTRY_FMT, raw, offset)
            channels.append({"ch": ch + 1, "packets": pkts, "pps": pps})
        pkt = RRPacket(
            ts=ts, mode="rr", cycle_time_ms=self.cycle_time_ms,
            cycle_num=cycle_num, cycle_total=cycle_total,
            all_time_total=all_time_total, channels=channels
        )
        self.pkt_queue.put(pkt)

    def _decode_ad(self, payload: bytes):
        ts  = time.time()
        msg = decode_ad_protobuf(payload)
        channels = []
        for i, ch in enumerate(msg["channels"]):
            channels.append({
                "ch": i + 1,
                "avg_pps":  ch.get("avg_pps", 0.0),
                "dwell_ms": ch.get("dwell_ms", 0),
                "z_score":  ch.get("z_score", 0.0),
            })
        pkt = ADPacket(
            ts=ts, mode="ad", cycle_time_ms=self.cycle_time_ms,
            cycle_num=msg["cycle_num"], cycle_total=msg["cycle_total"],
            all_time_total=msg["all_time_total"],
            global_mean=msg["global_mean"],
            global_std_dev=msg["global_std_dev"],
            channels=channels
        )
        self.pkt_queue.put(pkt)


# ─── Experiment orchestrator ──────────────────────────────────────────────────

PHASES = [
    # (mode, cycle_ms, (cmd_bytes...), description)
    # Commands are sent as separate bytes — never ORed together
    ("rr",  10000, (CMD_MODE_RR,),                        "Round-Robin baseline (10 s cycle)"),
    ("ad",   3000, (CMD_MODE_AD, CMD_CYCLE_3S),            "Adaptive 3 s cycle"),
    ("ad",   5000, (CMD_MODE_AD, CMD_CYCLE_5S),            "Adaptive 5 s cycle"),
    ("ad",   7000, (CMD_MODE_AD, CMD_CYCLE_7S),            "Adaptive 7 s cycle"),
]

class Experiment:
    def __init__(self, port: str, baud: int, phase_duration_s: int,
                 warmup_cycles: int, out_dir: str):
        self.port             = port
        self.baud             = baud
        self.phase_duration_s = phase_duration_s
        self.warmup_cycles    = warmup_cycles
        self.out_dir          = out_dir
        self.pkt_queue        = queue.Queue()
        self.reader: Optional[PacketReader] = None

        os.makedirs(out_dir, exist_ok=True)
        self.raw_fh    = open(f"{out_dir}/raw_packets.jsonl", "w")
        self.cycles_fh = open(f"{out_dir}/cycles.csv", "w", newline="")
        self.chan_fh   = open(f"{out_dir}/channels.csv", "w", newline="")
        self.log_fh    = open(f"{out_dir}/experiment_log.txt", "w")

        self.cycles_writer = csv.DictWriter(self.cycles_fh, fieldnames=[
            "ts", "phase", "mode", "cycle_time_ms",
            "cycle_num", "cycle_total", "all_time_total",
            "global_mean", "global_std_dev", "packets_per_sec_cycle"
        ])
        self.cycles_writer.writeheader()

        self.chan_writer = csv.DictWriter(self.chan_fh, fieldnames=[
            "ts", "phase", "mode", "cycle_time_ms", "cycle_num",
            "ch", "packets", "pps", "avg_pps", "dwell_ms", "z_score"
        ])
        self.chan_writer.writeheader()

    def log(self, msg: str):
        ts = datetime.now(timezone.utc).isoformat()
        line = f"[{ts}] {msg}"
        print(line)
        self.log_fh.write(line + "\n")
        self.log_fh.flush()

    def _flush_queue(self, phase_name: str, mode: str, cycle_time_ms: int,
                     deadline: float, skip_n: int):
        """Drain queue until deadline, writing all decoded packets."""
        collected = 0
        skipped   = 0
        while time.time() < deadline:
            try:
                pkt = self.pkt_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # Write raw
            d = asdict(pkt)
            d["phase"] = phase_name
            self.raw_fh.write(json.dumps(d) + "\n")
            self.raw_fh.flush()

            if skipped < skip_n:
                skipped += 1
                continue

            # Write cycle summary
            if isinstance(pkt, RRPacket):
                cycle_dur_s = cycle_time_ms / 1000.0
                row = {
                    "ts": pkt.ts, "phase": phase_name, "mode": "rr",
                    "cycle_time_ms": cycle_time_ms,
                    "cycle_num": pkt.cycle_num,
                    "cycle_total": pkt.cycle_total,
                    "all_time_total": pkt.all_time_total,
                    "global_mean": "",
                    "global_std_dev": "",
                    "packets_per_sec_cycle": round(pkt.cycle_total / cycle_dur_s, 4)
                }
                self.cycles_writer.writerow(row)
                self.cycles_fh.flush()
                for ch in pkt.channels:
                    self.chan_writer.writerow({
                        "ts": pkt.ts, "phase": phase_name, "mode": "rr",
                        "cycle_time_ms": cycle_time_ms,
                        "cycle_num": pkt.cycle_num, "ch": ch["ch"],
                        "packets": ch["packets"], "pps": ch["pps"],
                        "avg_pps": "", "dwell_ms": cycle_time_ms // NUM_CHANNELS,
                        "z_score": ""
                    })
                self.chan_fh.flush()

            elif isinstance(pkt, ADPacket):
                cycle_dur_s = cycle_time_ms / 1000.0
                row = {
                    "ts": pkt.ts, "phase": phase_name, "mode": "ad",
                    "cycle_time_ms": cycle_time_ms,
                    "cycle_num": pkt.cycle_num,
                    "cycle_total": pkt.cycle_total,
                    "all_time_total": pkt.all_time_total,
                    "global_mean": round(pkt.global_mean, 6),
                    "global_std_dev": round(pkt.global_std_dev, 6),
                    "packets_per_sec_cycle": round(pkt.cycle_total / cycle_dur_s, 4)
                }
                self.cycles_writer.writerow(row)
                self.cycles_fh.flush()
                for ch in pkt.channels:
                    self.chan_writer.writerow({
                        "ts": pkt.ts, "phase": phase_name, "mode": "ad",
                        "cycle_time_ms": cycle_time_ms,
                        "cycle_num": pkt.cycle_num, "ch": ch["ch"],
                        "packets": "",
                        "pps": "",
                        "avg_pps": round(ch["avg_pps"], 6),
                        "dwell_ms": ch["dwell_ms"],
                        "z_score": round(ch["z_score"], 6),
                    })
                self.chan_fh.flush()

            collected += 1
        return collected

    def run(self):
        self.log("=== Experiment start ===")
        self.log(f"Port={self.port} Baud={self.baud} "
                 f"Phase duration={self.phase_duration_s}s "
                 f"Warmup cycles={self.warmup_cycles}")

        self.reader = PacketReader(self.port, self.baud, self.pkt_queue)
        self.reader.start()

        for phase_idx, (mode, cycle_ms, cmd, desc) in enumerate(PHASES):
            phase_name = f"P{phase_idx+1}_{mode}_{cycle_ms}ms"
            self.log(f"--- Phase {phase_idx+1}: {desc} ---")

            # Send command bytes to ESP32 (each byte separately, small gap between)
            self.reader.mode          = mode
            self.reader.cycle_time_ms = cycle_ms
            for cmd_byte in cmd:
                self.reader.send_cmd(cmd_byte)
                time.sleep(0.05)  # give firmware time to process each byte
            time.sleep(0.5)   # let ESP32 settle into new mode

            # Drain stale packets from before command
            while not self.pkt_queue.empty():
                try: self.pkt_queue.get_nowait()
                except: pass

            deadline = time.time() + self.phase_duration_s
            n = self._flush_queue(phase_name, mode, cycle_ms,
                                  deadline, skip_n=self.warmup_cycles)
            self.log(f"  Collected {n} cycles (after {self.warmup_cycles} warmup skipped)")

        self.reader.stop()
        self.raw_fh.close()
        self.cycles_fh.close()
        self.chan_fh.close()
        self.log("=== Experiment complete ===")
        self.log_fh.close()
        print(f"\nData written to: {self.out_dir}/")


# ─── Port auto-detection ─────────────────────────────────────────────────────

# USB vendor/product IDs commonly used by ESP32 boards
ESP32_KNOWN_VIDS = {
    0x10C4,  # Silicon Labs CP210x (most common ESP32 USB-UART)
    0x1A86,  # CH340 / CH341 (cheap clone boards)
    0x0403,  # FTDI FT232
    0x239A,  # Adafruit
    0x303A,  # Espressif native USB (ESP32-S2/S3)
}

def auto_detect_port() -> str:
    """
    Scan connected serial ports and return the most likely ESP32 port.
    Preference order:
      1. Known ESP32 USB-UART VID
      2. Port description containing 'CP210', 'CH340', 'FTDI', or 'ESP'
      3. First available USB serial port
    Raises RuntimeError if nothing plausible is found.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Is the ESP32 plugged in?")

    # Pass 1 — known VIDs
    for p in ports:
        if p.vid in ESP32_KNOWN_VIDS:
            return p.device

    # Pass 2 — description heuristic
    keywords = ("cp210", "ch340", "ch341", "ftdi", "esp", "uart", "usb serial")
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in keywords):
            return p.device

    # Pass 3 — first USB port (non-Bluetooth)
    for p in ports:
        desc = (p.description or "").lower()
        if "bluetooth" not in desc:
            return p.device

    raise RuntimeError(
        f"Could not identify an ESP32 among ports: "
        f"{[p.device for p in ports]}\n"
        f"Use --port to specify manually."
    )


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ESP32 channel hopper experiment runner"
    )
    parser.add_argument("--port", default=None,
                        help="Serial port. If omitted, auto-detected.")
    parser.add_argument("--baud",     type=int, default=115200)
    parser.add_argument("--duration", type=int, default=300,
                        help="Phase duration in seconds (default: 300 = 5 min)")
    parser.add_argument("--warmup",   type=int, default=3,
                        help="Warmup cycles to skip per phase (default: 3)")
    parser.add_argument("--out",      default="data",
                        help="Output directory (default: ./data)")
    args = parser.parse_args()

    if args.port is None:
        try:
            args.port = auto_detect_port()
            print(f"Auto-detected ESP32 on: {args.port}")
        except RuntimeError as e:
            print(f"ERROR: {e}")
            sys.exit(1)
    else:
        print(f"Using port: {args.port}")

    exp = Experiment(
        port=args.port,
        baud=args.baud,
        phase_duration_s=args.duration,
        warmup_cycles=args.warmup,
        out_dir=args.out,
    )
    try:
        exp.run()
    except KeyboardInterrupt:
        print("\nInterrupted — flushing and closing files.")
        if exp.reader:
            exp.reader.stop()
        exp.raw_fh.close()
        exp.cycles_fh.close()
        exp.chan_fh.close()
        exp.log_fh.close()


if __name__ == "__main__":
    main()