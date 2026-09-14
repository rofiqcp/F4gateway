#!/usr/bin/env python3
"""Capture XPT2046 touch coverage from F4gateway over USB CDC.

The tool is diagnostic-only: it never sends HMI button/action commands.
It polls TOUCH:STATUS, prints live pressure/coordinates, and saves CSV.
"""
import argparse
import csv
import glob
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import serial

STATUS_RE = re.compile(
    r"TOUCH:STATUS:READ=(?P<read>\d+):REJECT=(?P<reject>\d+):Z=(?P<z>\d+)"
    r":RAW=(?P<raw_x>\d+),(?P<raw_y>\d+):XY=(?P<x>\d+),(?P<y>\d+)"
    r":CS=(?P<cs>\d+):BUS=(?P<bus>\d+):PAGE=(?P<page>[^\r\n]+)"
)
def auto_device():
    pats = [
        "/dev/serial/by-id/*BLACKPILL_F411CE_CDC*",
    ]
    for pat in pats:
        found = sorted(glob.glob(pat))
        if found:
            return found[0]
    raise FileNotFoundError("F411 CDC port not found")


def verified_bridge_pids(device):
    try:
        out = subprocess.check_output(["fuser", device], text=True, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        return []
    pids = []
    for tok in out.split():
        if not tok.isdigit():
            continue
        pid = int(tok)
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "stmf4_hmi_bridge" in cmd:
            pids.append(pid)
    return pids
def take_port(device):
    pids = verified_bridge_pids(device)
    for pid in pids:
        os.kill(pid, signal.SIGTERM)
    if pids:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if not verified_bridge_pids(device):
                break
            time.sleep(0.05)
    return pids


def read_status(ser):
    ser.reset_input_buffer()
    ser.write(b"TOUCH:STATUS\n")
    ser.flush()
    deadline = time.monotonic() + 0.20
    while time.monotonic() < deadline:
        raw = ser.readline().decode(errors="replace").strip()
        match = STATUS_RE.search(raw)
        if match:
            row = match.groupdict()
            for key in ("read", "reject", "z", "raw_x", "raw_y", "x", "y", "cs", "bus"):
                row[key] = int(row[key])
            return row
    return None
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--threshold", type=int, default=200)
    ap.add_argument("--take-port", action="store_true",
                    help="temporarily terminate only stmf4_hmi_bridge if it owns CDC")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    device = args.device or auto_device()
    if args.take_port:
        stopped = take_port(device)
        if stopped:
            print(f"[probe] released bridge PID(s): {stopped}")
        time.sleep(0.15)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    output = Path(args.output or f"tools/records/touch_probe_{stamp}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ["t_s", "read", "reject", "z", "raw_x", "raw_y", "x", "y", "cs", "bus", "page", "accepted"]
    accepted_points = []
    total = 0
    misses = 0
    period = 1.0 / max(args.hz, 1.0)
    start = time.monotonic()
    next_tick = start

    print(f"[probe] device={device} duration={args.duration:.1f}s rate={args.hz:.1f}Hz threshold={args.threshold}")
    print("[probe] Sentuh seluruh area layar. Baris HIT = sentuhan valid menurut threshold.")
    with serial.Serial(device, 115200, timeout=0.05, write_timeout=0.2) as ser, output.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        while time.monotonic() - start < args.duration:
            row = read_status(ser)
            now = time.monotonic()
            if row is None:
                misses += 1
            else:
                row["t_s"] = round(now - start, 4)
                row["accepted"] = int(row["z"] >= args.threshold and 0 <= row["raw_x"] <= 4095 and 0 <= row["raw_y"] <= 4095)
                writer.writerow(row)
                total += 1
                if row["accepted"]:
                    accepted_points.append((row["x"], row["y"], row["raw_x"], row["raw_y"], row["z"]))
                    print(f"HIT t={row['t_s']:6.2f}s Z={row['z']:3d} RAW={row['raw_x']:4d},{row['raw_y']:4d} XY={row['x']:3d},{row['y']:3d} PAGE={row['page']}")
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    print(f"[probe] samples={total} misses={misses} hits={len(accepted_points)}")
    print(f"[probe] CSV={output}")
    if accepted_points:
        xs = [p[0] for p in accepted_points]
        ys = [p[1] for p in accepted_points]
        zs = [p[4] for p in accepted_points]
        print(f"[probe] screen coverage X={min(xs)}..{max(xs)} Y={min(ys)}..{max(ys)} Zmax={max(zs)}")
        print("[probe] Gunakan rentang XY tersebut sebagai dasar area tombol yang benar-benar terbaca.")
    else:
        print("[probe] Tidak ada HIT. Tekanan belum melewati threshold atau koordinat raw belum valid.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[probe] stopped")
    except Exception as exc:
        print(f"[probe] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
