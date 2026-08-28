#!/usr/bin/env python3
"""
BPRL IMU Calibration — collect static bias, write to flash.

Requires a firmware built with -DBPRL_DEBUG.
The $IMU telemetry stream (emitted by DebugThread) is needed for data collection.

Usage:
    python3 tools/calibrate.py                        # default: calibrate (30 s collection)
    python3 tools/calibrate.py calibrate [--duration N]

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
    --duration N          Collection time in seconds (default: 30)
"""

import argparse
import time

from bprl_common import (
    console, open_port, add_port_args, SerialReader,
    parse_imu_line,
)

from rich.progress import Progress
from rich.table import Table


def cmd_calibrate(ser, args):
    duration = getattr(args, "duration", 30)
    reader   = SerialReader(ser)

    console.print("\n[bold]IMU Static Bias Calibration[/bold]")
    console.print("[yellow]Requires -DBPRL_DEBUG firmware build.[/yellow]")
    console.print("Place the drone on a [bold]level surface[/bold] and do not move it.")
    try:
        input("Press [Enter] to begin collection...")
    except EOFError:
        pass

    console.print(f"Collecting {duration} s of IMU data...")

    sums_gyro  = [[0.0]*3 for _ in range(3)]
    sums_accel = [[0.0]*3 for _ in range(3)]
    counts     = [0, 0, 0]
    t_end      = time.monotonic() + duration

    with Progress(transient=True) as prog:
        task = prog.add_task("Collecting...", total=duration)
        while time.monotonic() < t_end:
            for line in reader.pop_lines():
                s = parse_imu_line(line)
                if s is None:
                    continue
                for i in range(3):
                    if not s.valid[i]:
                        continue
                    counts[i] += 1
                    for k in range(3):
                        sums_gyro[i][k]  += s.gyro[i][k]
                        sums_accel[i][k] += s.accel[i][k]
            elapsed = duration - (t_end - time.monotonic())
            prog.update(task, completed=min(elapsed, duration))
            time.sleep(0.05)

    reader.stop()

    if min(counts) < 10:
        console.print(f"[red]Too few samples (got {counts}). Check connection and DEBUG build.")
        return

    gyro_bias  = [[sums_gyro[i][k]  / counts[i] for k in range(3)] for i in range(3)]
    accel_bias = [[sums_accel[i][k] / counts[i] for k in range(3)] for i in range(3)]
    for i in range(3):
        accel_bias[i][2] -= 9.80665

    tbl = Table(title="Computed Biases", show_header=True)
    tbl.add_column("IMU")
    tbl.add_column("gx (rad/s)", justify="right")
    tbl.add_column("gy (rad/s)", justify="right")
    tbl.add_column("gz (rad/s)", justify="right")
    tbl.add_column("ax (m/s²)", justify="right")
    tbl.add_column("ay (m/s²)", justify="right")
    tbl.add_column("az (m/s²)", justify="right")
    for i in range(3):
        tbl.add_row(
            f"IMU{i}",
            *[f"{gyro_bias[i][k]:+.5f}"  for k in range(3)],
            *[f"{accel_bias[i][k]:+.5f}" for k in range(3)],
        )
    console.print(tbl)
    console.print(f"  Samples: IMU0={counts[0]}  IMU1={counts[1]}  IMU2={counts[2]}")

    try:
        ans = input("\nWrite to flight controller flash? [y/N] ").strip().lower()
    except EOFError:
        ans = "n"
    if ans != "y":
        console.print("[yellow]Calibration not written.")
        return

    for i in range(3):
        gx, gy, gz = gyro_bias[i]
        ax, ay, az = accel_bias[i]
        cmd = f"CAL,set,{i},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f}"
        ser.write((cmd + "\n").encode())
        ser.flush()
        time.sleep(0.1)

    ser.write(b"CAL,commit\n")
    ser.flush()

    deadline = time.monotonic() + 3.0
    buf = b""
    ok_received = False
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                resp = line_b.decode("ascii", errors="replace").strip()
                if resp == "CAL,OK":
                    ok_received = True
                    break
        if ok_received:
            break
        time.sleep(0.05)

    if not ok_received:
        console.print("[red]No CAL,OK received — write may have failed.")
        return

    console.print("[green]Calibration written to flash.")

    ser.write(b"CAL,query\n")
    ser.flush()
    deadline = time.monotonic() + 3.0
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                resp = line_b.decode("ascii", errors="replace").strip()
                if resp.startswith("CAL,DATA,"):
                    console.print(f"[dim]Stored: {resp}")
                    return
        time.sleep(0.05)
    console.print("[yellow]No CAL,DATA response from query.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL IMU calibration — requires -DBPRL_DEBUG firmware build")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")

    cal_p = sub.add_parser("calibrate",
                            help="Collect IMU static bias and write to flash (DEBUG build required)")
    cal_p.add_argument("--duration", type=int, default=30,
                       help="Collection time in seconds (default: 30)")

    args = parser.parse_args()
    if args.command is None:
        args.command = "calibrate"
        args.duration = 30

    ser = open_port(args.port, args.baud)
    try:
        cmd_calibrate(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
