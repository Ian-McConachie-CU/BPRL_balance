#!/usr/bin/env python3
"""
BPRL_Balance CAN Tools — CAN bus status and ID scanner.

Works on any firmware build.

Usage:
    python3 tools/can_tools.py                         # default: CAN status
    python3 tools/can_tools.py can-status              # FDCAN1/2 protocol status and error counters
    python3 tools/can_tools.py can-scan [--duration N] # scan all CAN IDs on bus 1 and show Hz breakdown

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args

from rich.table import Table

# ── PSR / ECR decode helpers ──────────────────────────────────────────────────

_LEC_NAMES = ["NoError","Stuff","Form","Ack","Bit1","Bit0","CRC","NoChange"]
_ACT_NAMES = ["Synchronizing","Idle","Receiver","Transmitter"]


def _decode_psr(psr: int) -> str:
    lec   = _LEC_NAMES[psr & 0x7]
    act   = _ACT_NAMES[(psr >> 3) & 0x3]
    ep    = bool(psr & (1 << 5))
    ew    = bool(psr & (1 << 6))
    bo    = bool(psr & (1 << 7))
    pxe   = bool(psr & (1 << 14))
    flags = []
    if bo:  flags.append("BUS-OFF")
    if ep:  flags.append("ErrorPassive")
    if ew:  flags.append("ErrorWarning")
    if pxe: flags.append("ProtoExcept")
    flag_str = " ".join(flags) if flags else "OK"
    return f"ACT={act}  LEC={lec}  {flag_str}"


def _decode_ecr(ecr: int) -> str:
    tec = ecr & 0xFF
    rec = (ecr >> 8) & 0x7F
    rp  = bool(ecr & (1 << 15))
    cel = (ecr >> 16) & 0xFF
    return f"TEC={tec}  REC={rec}{'(passive)' if rp else ''}  CEL={cel}"


def _decode_rxf0s(rxf0s: int) -> str:
    fill = rxf0s & 0x7F
    full = bool(rxf0s & (1 << 24))
    lost = bool(rxf0s & (1 << 25))
    return f"fill={fill}{'  FULL' if full else ''}{'  MSGS_LOST' if lost else ''}"


# ── CAN status ────────────────────────────────────────────────────────────────

def cmd_can_status(ser, _args):
    ser.write(b"CAN,status\r\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        m = re.match(
            r"CAN,STATUS,psr=(0x[0-9a-fA-F]+),ecr=(0x[0-9a-fA-F]+),"
            r"rxf0s=(0x[0-9a-fA-F]+),cccr=(0x[0-9a-fA-F]+)", line)
        if m:
            psr   = int(m.group(1), 16)
            ecr   = int(m.group(2), 16)
            rxf0s = int(m.group(3), 16)
            cccr  = int(m.group(4), 16)
            console.print("[bold]FDCAN1 Status[/bold]")
            console.print(f"  PSR   0x{psr:08x}  →  {_decode_psr(psr)}")
            console.print(f"  ECR   0x{ecr:08x}  →  {_decode_ecr(ecr)}")
            console.print(f"  RXF0S 0x{rxf0s:08x}  →  {_decode_rxf0s(rxf0s)}")
            console.print(f"  CCCR  0x{cccr:08x}  →  INIT={cccr&1}  MON={(cccr>>5)&1}  TEST={(cccr>>7)&1}")
            console.print()
            console.print("[dim]ACT=Idle + LEC=NoError/NoChange + TEC=0/REC=0 → bus healthy, IMX5 not transmitting[/dim]")
            console.print("[dim]ACT=Synchronizing → baud rate mismatch or no termination[/dim]")
            console.print("[dim]LEC=Ack + TEC>0 → FC transmitting but no ack (only one node, or wrong baud)[/dim]")
            console.print("[dim]LEC=Bit1/Form/CRC + REC>0 → IMX5 transmitting at different baud rate[/dim]")
            return
    console.print("[red]No CAN,STATUS response — firmware may not support this command[/red]")


# ── CAN scan ──────────────────────────────────────────────────────────────────

# Bus 1: RMD hip motor responses (0x241–0x244), SDC102 wheel responses (0x245–0x246),
#         IMX5 INS (registered dynamically by CAN.cpp)
# Bus 2: CAN IMU and current sensor (IDs TBD)
REGISTERED_IDS = {0x241, 0x242, 0x243, 0x244, 0x245, 0x246}


def cmd_can_scan(ser, args):
    duration = getattr(args, "duration", 1)

    ser.reset_input_buffer()
    ser.write(b"CAN,scan,start\r\n")
    deadline = time.monotonic() + 2.0
    started  = False
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if "started" in line:
            started = True
            break
    if not started:
        console.print("[red]No response to CAN,scan,start — is firmware running?[/red]")
        return

    console.print(f"[dim]Scanning all CAN IDs for {duration} s...[/dim]")
    time.sleep(duration)

    ser.write(b"CAN,scan,stop\r\n")
    entries  = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line == "CAN,SCAN,END":
            break
        m = re.match(r"CAN,SCAN,id=(EXT:)?(0x[0-9a-fA-F]+),count=(\d+)", line)
        if m:
            entries.append({
                "is_ext": m.group(1) is not None,
                "id_str": m.group(2),
                "id_int": int(m.group(2), 16),
                "count":  int(m.group(3)),
            })

    if not entries:
        console.print("[red]No CAN,SCAN results — no frames on bus or firmware too old.[/red]")
        return

    entries.sort(key=lambda e: e["count"], reverse=True)
    total = sum(e["count"] for e in entries)

    tbl = Table(
        title=f"CAN IDs seen in {duration} s  (total {total} frames = {total//duration} Hz)",
        show_header=True, header_style="bold")
    tbl.add_column("ID",          style="cyan", min_width=8)
    tbl.add_column("Type",        min_width=5)
    tbl.add_column("Frames",      justify="right", min_width=8)
    tbl.add_column("Hz",          justify="right", min_width=6)
    tbl.add_column("% total",     justify="right", min_width=8)
    tbl.add_column("Registered?", min_width=12)

    for e in entries:
        pct    = e["count"] / total * 100
        hz     = e["count"] / duration
        is_reg = e["id_int"] in REGISTERED_IDS
        reg_str = "[green]● yes[/green]" if is_reg else "[dim]○ no[/dim]"
        tbl.add_row(
            e["id_str"],
            "EXT" if e["is_ext"] else "STD",
            str(e["count"]),
            f"{hz:.0f}",
            f"{pct:.1f}%",
            reg_str,
        )

    console.print(tbl)
    unmatched_hz = sum(e["count"] for e in entries
                       if e["id_int"] not in REGISTERED_IDS) // duration
    if unmatched_hz > 10:
        ids = ", ".join(e["id_str"] for e in entries
                        if e["id_int"] not in REGISTERED_IDS)
        console.print(f"\n[yellow]{unmatched_hz} Hz of unregistered frames on: {ids}[/yellow]")
        console.print("[yellow]Register these IDs with bprl_can_register() to use their data.[/yellow]")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL CAN tools — bus status and ID scanner")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("can-status", help="Read FDCAN1 protocol status and error counters")

    scan_p = sub.add_parser("can-scan", help="Scan all CAN IDs on bus and show Hz breakdown")
    scan_p.add_argument("--duration", type=int, default=1,
                        help="Scan window in seconds (default: 1)")

    args = parser.parse_args()
    if args.command is None:
        args.command = "can-status"

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "can-status":
            cmd_can_status(ser, args)
        elif args.command == "can-scan":
            cmd_can_scan(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
