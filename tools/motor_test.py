#!/usr/bin/env python3
"""
BPRL_Balance CAN Motor Test — send torque/velocity commands to individual CAN motors
and read back their state (position, velocity, torque, temperature).

Works on any firmware build.

Usage:
    python3 tools/motor_test.py                        # default: motor test
    python3 tools/motor_test.py motor-test

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)

Motor layout:
    ID 1  Hip FL   — LKMTECH MG8016E-i6  (RMD protocol)
    ID 2  Hip FR   — LKMTECH MG8016E-i6
    ID 3  Hip RL   — LKMTECH MG8016E-i6
    ID 4  Hip RR   — LKMTECH MG8016E-i6
    ID 5  Wheel L  — Steadywin GIM6010-6 (SDC102 protocol)
    ID 6  Wheel R  — Steadywin GIM6010-6

Commands:
    status            — print current state of all 6 motors (from firmware)
    torque <id> <Nm>  — send a torque command to motor <id> (1-6)
    stop              — send zero torque to all motors
    quit              — exit
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args, send_cmd

MOTOR_LABELS = {
    1: "Hip FL  (RMD)",
    2: "Hip FR  (RMD)",
    3: "Hip RL  (RMD)",
    4: "Hip RR  (RMD)",
    5: "Wheel L (SDC102)",
    6: "Wheel R (SDC102)",
}


def cmd_motor_status(ser):
    """Request MOTOR,status from firmware and print the response."""
    ser.reset_input_buffer()
    send_cmd(ser, "MOTOR,status")
    deadline = time.monotonic() + 2.0
    lines = []
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line.startswith("MOTOR,"):
            lines.append(line)
        if line == "MOTOR,END":
            break

    if not lines:
        console.print("[red]No MOTOR,status response — is firmware running?[/red]")
        return

    console.print("[bold]CAN Motor State[/bold]")
    for line in lines:
        if line == "MOTOR,END":
            continue
        # Expected: MOTOR,<id>,pos=<rad>,vel=<rad/s>,tq=<Nm>,temp=<C>,valid=<0|1>
        m = re.match(
            r"MOTOR,(\d+),pos=([+-]?\d+\.?\d*),vel=([+-]?\d+\.?\d*),"
            r"tq=([+-]?\d+\.?\d*),temp=([+-]?\d+\.?\d*),valid=(\d)",
            line)
        if m:
            mid   = int(m.group(1))
            pos   = float(m.group(2))
            vel   = float(m.group(3))
            tq    = float(m.group(4))
            temp  = float(m.group(5))
            valid = bool(int(m.group(6)))
            label = MOTOR_LABELS.get(mid, f"Motor {mid}")
            vstyle = "green" if valid else "dim"
            console.print(
                f"  [{vstyle}]ID {mid} {label:20s}  "
                f"pos={pos:+8.3f} rad  vel={vel:+7.3f} rad/s  "
                f"tq={tq:+6.2f} Nm  temp={temp:5.1f} °C"
                f"{'  ● valid' if valid else '  ○ no data'}[/{vstyle}]")
        else:
            console.print(f"  [dim]{line}[/dim]")


def cmd_motor_test(ser, _args):
    # TODO: implement interactive CAN motor torque/velocity test loop.
    #   Planned commands:
    #     torque <id> <Nm>   — call can_motor_set_torque via USB command
    #     velocity <id> <rad/s> — call can_motor_set_velocity via USB command
    #     stop               — zero torque all motors
    #   Requires firmware-side USB command handlers for MOTOR,torque and MOTOR,velocity.
    #   For now, only 'status' is wired up on the firmware side.

    console.print("[bold yellow]CAN Motor Test[/bold yellow]")
    console.print("[dim]Commands: [cyan]status[/cyan]  |  [cyan]quit[/cyan][/dim]")
    console.print("[dim]  torque / velocity commands: TODO (firmware handlers not yet wired)[/dim]\n")

    try:
        while True:
            try:
                cmd = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd in ("quit", "q", "exit"):
                break

            if cmd in ("status", "s"):
                cmd_motor_status(ser)
                continue

            if cmd:
                console.print("[dim]Unknown command. Try: status | quit[/dim]")

    except KeyboardInterrupt:
        pass

    console.print("[green]Motor test ended.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL_Balance CAN motor test — status and torque commands")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("motor-test", help="Interactive CAN motor test")

    args = parser.parse_args()
    if args.command is None:
        args.command = "motor-test"

    ser = open_port(args.port, args.baud)
    try:
        cmd_motor_test(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
