#!/usr/bin/env python3
"""
BPRL_Balance Targets Test — live view of the processed system targets
(post-Radio.cpp normalization/inversion) and the robot state machine's
current mode, via the `TGT,status` USB command.

Unlike radio_test.py (which shows raw SBUS channel values straight off the
wire), this tool shows what the robot's control pipeline actually sees:
g_input[] targets, g_armed, and RobotStateMachine::mode() -- so it exercises
the full radio input plumbing end to end (SBUS -> Radio.cpp accessors,
including any inversions -> g_input[]/g_armed -> RobotStateMachine).

Works on any firmware build.

Usage:
    python3 tools/targets_test.py                # default: live targets monitor
    python3 tools/targets_test.py tgt-status

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
    --rate N              Poll rate in Hz (default 20)
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args

from rich.live import Live
from rich.table import Table
from rich.text import Text
from rich.panel import Panel

TGT_RE = re.compile(
    r"TGT,STATUS,armed=(\d),mode_sw=([+-]?\d+\.?\d*),mode=(\d),mode_name=(\w+),"
    r"vel_tgt=([+-]?\d+\.?\d*),yaw_tgt=([+-]?\d+\.?\d*),"
    r"height_tgt=([+-]?\d+\.?\d*),lean_tgt=([+-]?\d+\.?\d*)")

BAR_WIDTH = 30

MODE_STYLES = {
    "IDLE":      "dim",
    "BALANCING": "bold green",
    "CAR":       "bold yellow",
}


def parse_tgt_line(line: str):
    m = TGT_RE.match(line)
    if not m:
        return None
    return {
        "armed":      bool(int(m.group(1))),
        "mode_sw":    float(m.group(2)),
        "mode":       int(m.group(3)),
        "mode_name":  m.group(4),
        "vel_tgt":    float(m.group(5)),
        "yaw_tgt":    float(m.group(6)),
        "height_tgt": float(m.group(7)),
        "lean_tgt":   float(m.group(8)),
    }


def make_bar(value: float) -> Text:
    # value in [-1, 1]
    frac = max(-1.0, min(1.0, value))
    filled = int(round((frac + 1.0) / 2.0 * BAR_WIDTH))
    center_pos = BAR_WIDTH // 2

    bar = Text()
    for i in range(BAR_WIDTH):
        ch = "#" if i < filled else "-"
        style = "cyan" if i < filled else "dim"
        if i == center_pos:
            ch = "|"
            style = "yellow" if i >= filled else "dim yellow"
        bar.append(ch, style=style)
    return bar


def build_panel(state: dict) -> Panel:
    age   = time.monotonic() - state["last_rx"]
    stale = age > 1.0 or state["last_rx"] == 0.0

    arm_text = Text("● ARMED  " if state["armed"] else "○ disarmed", style="bold red" if state["armed"] else "green")
    mode_style = MODE_STYLES.get(state["mode_name"], "white")
    mode_text = Text(f"mode={state['mode_name']}", style=mode_style)
    mode_sw_text = Text(f"  (mode_sw={state['mode_sw']:+.3f})", style="dim")

    header = Table.grid(padding=(0, 2))
    header.add_column()
    header.add_column()
    header.add_column()
    header.add_row(arm_text, mode_text, mode_sw_text)

    tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 1))
    tbl.add_column("Target", min_width=16)
    tbl.add_column("Value", justify="right", min_width=8)
    tbl.add_column(f"-1{'':>{BAR_WIDTH-4}}+1", min_width=BAR_WIDTH)

    rows = [
        ("Velocity target", "vel_tgt"),
        ("Yaw target",      "yaw_tgt"),
        ("Height target",   "height_tgt"),
        ("Lean target",     "lean_tgt"),
    ]
    for label, key in rows:
        val = state[key]
        tbl.add_row(label, f"{val:+.3f}", make_bar(val))

    if not state["usb_rx_any"]:
        note = "  [dim](no USB response yet — check connection / firmware build)[/dim]"
    elif stale:
        note = "  [yellow](stale — no recent TGT,status reply)[/yellow]"
    else:
        note = ""
    title = f"BPRL System Targets — RobotStateMachine input pipeline{note}"

    body = Table.grid()
    body.add_row(header)
    body.add_row("")
    body.add_row(tbl)
    body.add_row("")
    body.add_row(Text(
        "height_tgt/lean_tgt are read but not yet consumed by any controller "
        "(needs FiveBarIK leg-length control, see controls_plan.md). CAR mode "
        "is reachable (mode switch low + armed) but STUBBED -- zero torque, "
        "same as IDLE, until wheel mixing + the car/balance transitions are "
        "implemented.", style="dim"))
    return Panel(body, title=title, border_style="blue")


def cmd_tgt_status(ser, args):
    rate = max(1, getattr(args, "rate", 20))
    period = 1.0 / rate

    state = {
        "armed": False, "mode_sw": 0.0, "mode": 0, "mode_name": "IDLE",
        "vel_tgt": 0.0, "yaw_tgt": 0.0, "height_tgt": 0.0, "lean_tgt": 0.0,
        "usb_rx_any": False, "last_rx": 0.0,
    }

    try:
        with Live(build_panel(state), refresh_per_second=rate,
                  console=console) as live:
            while True:
                t0 = time.monotonic()
                ser.reset_input_buffer()
                ser.write(b"TGT,status\r\n")
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    state["usb_rx_any"] = True
                    parsed = parse_tgt_line(line)
                    if parsed:
                        state.update(parsed)
                        state["last_rx"] = time.monotonic()
                        break
                live.update(build_panel(state))
                elapsed = time.monotonic() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL_Balance targets test — live system targets + state machine mode monitor")
    add_port_args(parser)
    parser.add_argument("--rate", type=int, default=20,
                         help="Poll rate in Hz (default: 20)")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("tgt-status", help="Live system targets monitor")

    args = parser.parse_args()
    if args.command is None:
        args.command = "tgt-status"

    ser = open_port(args.port, args.baud)
    try:
        cmd_tgt_status(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
