#!/usr/bin/env python3
"""
BPRL_Balance Radio Test — live dump of all 16 raw SBUS channels received on
TELEM2 (USART3), plus receiver frame_lost/failsafe flags and armed state.

Works on any firmware build (uses the RC,status USB command, not the
BPRL_DEBUG-only $TEL stream).

Usage:
    python3 tools/radio_test.py                # default: live channel monitor
    python3 tools/radio_test.py rc-status

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

# SBUS 11-bit channel range: 172 (min) .. 992 (center) .. 1811 (max)
SBUS_MIN, SBUS_CENTER, SBUS_MAX = 172, 992, 1811

# Channel order from the transmitter — see src/coms/Radio.hpp / README.md.
# This robot's channel layout is NOT the usual drone throttle/roll/pitch/yaw
# mapping. Channels 5/7/8/9 are physical Aux switches on the transmitter
# that exist but aren't wired to anything in firmware yet.
CHANNEL_LABELS = {
    0: "Yaw stick",
    1: "Vel target",
    2: "Height set*",
    3: "Leanover*",
    4: "Arm (AuxF)",
    5: "AuxA (unwired)",
    6: "Mode sel (AuxB)",
    7: "AuxH (unwired)",
    8: "AuxD (unwired)",
    9: "AuxC (unwired)",
}

RC_RE = re.compile(
    r"RC,STATUS,frame_lost=(\d),failsafe=(\d),armed=(\d)((?:,ch\d+=\d+)+)")
CH_RE = re.compile(r"ch(\d+)=(\d+)")

BAR_WIDTH = 30


def parse_rc_line(line: str):
    m = RC_RE.match(line)
    if not m:
        return None
    frame_lost = bool(int(m.group(1)))
    failsafe   = bool(int(m.group(2)))
    armed      = bool(int(m.group(3)))
    channels = [SBUS_CENTER] * 16
    for cm in CH_RE.finditer(m.group(4)):
        idx, val = int(cm.group(1)), int(cm.group(2))
        if 0 <= idx < 16:
            channels[idx] = val
    return frame_lost, failsafe, armed, channels


def make_bar(value: int) -> Text:
    frac = (value - SBUS_MIN) / (SBUS_MAX - SBUS_MIN)
    frac = max(0.0, min(1.0, frac))
    filled = int(round(frac * BAR_WIDTH))
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
    frame_lost = state["frame_lost"]
    failsafe   = state["failsafe"]
    armed      = state["armed"]
    channels   = state["channels"]
    age        = time.monotonic() - state["last_rx"]
    stale      = age > 1.0 or state["last_rx"] == 0.0

    tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 1))
    tbl.add_column("Ch", justify="right", min_width=3)
    tbl.add_column("Label", min_width=13)
    tbl.add_column("Raw", justify="right", min_width=5)
    tbl.add_column(f"{SBUS_MIN}{'':>{BAR_WIDTH-6}}{SBUS_MAX}", min_width=BAR_WIDTH)
    tbl.add_column("Norm", justify="right", min_width=8)

    for ch in range(16):
        raw   = channels[ch]
        label = CHANNEL_LABELS.get(ch, "")
        if ch == 4:
            norm = "ARMED" if raw > SBUS_CENTER else "safe"
        elif ch <= 9:
            # All other used/reserved channels (0,1,2,3,6 wired; 5,7,8,9
            # unwired Aux switches) are plain +/-1 sticks/switches -- still
            # shown for the unwired ones so you can confirm they move.
            norm = f"{(raw - SBUS_CENTER) / (SBUS_CENTER - SBUS_MIN):+.3f}"
        else:
            norm = ""
        label_style = "" if label else "dim"
        tbl.add_row(str(ch), Text(label, style=label_style), str(raw),
                    make_bar(raw), norm)

    flags = Text()
    flags.append("● FRAME_LOST  " if frame_lost else "○ frame ok    ",
                 style="bold red" if frame_lost else "green")
    flags.append("● FAILSAFE  " if failsafe else "○ no failsafe  ",
                 style="bold red" if failsafe else "green")
    flags.append("● ARMED" if armed else "○ disarmed",
                 style="bold red" if armed else "green")

    if not state["usb_rx_any"]:
        note = "  [dim](no USB response yet — check connection / firmware build)[/dim]"
    elif stale:
        note = "  [yellow](stale — no recent RC,status reply)[/yellow]"
    else:
        note = ""
    title = f"BPRL SBUS Radio — TELEM2/USART3{note}"

    body = Table.grid()
    body.add_row(flags)
    body.add_row("")
    body.add_row(tbl)
    body.add_row("")
    body.add_row(Text(
        "* ch2/ch3 (height set / leanover) are read but not yet consumed "
        "by any controller — needs leg-length control, see controls_plan.md. "
        "ch5/ch7/ch8/ch9 (Aux A/H/D/C) are physical switches not wired to "
        "anything yet.", style="dim"))
    return Panel(body, title=title, border_style="blue")


def cmd_rc_status(ser, args):
    rate = max(1, getattr(args, "rate", 20))
    period = 1.0 / rate

    state = {
        "frame_lost": True, "failsafe": True, "armed": False,
        "channels": [SBUS_CENTER] * 16,
        "usb_rx_any": False, "last_rx": 0.0,
    }

    try:
        with Live(build_panel(state), refresh_per_second=rate,
                  console=console) as live:
            while True:
                t0 = time.monotonic()
                ser.reset_input_buffer()
                ser.write(b"RC,status\r\n")
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    state["usb_rx_any"] = True
                    parsed = parse_rc_line(line)
                    if parsed:
                        frame_lost, failsafe, armed, channels = parsed
                        state.update(frame_lost=frame_lost, failsafe=failsafe,
                                      armed=armed, channels=channels,
                                      last_rx=time.monotonic())
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
        description="BPRL_Balance radio test — live SBUS channel monitor")
    add_port_args(parser)
    parser.add_argument("--rate", type=int, default=20,
                         help="Poll rate in Hz (default: 20)")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("rc-status", help="Live SBUS channel monitor")

    args = parser.parse_args()
    if args.command is None:
        args.command = "rc-status"

    ser = open_port(args.port, args.baud)
    try:
        cmd_rc_status(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
