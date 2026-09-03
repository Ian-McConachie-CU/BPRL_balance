#!/usr/bin/env python3
"""
BPRL_Balance CAN Motor Test — live status of all 6 CAN motors, and a bench
sweep-test for exercising one motor at a time.

Everything here talks to the SAME firmware command path the real controller
uses (MOTOR,status / MOTOR,test,*) — there is no separate/bypassing debug
protocol, so the hip safety gate in src/coms/CANMotor.cpp (zero-offset,
angle/velocity soft limits, hard torque clamp) applies here exactly as it
does to normal balance/car control. See CANMotor.hpp's header comment.

Usage:
    python3 tools/motor_test.py                    # default: live status, all 6 motors
    python3 tools/motor_test.py motor-status
    python3 tools/motor_test.py wheel-status        # ONLY the 2 wheels, actively requested (RTR)
    python3 tools/motor_test.py motor-sweep <id>    # id: 1-6

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
    --rate N              Poll rate in Hz (default 20)

Motor layout (see main.cpp's can_motor_register calls):
    ID 1  Hip FL   — LKMTECH MG8016E-i6  (RMD protocol)
    ID 2  Hip FR   — LKMTECH MG8016E-i6
    ID 3  Hip RL   — LKMTECH MG8016E-i6
    ID 4  Hip RR   — LKMTECH MG8016E-i6
    ID 5  Wheel L  — Steadywin GIM6010-8 on GDS68 (ODrive CAN Simple)
    ID 6  Wheel R  — Steadywin GIM6010-8 on GDS68 (ODrive CAN Simple)

motor-sweep behavior (see src/controllers/MotorTest.{hpp,cpp}):
    Hip (1-4):   slow position sweep, positive until it hits its safety
                 bound, then negative until it hits the other bound, repeat.
    Wheel (5-6): slow spin for one full output-shaft rotation, then
                 reverses, repeat.
    Only runs while DISARMED — arming the transmitter stops it immediately
    (see ControlThread in src/threads.cpp). Ctrl-C here also stops it
    (MOTOR,test,stop is always sent on exit, including on error).
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args, send_cmd

from rich.live import Live
from rich.table import Table
from rich.text import Text
from rich.panel import Panel

MOTOR_LABELS = {
    1: "Hip FL  (RMD)",
    2: "Hip FR  (RMD)",
    3: "Hip RL  (RMD)",
    4: "Hip RR  (RMD)",
    5: "Wheel L (ODrive)",
    6: "Wheel R (ODrive)",
}

MOTOR_RE = re.compile(
    r"MOTOR,(\d+),pos=([+-]?\d+\.?\d*),vel=([+-]?\d+\.?\d*),"
    r"tq=([+-]?\d+\.?\d*),temp=([+-]?\d+\.?\d*),v=(\d),"
    r"tx_ok=(\d+),tx_fail=(\d+),rx=(\d+)")


# How often does this motor's telemetry actually arrive? rx (from
# MOTOR,status) is a cumulative reply COUNT, not a rate -- this tracks a
# rolling ~1s window per motor id and turns the count into Hz, answering
# "what's the practical bandwidth of the motor feedback" directly rather
# than needing to eyeball rx deltas across snapshots by hand. Deliberately
# NOT recomputed every poll (at this tool's 10-20 Hz default poll rate, a
# single-poll delta is too small/jittery to read meaningfully -- a couple
# of counts either way swings the instantaneous number a lot); holding for
# a full second before each update gives a stable, easy-to-read rate.
HZ_WINDOW_S = 1.0


def update_rx_hz(state: dict, mid: int, rx_count: int, now: float):
    win = state.setdefault("hz_window", {})
    w = win.get(mid)
    if w is None:
        win[mid] = {"start_rx": rx_count, "start_t": now}
        return
    elapsed = now - w["start_t"]
    if elapsed >= HZ_WINDOW_S:
        hz = (rx_count - w["start_rx"]) / elapsed
        state["motors"][mid]["hz"] = hz
        win[mid] = {"start_rx": rx_count, "start_t": now}
    # else: leave whatever Hz was last computed in place -- window still filling.


def poll_motor_status(ser, state: dict):
    """Send MOTOR,status and fold any replies into state['motors'][id]."""
    ser.reset_input_buffer()
    send_cmd(ser, "MOTOR,status")
    deadline = time.monotonic() + 0.3
    # Count FRESH matches this call, not len(state["motors"]) -- that dict
    # persists across calls (on purpose, so a motor that misses one poll
    # keeps showing its last value), so after the first successful poll it
    # already has all 6 keys before this call even starts. Breaking on its
    # size meant every call after the first returned the instant motor 1's
    # line arrived (firmware replies in ID order 1->6, so it's always
    # first) -- motor 1 got fresh data every time, motors 2-6 (wheels
    # included) were never touched again after the very first poll. Same
    # bug explains both "only motor 1 updates live" and "wheels never give
    # live data".
    seen_this_poll = 0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        state["usb_rx_any"] = True
        m = MOTOR_RE.match(line)
        if m:
            mid = int(m.group(1))
            now = time.monotonic()
            rx_count = int(m.group(9))
            state["motors"][mid] = {
                "pos": float(m.group(2)), "vel": float(m.group(3)),
                "tq": float(m.group(4)), "temp": float(m.group(5)),
                "valid": bool(int(m.group(6))),
                "tx_ok": int(m.group(7)), "tx_fail": int(m.group(8)), "rx": rx_count,
                "hz": state["motors"].get(mid, {}).get("hz"),
            }
            update_rx_hz(state, mid, rx_count, now)
            state["last_rx"] = now
            seen_this_poll += 1
        if seen_this_poll >= 6:
            break


def build_motor_table(state: dict, highlight_id: int = None, ids=range(1, 7)) -> Table:
    tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 1))
    tbl.add_column("ID", justify="right", min_width=2)
    tbl.add_column("Motor", min_width=17)
    tbl.add_column("pos (rad)", justify="right", min_width=10)
    tbl.add_column("vel (rad/s)", justify="right", min_width=11)
    tbl.add_column("tq (Nm)", justify="right", min_width=8)
    tbl.add_column("temp (C)", justify="right", min_width=8)
    tbl.add_column("tx ok/fail", justify="right", min_width=10)
    tbl.add_column("rx", justify="right", min_width=7)
    tbl.add_column("Hz", justify="right", min_width=7)
    tbl.add_column("", min_width=10)

    for mid in ids:
        m = state["motors"].get(mid)
        label = MOTOR_LABELS.get(mid, f"Motor {mid}")
        row_style = "bold yellow" if mid == highlight_id else ("" if (m and m["valid"]) else "dim")
        mark = " <-- under test" if mid == highlight_id else ""
        if m is None:
            tbl.add_row(str(mid), Text(label, style=row_style), "-", "-", "-", "-", "-", "-", "-",
                        Text("no data" + mark, style=row_style))
            continue
        status = ("valid" + mark) if m["valid"] else ("no data" + mark)
        tx_style = "red" if m["tx_fail"] > 0 else row_style
        hz_str = f"{m['hz']:.0f}" if m.get("hz") is not None else "..."
        tbl.add_row(str(mid), Text(label, style=row_style),
                    f"{m['pos']:+.3f}", f"{m['vel']:+.3f}", f"{m['tq']:+.2f}",
                    f"{m['temp']:.1f}",
                    Text(f"{m['tx_ok']}/{m['tx_fail']}", style=tx_style),
                    str(m["rx"]),
                    hz_str,
                    Text(status, style=row_style))
    return tbl


def build_status_panel(state: dict, title: str, footer: str = "", ids=range(1, 7)) -> Panel:
    age = time.monotonic() - state["last_rx"]
    stale = age > 1.0 or state["last_rx"] == 0.0

    if not state["usb_rx_any"]:
        note = "  [dim](no USB response yet — check connection / firmware build)[/dim]"
    elif stale:
        note = "  [yellow](stale — no recent MOTOR,status reply)[/yellow]"
    else:
        note = ""

    body = Table.grid()
    body.add_row(build_motor_table(state, state.get("highlight_id"), ids=ids))
    if footer:
        body.add_row("")
        body.add_row(Text(footer, style="dim"))
    return Panel(body, title=f"{title}{note}", border_style="blue")


# ── Tool 1: live status of all 6 motors ─────────────────────────────────────

def cmd_motor_status(ser, args):
    rate = max(1, getattr(args, "rate", 20))
    period = 1.0 / rate
    state = {"motors": {}, "usb_rx_any": False, "last_rx": 0.0, "highlight_id": None}

    try:
        with Live(build_status_panel(state, "BPRL Motor Status — all 6"),
                  refresh_per_second=rate, console=console) as live:
            while True:
                t0 = time.monotonic()
                poll_motor_status(ser, state)
                live.update(build_status_panel(state, "BPRL Motor Status — all 6"))
                elapsed = time.monotonic() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass


# ── Tool 1b: wheel-only, ACTIVELY requested (not just passive broadcast) ────

WHEEL_IDS = (5, 6)


def poll_wheel_status(ser, state: dict):
    """Actively request a fresh read from each wheel (MOTOR,<id>,request --
    RTR for Get_Encoder_Estimates, see CANMotor.cpp's can_motor_request_encoder())
    before reading MOTOR,status back, instead of just waiting on whatever
    the ODrive's own passive broadcast happens to deliver next. Safe at
    Python polling rates (10-20 Hz) -- this is the same call ControlThread
    used to make every 2.5 ms tick before that was found to overload the
    bus (see telemetry_plan.md item C); at this rate it's a few requests a
    second, not a few hundred."""
    ser.reset_input_buffer()
    for wid in WHEEL_IDS:
        send_cmd(ser, f"MOTOR,{wid},request")
    # Drain the REQUEST,OK/ERR acks so they don't get mistaken for MOTOR,...
    # status lines below; also gives the RTR reply a brief moment to land.
    deadline = time.monotonic() + 0.05
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            state["usb_rx_any"] = True

    ser.reset_input_buffer()
    send_cmd(ser, "MOTOR,status")
    deadline = time.monotonic() + 0.3
    seen_this_poll = 0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        state["usb_rx_any"] = True
        m = MOTOR_RE.match(line)
        if m and int(m.group(1)) in WHEEL_IDS:
            mid = int(m.group(1))
            now = time.monotonic()
            rx_count = int(m.group(9))
            state["motors"][mid] = {
                "pos": float(m.group(2)), "vel": float(m.group(3)),
                "tq": float(m.group(4)), "temp": float(m.group(5)),
                "valid": bool(int(m.group(6))),
                "tx_ok": int(m.group(7)), "tx_fail": int(m.group(8)), "rx": rx_count,
                "hz": state["motors"].get(mid, {}).get("hz"),
            }
            update_rx_hz(state, mid, rx_count, now)
            state["last_rx"] = now
            seen_this_poll += 1
        if seen_this_poll >= len(WHEEL_IDS):
            break


def cmd_wheel_status(ser, args):
    rate = max(1, getattr(args, "rate", 20))
    period = 1.0 / rate
    state = {"motors": {}, "usb_rx_any": False, "last_rx": 0.0, "highlight_id": None}
    title = "BPRL Wheel Status — actively requested (RTR), not passive"
    footer = ("Hz here is TOPPED UP by this tool's own RTR requests (~%.0f Hz, --rate) on top of "
               "whatever the ODrive broadcasts on its own -- use 'motor-status' instead for the "
               "wheel's natural/passive rate, unaffected by this tool's polling." % rate)

    try:
        with Live(build_status_panel(state, title, footer=footer, ids=WHEEL_IDS),
                  refresh_per_second=rate, console=console) as live:
            while True:
                t0 = time.monotonic()
                poll_wheel_status(ser, state)
                live.update(build_status_panel(state, title, footer=footer, ids=WHEEL_IDS))
                elapsed = time.monotonic() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass


# ── Tool 2: bench sweep test for one motor ──────────────────────────────────

def cmd_motor_sweep(ser, args):
    motor_id = args.motor_id
    if not (1 <= motor_id <= 6):
        console.print(f"[red]motor_id must be 1-6, got {motor_id}[/red]")
        return

    ser.reset_input_buffer()
    send_cmd(ser, f"MOTOR,test,{motor_id},start")
    deadline = time.monotonic() + 1.0
    started = False
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line.startswith("MOTOR,TEST,STARTED"):
            started = True
        if line.startswith("MOTOR,TEST,ERR"):
            console.print(f"[red]Firmware refused: {line}[/red]")
            return
        if started and "id=" in line:
            break

    if not started:
        console.print("[red]No MOTOR,TEST,STARTED response — is firmware running?[/red]")
        return

    label = MOTOR_LABELS.get(motor_id, f"Motor {motor_id}")
    console.print(f"[green]Sweep test started: ID {motor_id} {label}[/green]")
    console.print("[dim]Disarmed-only — arming the transmitter stops this automatically. "
                  "Ctrl-C also stops it.[/dim]\n")

    rate = max(1, getattr(args, "rate", 20))
    period = 1.0 / rate
    state = {"motors": {}, "usb_rx_any": False, "last_rx": 0.0, "highlight_id": motor_id}
    title = f"BPRL Motor Sweep — ID {motor_id} {label}"
    footer = ("Hip: slow sweep between its safety bounds (CANMotor.cpp's HIP_ANGLE_MIN/MAX_RAD)."
              if motor_id <= 4 else
              "Wheel: slow spin, one full output-shaft rotation, then reverses.")

    try:
        with Live(build_status_panel(state, title, footer),
                  refresh_per_second=rate, console=console) as live:
            while True:
                t0 = time.monotonic()
                poll_motor_status(ser, state)
                live.update(build_status_panel(state, title, footer))
                elapsed = time.monotonic() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        # Always stop the sweep on the way out, including on error / Ctrl-C —
        # never leave a motor being driven by a bench test with nobody watching.
        send_cmd(ser, "MOTOR,test,stop")
        time.sleep(0.1)
        ser.reset_input_buffer()
        console.print("\n[green]Sweep test stopped.[/green]")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL_Balance CAN motor test — live status and bench sweep test")
    add_port_args(parser)
    parser.add_argument("--rate", type=int, default=20,
                         help="Poll rate in Hz (default: 20)")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("motor-status", help="Live status (angle + speed) of all 6 motors")
    sub.add_parser("wheel-status", help="Live status of ONLY the 2 wheel motors, actively requested (RTR)")
    p_sweep = sub.add_parser("motor-sweep", help="Bench sweep test for one motor")
    p_sweep.add_argument("motor_id", type=int, help="Motor id to test, 1-6")

    args = parser.parse_args()
    if args.command is None:
        args.command = "motor-status"

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "motor-status":
            cmd_motor_status(ser, args)
        elif args.command == "wheel-status":
            cmd_wheel_status(ser, args)
        elif args.command == "motor-sweep":
            cmd_motor_sweep(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
