#!/usr/bin/env python3
"""
BPRL_Balance Telemetry — live sensor dashboard, EKF lane status, IMU comparison.

Requires a firmware built with -DBPRL_DEBUG.

Usage:
    python3 tools/telemetry.py                        # default: live attitude dashboard
    python3 tools/telemetry.py telemetry              # attitude / rates / IMU dashboard
    python3 tools/telemetry.py ekf-status             # per-lane EKF roll/pitch/yaw table
    python3 tools/telemetry.py imu-compare            # side-by-side IMU accel+gyro with deltas

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

from bprl_common import (
    console, open_port, add_port_args, SerialReader,
    TelState, parse_tel_line,
    ImuSample, parse_imu_line,
    EkfLaneState, parse_ekfl_line,
)

from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# ── Telemetry display ─────────────────────────────────────────────────────────

IMU_LABELS = ["IMU0 (pri)   ", "IMU1 (ext)   ", "IMU2         "]


def _valid(v: bool) -> Text:
    return Text("● valid  ", style="green") if v else Text("○ absent ", style="dim")


def build_telemetry_panel(s: TelState) -> Panel:
    age   = time.monotonic() - s.last_rx
    stale = age > 1.0

    arm_text = Text("● ARMED  ", style="bold red") if s.armed \
               else Text("● DISARMED", style="bold green")
    t_sec = s.time_ms / 1000.0

    grid = Table.grid(padding=(0, 2))
    grid.add_column(min_width=18)
    grid.add_column(min_width=18)
    grid.add_column(min_width=18)
    grid.add_row("[bold]ATTITUDE[/bold]",
                 "[bold]BODY RATES (rad/s)[/bold]",
                 "[bold]RC INPUTS[/bold]")
    grid.add_row(f"Roll:  {s.roll:+7.2f}°",
                 f"P: {s.p:+8.4f}",
                 f"Thr:   {s.thr:5.3f}")
    grid.add_row(f"Pitch: {s.pitch:+7.2f}°",
                 f"Q: {s.q:+8.4f}",
                 f"Roll:  {s.rc_roll:+5.3f}")
    grid.add_row(f"Yaw:   {s.yaw:+7.2f}°",
                 f"R: {s.r:+8.4f}",
                 f"Pitch: {s.rc_pitch:+5.3f}")
    grid.add_row("", "", f"Yaw:   {s.rc_yaw:+5.3f}")

    imu_grid = Table.grid(padding=(0, 2))
    imu_grid.add_column(min_width=20)
    imu_grid.add_column()
    for i, lbl in enumerate(IMU_LABELS):
        imu_grid.add_row(f"  IMU{i} {lbl}:", _valid(s.imu_valid[i]))
    can_line = Text()
    can_line.append(_valid(s.can_valid))
    if s.can_valid:
        can_line.append(f"quat={s.can_quat_hz:3d} Hz  rate={s.can_rate_hz:3d} Hz",
                        style="cyan")
    imu_grid.add_row("  CAN INS (IMX5):    ", can_line)

    if not s.usb_rx_any:
        stale_note = "  [dim](no USB data — check connection)[/dim]"
    elif not s.received_any:
        stale_note = "  [dim](USB ok, waiting for $TEL — requires -DBPRL_DEBUG build)[/dim]"
    elif stale:
        stale_note = "  [yellow](stale — lost contact)[/yellow]"
    else:
        stale_note = ""
    diag  = f"  [dim]lines_rx={s.lines_rx}[/dim]"
    title = (f"BPRL_Balance Debug  {arm_text}    t={t_sec:8.1f} s{stale_note}{diag}")

    body = Table.grid()
    body.add_row(grid)
    body.add_row(Panel(imu_grid, title="IMU Status", border_style="dim"))
    return Panel(body, title=title, border_style="blue")


def cmd_telemetry(ser, _args):
    state  = TelState()
    reader = SerialReader(ser)
    try:
        with Live(build_telemetry_panel(state), refresh_per_second=10,
                  console=console) as live:
            while True:
                for line in reader.pop_lines():
                    state.lines_rx += 1
                    if not parse_tel_line(line, state):
                        state.usb_rx_any = True
                live.update(build_telemetry_panel(state))
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


# ── EKF per-lane status ───────────────────────────────────────────────────────

LANE_LABELS = ["Lane 0", "Lane 1", "Lane 2", "IMX5 INS"]


def build_ekf_panel(s: EkfLaneState) -> Panel:
    age   = time.monotonic() - s.last_rx
    stale = age > 1.0
    t_sec = s.time_ms / 1000.0

    if not s.usb_rx_any:
        stale_note = "  [dim](no USB data)[/dim]"
    elif not s.received_any:
        stale_note = "  [dim](waiting for $EKFL — requires -DBPRL_DEBUG build)[/dim]"
    elif stale:
        stale_note = "  [yellow](stale)[/yellow]"
    else:
        stale_note = ""

    tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 2))
    tbl.add_column("", min_width=12)
    for i, lbl in enumerate(LANE_LABELS):
        pri_tag = " [bold green](pri)[/bold green]" if (i < 3 and i == s.primary) else ""
        ins_tag = " [dim](zeros until CAN enabled)[/dim]" if i == 3 else ""
        tbl.add_column(f"{lbl}{pri_tag}{ins_tag}", min_width=18, justify="right")

    def _row(label, values, fmt, unit=""):
        cells = [f"[bold]{label}[/bold]"]
        for i, v in enumerate(values):
            style = "dim" if (i == 3 and v == 0.0) else ""
            cells.append(f"[{style}]{v:{fmt}}{unit}[/{style}]" if style
                         else f"{v:{fmt}}{unit}")
        tbl.add_row(*cells)

    _row("Roll",  s.roll,  "+8.2f", "°")
    _row("Pitch", s.pitch, "+8.2f", "°")
    _row("Yaw",   s.yaw,   "+8.2f", "°")
    tbl.add_row("")
    _row("p", s.p, "+9.4f")
    _row("q", s.q, "+9.4f")
    _row("r", s.r, "+9.4f")

    diag  = f"  [dim]lines_rx={s.lines_rx}[/dim]"
    title = f"BPRL_Balance EKF Lanes    t={t_sec:8.1f} s{stale_note}{diag}"
    return Panel(tbl, title=title, border_style="cyan")


def cmd_ekf_status(ser, _args):
    state  = EkfLaneState()
    reader = SerialReader(ser)
    try:
        with Live(build_ekf_panel(state), refresh_per_second=10,
                  console=console) as live:
            while True:
                for line in reader.pop_lines():
                    state.lines_rx += 1
                    if not parse_ekfl_line(line, state):
                        state.usb_rx_any = True
                live.update(build_ekf_panel(state))
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


# ── IMU comparison ────────────────────────────────────────────────────────────

@dataclass
class ImuCompareState:
    time_ms:      float = 0.0
    accel:        list  = field(default_factory=lambda: [[0.0]*3 for _ in range(3)])
    gyro:         list  = field(default_factory=lambda: [[0.0]*3 for _ in range(3)])
    valid:        list  = field(default_factory=lambda: [False]*3)
    can_pqr:      list  = field(default_factory=lambda: [0.0]*3)
    can_valid:    bool  = False
    ekf_roll:     list  = field(default_factory=lambda: [0.0]*4)
    ekf_pitch:    list  = field(default_factory=lambda: [0.0]*4)
    ekf_yaw:      list  = field(default_factory=lambda: [0.0]*4)
    ekf_primary:  int   = 0
    received_imu: bool  = False
    received_ekf: bool  = False
    received_any: bool  = False
    usb_rx_any:   bool  = False
    last_rx:      float = field(default_factory=time.monotonic)
    lines_rx:     int   = 0


def parse_imu_compare_line(line: str, state: ImuCompareState) -> bool:
    if line.startswith("$IMU,"):
        s = parse_imu_line(line)
        if s is None:
            return False
        state.time_ms = float(s.time_ms)
        state.accel   = s.accel
        state.gyro    = s.gyro
        state.valid   = s.valid
        parts = line[5:].split(",")
        if len(parts) >= 26:
            try:
                state.can_pqr   = [float(parts[22]), float(parts[23]), float(parts[24])]
                state.can_valid = bool(int(parts[25]))
            except (ValueError, IndexError):
                pass
        state.received_imu = True
        state.received_any = True
        state.usb_rx_any   = True
        state.last_rx      = time.monotonic()
        return True

    if line.startswith("$EKFL,"):
        tmp = EkfLaneState()
        if not parse_ekfl_line(line, tmp):
            return False
        state.ekf_roll    = list(tmp.roll)
        state.ekf_pitch   = list(tmp.pitch)
        state.ekf_yaw     = list(tmp.yaw)
        state.ekf_primary = tmp.primary
        state.received_ekf = True
        state.received_any = True
        state.usb_rx_any   = True
        state.last_rx      = time.monotonic()
        return True

    return False


_AXIS_LABELS    = ["X / P", "Y / Q", "Z / R"]
_IMU_COL_LABELS = ["IMU0 (pri)", "IMU1 (ext)", "IMU2      "]
_EKF_COL_LABELS = ["Lane 0", "Lane 1", "Lane 2", "CAN IMX5"]


def _delta_style(v: float, warn: float, err: float) -> str:
    if abs(v) > err:  return "red"
    if abs(v) > warn: return "yellow"
    return "green"


def build_imu_compare_panel(s: ImuCompareState) -> Panel:
    age   = time.monotonic() - s.last_rx
    stale = age > 1.0
    t_sec = s.time_ms / 1000.0

    if not s.usb_rx_any:
        stale_note = "  [dim](no USB data)[/dim]"
    elif not s.received_any:
        stale_note = "  [dim](waiting for $IMU/$EKFL — requires -DBPRL_DEBUG build)[/dim]"
    elif stale:
        stale_note = "  [yellow](stale)[/yellow]"
    else:
        stale_note = ""

    a_tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 2))
    a_tbl.add_column("Accel (m/s²)", min_width=12)
    for lbl in _IMU_COL_LABELS:
        a_tbl.add_column(lbl, min_width=13, justify="right")
    a_tbl.add_column("Δ 1−0", min_width=10, justify="right")
    a_tbl.add_column("Δ 2−0", min_width=10, justify="right")

    for k, ax in enumerate(["X", "Y", "Z"]):
        vals   = [s.accel[i][k] for i in range(3)]
        d1, d2 = vals[1] - vals[0], vals[2] - vals[0]
        a_tbl.add_row(
            f"  {ax}",
            *[f"{v:+10.4f}" for v in vals],
            f"[{_delta_style(d1,.1,.5)}]{d1:+9.4f}[/{_delta_style(d1,.1,.5)}]",
            f"[{_delta_style(d2,.1,.5)}]{d2:+9.4f}[/{_delta_style(d2,.1,.5)}]",
        )

    r_tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 2))
    r_tbl.add_column("Rate (rad/s)", min_width=12)
    for lbl in _IMU_COL_LABELS:
        r_tbl.add_column(lbl, min_width=13, justify="right")
    can_v_tag = "[green]CAN IMX5[/green]" if s.can_valid else "[dim]CAN IMX5[/dim]"
    r_tbl.add_column(can_v_tag, min_width=13, justify="right")
    r_tbl.add_column("Δ 1−0", min_width=10, justify="right")
    r_tbl.add_column("Δ 2−0", min_width=10, justify="right")

    for k, ax in enumerate(_AXIS_LABELS):
        imu_vals = [s.gyro[i][k] for i in range(3)]
        can_val  = s.can_pqr[k] if s.can_valid else float("nan")
        d1, d2   = imu_vals[1] - imu_vals[0], imu_vals[2] - imu_vals[0]
        can_str  = f"{can_val:+10.5f}" if s.can_valid else "  [dim]  ------[/dim]"
        r_tbl.add_row(
            f"  {ax}",
            *[f"{v:+10.5f}" for v in imu_vals],
            can_str,
            f"[{_delta_style(d1,.01,.05)}]{d1:+9.5f}[/{_delta_style(d1,.01,.05)}]",
            f"[{_delta_style(d2,.01,.05)}]{d2:+9.5f}[/{_delta_style(d2,.01,.05)}]",
        )

    ang_tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 2))
    ang_tbl.add_column("Angle (°)", min_width=12)
    for i, lbl in enumerate(_EKF_COL_LABELS):
        pri = " [bold green](pri)[/bold green]" if (i < 3 and i == s.ekf_primary) else ""
        can = " [green]●[/green]" if (i == 3 and s.can_valid) else \
              (" [dim]○[/dim]"   if i == 3 else "")
        ang_tbl.add_column(f"{lbl}{pri}{can}", min_width=14, justify="right")
    ang_tbl.add_column("Δ max−min", min_width=10, justify="right")

    not_received = not s.received_ekf
    for label, vals in [("  Roll",  s.ekf_roll),
                         ("  Pitch", s.ekf_pitch),
                         ("  Yaw",   s.ekf_yaw)]:
        if not_received:
            ang_tbl.add_row(label, *["[dim]---[/dim]"]*4, "[dim]---[/dim]")
            continue
        active = vals if s.can_valid else vals[:3]
        spread = max(active) - min(active)
        sp_col = _delta_style(spread, 1.0, 5.0)
        cells  = [f"{v:+9.2f}°" for v in vals[:3]]
        can_cell = f"{vals[3]:+9.2f}°" if s.can_valid else "[dim]  ------[/dim]"
        ang_tbl.add_row(label, *cells, can_cell,
                        f"[{sp_col}]{spread:+8.2f}°[/{sp_col}]")

    a_tbl.add_row("[bold]Valid[/bold]",
                  *["[green]●[/green]" if v else "[dim]○[/dim]" for v in s.valid], "", "")

    diag  = f"  [dim]lines_rx={s.lines_rx}[/dim]"
    title = f"IMU Comparison    t={t_sec:8.1f} s{stale_note}{diag}"

    body = Table.grid()
    body.add_row(Panel(a_tbl,   title="Accelerometers", border_style="dim"))
    body.add_row(Panel(r_tbl,   title="Angular Rates",  border_style="dim"))
    body.add_row(Panel(ang_tbl, title="Attitude (EKF lanes + CAN IMX5)", border_style="dim"))
    return Panel(body, title=title, border_style="magenta")


def cmd_imu_compare(ser, _args):
    state  = ImuCompareState()
    reader = SerialReader(ser)
    try:
        with Live(build_imu_compare_panel(state), refresh_per_second=10,
                  console=console) as live:
            while True:
                for line in reader.pop_lines():
                    state.lines_rx += 1
                    if not parse_imu_compare_line(line, state):
                        state.usb_rx_any = True
                live.update(build_imu_compare_panel(state))
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL_Balance telemetry tools — requires -DBPRL_DEBUG firmware build")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("telemetry",   help="Live attitude / rates / IMU dashboard")
    sub.add_parser("ekf-status",  help="Per-lane EKF roll/pitch/yaw/p/q/r table")
    sub.add_parser("imu-compare", help="Side-by-side IMU accel+gyro comparison with deltas")

    args = parser.parse_args()
    if args.command is None:
        args.command = "telemetry"

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "telemetry":
            cmd_telemetry(ser, args)
        elif args.command == "ekf-status":
            cmd_ekf_status(ser, args)
        elif args.command == "imu-compare":
            cmd_imu_compare(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
