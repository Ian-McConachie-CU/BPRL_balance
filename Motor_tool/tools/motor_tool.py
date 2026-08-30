#!/usr/bin/env python3
"""
Motor_tool ground station — interactive CAN motor test/setup client.

Standalone: only needs pyserial + rich (pip install pyserial rich). Talks to
the Motor_tool firmware (see ../README.md) over USB CDC with the same
line-based text protocol BPRL_balance's other ground tools use.

Usage:
    python3 tools/motor_tool.py                 # auto-detect port, start REPL
    python3 tools/motor_tool.py --port /dev/ttyACM0

Once connected, type `help` for the interactive command list, or see
../README.md for the full USB command reference this wraps.
"""

import argparse
import glob
import re
import sys
import time
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial rich")
    raise

try:
    from rich.console import Console
    from rich.table import Table
except ImportError:
    print("ERROR: rich not installed. Run: pip install pyserial rich")
    raise

console = Console()

MOTOR_TOOL_VID = 0x0483
MOTOR_TOOL_PID = 0x5740


# ── Connection ──────────────────────────────────────────────────────────────

def find_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        if p.vid == MOTOR_TOOL_VID and p.pid == MOTOR_TOOL_PID:
            return p.device
    candidates = glob.glob("/dev/ttyACM*")
    return candidates[0] if candidates else None


def open_port(port: Optional[str], baud: int) -> serial.Serial:
    if port is None:
        port = find_port()
    if port is None:
        console.print("[red]ERROR: No Motor_tool device found. Use --port to specify.")
        raise SystemExit(1)
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        console.print(f"[green]Connected: {port}")
        time.sleep(0.15)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        console.print(f"[red]ERROR: Cannot open {port}: {e}")
        raise SystemExit(1)


def send(ser: serial.Serial, line: str):
    ser.write((line + "\n").encode())
    ser.flush()


def read_lines_until(ser: serial.Serial, end_marker: Optional[str], timeout: float = 2.0,
                      quiet: float = 0.25):
    """Read reply lines from the firmware.

    With an end_marker, reads until that exact line arrives (or timeout).
    Without one, some commands (e.g. CAN,status) reply with more than one
    line and there's no marker to stop on — so instead we keep draining
    until `quiet` seconds pass with nothing new, up to the overall timeout.
    Stopping after just the first line risks displaying a stale reply left
    over from a previous command and shifting every later command's output
    by one, which is exactly the kind of confusing result this is meant to
    avoid.
    """
    deadline = time.monotonic() + timeout
    lines = []
    last_line_at = None
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            if end_marker is None and last_line_at is not None \
                    and time.monotonic() - last_line_at > quiet:
                break
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        lines.append(line)
        last_line_at = time.monotonic()
        if end_marker is not None and line == end_marker:
            break
    return lines


# ── Commands ────────────────────────────────────────────────────────────────

# Defaults match this robot's current layout: 4 RMD hip motors (IDs 1-4),
# 2 GIM wheel motors (IDs 10-11), all on bus 1.
DEFAULT_RMD_IDS = [1, 2, 3, 4]
DEFAULT_GIM_IDS = [10, 11]


def cmd_poll(ser, rmd_ids, gim_ids):
    """Actively ping every RMD id (real 0x9A status request) and every GIM
    id (Get Fault + a couple of runtime indicators — both real, documented,
    non-motion status reads), then show the aggregate table."""
    console.print(f"[dim]Polling RMD IDs {rmd_ids}...[/dim]")
    for i in rmd_ids:
        send(ser, f"RMD,{i},STATUS")
        read_lines_until(ser, None, timeout=0.3)

    console.print(f"[dim]Polling GIM IDs {gim_ids} (fault + indicators)...[/dim]")
    for i in gim_ids:
        send(ser, f"GIM,{i},FAULT")
        read_lines_until(ser, None, timeout=0.3)
        send(ser, f"GIM,{i},IND,0")   # bus voltage
        read_lines_until(ser, None, timeout=0.3)
        send(ser, f"GIM,{i},IND,2")   # motor temperature
        read_lines_until(ser, None, timeout=0.3)

    time.sleep(0.05)
    cmd_status(ser)


def cmd_status(ser):
    send(ser, "STATUS")
    lines = read_lines_until(ser, "STATUS,END")
    if not lines:
        console.print("[red]No response — is the firmware running?[/red]")
        return

    rmd_tbl = Table(title="RMD (MG8016E-i6)", show_header=True, header_style="bold")
    for col in ["ID", "pos (rad)", "vel (rad/s)", "torque (Nm)", "temp (C)", "volt (V)", "err", "age (ms)"]:
        rmd_tbl.add_column(col, justify="right")
    gim_tbl = Table(title="GIM (GIM6010-6)", show_header=True, header_style="bold")
    for col in ["ID", "pos", "vel", "torque", "temp", "fault", "last indicator", "enabled", "age (ms)"]:
        gim_tbl.add_column(col, justify="right")

    any_rmd = any_gim = any_imu = False
    for line in lines:
        m = re.match(r"STATUS,RMD,(\d+),pos=([-\d.]+),vel=([-\d.]+),tq=([-\d.]+),"
                      r"temp=([-\d.]+),volt=([-\d.]+),err=0x([0-9a-fA-F]+),age_ms=(\d+)", line)
        if m:
            any_rmd = True
            rmd_tbl.add_row(m.group(1), m.group(2), m.group(3), m.group(4),
                             m.group(5), m.group(6), "0x" + m.group(7), m.group(8))
            continue
        m = re.match(r"STATUS,GIM,(\d+),pos=([-\d.]+),vel=([-\d.]+),tq=([-\d.]+),"
                      r"temp=([-\d.]+),fault=0x([0-9a-fA-F]+),ind(\d+)=([-\d.]+),"
                      r"enabled=(\d),valid=(\d),age_ms=(\d+)", line)
        if m:
            any_gim = True
            gim_tbl.add_row(m.group(1), m.group(2), m.group(3), m.group(4), m.group(5),
                             "0x" + m.group(6), f"#{m.group(7)}={m.group(8)}",
                             "yes" if m.group(9) == "1" else "no", m.group(10))
            continue
        if line.startswith("IMU,STATUS,"):
            # This was previously dropped silently — STATUS never showed IMU
            # data even when it was working; 'imu'/'imu <seconds>' is the
            # only thing that ever displayed it. Shown here now too.
            any_imu = True
            console.print("[bold]IMX5 (bus 2)[/bold]")
            print_imu_line(line)

    if any_rmd:
        console.print(rmd_tbl)
    if any_gim:
        console.print(gim_tbl)
    if not any_rmd and not any_gim and not any_imu:
        console.print("[dim]No motors have reported feedback yet.[/dim]")


def cmd_scan(ser, duration: float):
    send(ser, "CAN,scan,start")
    started = read_lines_until(ser, None, timeout=2.0)
    if not any("started" in l for l in started):
        console.print("[red]No response to scan start.[/red]")
        return
    console.print(f"[dim]Scanning for {duration:.1f}s...[/dim]")
    time.sleep(duration)
    send(ser, "CAN,scan,stop")
    lines = read_lines_until(ser, "CAN,SCAN,END", timeout=3.0)

    entries = []
    for line in lines:
        m = re.match(r"CAN,SCAN,id=(EXT:)?0x([0-9a-fA-F]+),count=(\d+)", line)
        if m:
            entries.append((m.group(1) is not None, m.group(2), int(m.group(3))))
    if not entries:
        console.print("[yellow]No frames seen — check wiring, termination, and that the drive's Bus Type is set to CAN.[/yellow]")
        return
    entries.sort(key=lambda e: e[2], reverse=True)
    tbl = Table(title=f"CAN IDs seen in {duration:.1f}s", show_header=True, header_style="bold")
    tbl.add_column("ID"); tbl.add_column("Type"); tbl.add_column("Frames", justify="right")
    tbl.add_column("Hz", justify="right")
    for is_ext, id_str, count in entries:
        tbl.add_row(("EXT:" if is_ext else "") + "0x" + id_str,
                     "EXT" if is_ext else "STD", str(count), f"{count/duration:.0f}")
    console.print(tbl)


def cmd_monitor(ser, duration: float):
    send(ser, "CAN,monitor,start")
    console.print(f"[dim]Live monitor for {duration:.1f}s (Ctrl-C to stop early)...[/dim]")
    deadline = time.monotonic() + duration
    try:
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("MON,"):
                parts = line[4:].split(",")
                if len(parts) >= 5:
                    t_ms, bus, cid, is_ext, dlc = parts[:5]
                    data = parts[5:5 + int(dlc)]
                    console.print(f"[cyan]bus{bus}[/cyan]  {cid:>7s}  "
                                  f"{'EXT' if is_ext == '1' else 'STD'}  "
                                  f"[{dlc}]  " + " ".join(data))
    except KeyboardInterrupt:
        pass
    send(ser, "CAN,monitor,stop")
    read_lines_until(ser, None, timeout=0.5)


def cmd_imu(ser, watch_seconds: Optional[float]):
    """Single IMU,status read, or repeated ('watch') until Ctrl-C/timeout."""
    if watch_seconds is None:
        send(ser, "IMU,status")
        for l in read_lines_until(ser, None, timeout=1.0):
            print_imu_line(l)
        return

    console.print(f"[dim]Watching IMU for {watch_seconds:.1f}s (Ctrl-C to stop early)...[/dim]")
    deadline = time.monotonic() + watch_seconds
    try:
        while time.monotonic() < deadline:
            send(ser, "IMU,status")
            for l in read_lines_until(ser, None, timeout=0.5):
                print_imu_line(l)
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass


def print_imu_line(line: str):
    if line == "IMU,STATUS,valid=0":
        console.print("[dim]IMU: no valid frame yet (nothing decoded on bus 2)[/dim]")
        return
    m = re.match(
        r"IMU,STATUS,valid=1,rpy_deg=([-\d.]+),([-\d.]+),([-\d.]+),"
        r"pqr=([-\d.]+),([-\d.]+),([-\d.]+),"
        r"accel=([-\d.]+),([-\d.]+),([-\d.]+),"
        r"quat_age_ms=(\d+),rate_age_ms=(\d+)", line)
    if not m:
        console.print(line)
        return
    roll, pitch, yaw, p, q, r, ax, ay, az, qage, rage = m.groups()
    console.print(
        f"  rpy=[{roll:>7s},{pitch:>7s},{yaw:>7s}] deg   "
        f"pqr=[{p:>7s},{q:>7s},{r:>7s}] rad/s   "
        f"accel=[{ax:>6s},{ay:>6s},{az:>6s}] m/s^2   "
        f"age(quat/rate)={qage}/{rage} ms")


def cmd_send_raw(ser, args):
    if len(args) < 9:
        console.print("[red]Usage: send <id_hex> <ext0|1> <b0> ... <b7>[/red]")
        return
    can_id, ext = args[0], args[1]
    data_bytes = args[2:10]
    send(ser, f"CAN,send,{can_id},{ext}," + ",".join(data_bytes))
    for line in read_lines_until(ser, None, timeout=1.0):
        console.print(line)


def print_help():
    console.print("""[bold]Motor_tool interactive commands[/bold]
  status                                  print all known motor state
  poll [rmd_id ...]                       actively ping RMD ids (default 1-4) + scan for GIM ids, then show status
  scan [seconds]                          ID scanner on the active bus
  monitor [seconds]                       live raw frame dump (Ctrl-C to stop)
  bus <1|2>                               select active CAN bus
  send <id_hex> <ext0|1> <b0>..<b7>       inject a raw 8-byte CAN frame
  selftest                                internal-loopback test of the active bus (no wiring needed)
  imu [seconds]                           IMX5 (bus 2) reading, once or watched live

  rmd <id> torque <Nm>                    MG8016E-i6 torque command
  rmd <id> torqueraw <ratio>              MG8016E-i6 torque, raw -2048..2048 (confirmed, -33..33A for MG series)
  rmd scale [ratio_per_Nm]                get/set the Nm->ratio scale for 'rmd torque'
  rmd <id> vel <rad/s>                    MG8016E-i6 velocity command
  rmd <id> pos <rad> <maxspeed_rad/s>     MG8016E-i6 position command
  rmd <id> stop | off | resume            MG8016E-i6 zero-out / disable / re-enable
  rmd <id> status                         MG8016E-i6 request status1 read
  rmd <id> clearerr                       MG8016E-i6 clear latched error

  gim <id> start | stop | pause            GIM6010-6 enter/exit running state / pause current command
  gim <id> torque <Nm> [duration_ms]       GIM6010-6 torque command (real Nm, confirmed)
  gim <id> velocity <rad/s> [duration_ms]  GIM6010-6 velocity command
  gim <id> position <rad> [duration_ms]    GIM6010-6 position command
  gim <id> fault | ackfault                GIM6010-6 request/clear fault status
  gim <id> ind <ind_id>                    GIM6010-6 request one runtime indicator (0=bus V, 2=motor temp, 14=speed RPM, ...)
  gim limit [Nm]                           get/set the GIM torque clamp
  gim kt [Nm_per_A] / gim gear [ratio]     set the constants used to decode GIM torque feedback

  stop                                    STOP,ALL — zero/disable everything now
  raw <line>                              send a raw firmware command line verbatim
  help                                    this text
  quit                                    exit
""")


def repl(ser):
    console.print("[bold]Motor_tool[/bold] — type [cyan]help[/cyan] for commands, [cyan]quit[/cyan] to exit.")
    while True:
        try:
            line = input("motor_tool> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "q", "exit"):
            break
        elif cmd == "help":
            print_help()
        elif cmd == "status":
            cmd_status(ser)
        elif cmd == "poll":
            rmd_ids = [int(p) for p in parts[1:]] if len(parts) > 1 else DEFAULT_RMD_IDS
            cmd_poll(ser, rmd_ids, DEFAULT_GIM_IDS)
        elif cmd == "scan":
            cmd_scan(ser, float(parts[1]) if len(parts) > 1 else 1.0)
        elif cmd == "monitor":
            cmd_monitor(ser, float(parts[1]) if len(parts) > 1 else 5.0)
        elif cmd == "imu":
            cmd_imu(ser, float(parts[1]) if len(parts) > 1 else None)
        elif cmd == "selftest":
            send(ser, "CAN,selftest")
            for l in read_lines_until(ser, None, timeout=2.0):
                style = "green" if "PASS" in l else ("red" if "FAIL" in l else "")
                console.print(f"[{style}]{l}[/{style}]" if style else l)
        elif cmd == "bus" and len(parts) == 2:
            send(ser, f"BUS,{parts[1]}")
            for l in read_lines_until(ser, None, timeout=1.0):
                console.print(l)
        elif cmd == "send":
            cmd_send_raw(ser, parts[1:])
        elif cmd == "stop":
            send(ser, "STOP,ALL")
            for l in read_lines_until(ser, None, timeout=1.0):
                console.print(l)
        elif cmd == "raw":
            send(ser, line[4:])
            for l in read_lines_until(ser, None, timeout=1.0):
                console.print(l)
        elif cmd == "rmd" and len(parts) >= 2:
            handle_rmd(ser, parts[1:])
        elif cmd == "gim" and len(parts) >= 2:
            handle_gim(ser, parts[1:])
        else:
            console.print("[dim]Unknown command — type 'help'.[/dim]")


def handle_rmd(ser, args):
    if args[0] == "scale":
        send(ser, f"RMD,SCALE,{args[1]}" if len(args) > 1 else "RMD,SCALE,")
        for l in read_lines_until(ser, None, timeout=1.0):
            console.print(l)
        return
    if len(args) < 2:
        console.print("[red]Usage: rmd <id> <torque|torqueraw|vel|pos|stop|off|resume|status|clearerr> ...[/red]")
        return
    mid, sub = args[0], args[1].lower()
    cmd_map = {
        "torque":    lambda: f"RMD,{mid},TORQUE,{args[2]}",
        "torqueraw": lambda: f"RMD,{mid},TORQUERAW,{args[2]}",
        "vel":       lambda: f"RMD,{mid},VELOCITY,{args[2]}",
        "pos":       lambda: f"RMD,{mid},POSITION,{args[2]},{args[3]}",
        "stop":      lambda: f"RMD,{mid},STOP",
        "off":       lambda: f"RMD,{mid},OFF",
        "resume":    lambda: f"RMD,{mid},RESUME",
        "status":    lambda: f"RMD,{mid},STATUS",
        "clearerr":  lambda: f"RMD,{mid},CLEARERR",
    }
    if sub not in cmd_map:
        console.print(f"[red]Unknown rmd subcommand: {sub}[/red]")
        return
    try:
        send(ser, cmd_map[sub]())
    except IndexError:
        console.print("[red]Missing argument(s).[/red]")
        return
    for l in read_lines_until(ser, None, timeout=1.0):
        console.print(l)


def handle_gim(ser, args):
    if args[0] == "limit":
        send(ser, f"GIM,LIMIT,{args[1]}" if len(args) > 1 else "GIM,LIMIT,")
        for l in read_lines_until(ser, None, timeout=1.0):
            console.print(l)
        return
    if args[0] in ("kt", "gear"):
        cmd_word = "KT" if args[0] == "kt" else "GEAR"
        send(ser, f"GIM,{cmd_word},{args[1]}" if len(args) > 1 else f"GIM,{cmd_word},")
        for l in read_lines_until(ser, None, timeout=1.0):
            console.print(l)
        return
    if len(args) < 2:
        console.print("[red]Usage: gim <id> <start|stop|pause|torque|velocity|position|fault|ackfault|ind> ...[/red]")
        return
    mid, sub = args[0], args[1].lower()
    try:
        if sub == "start":
            send(ser, f"GIM,{mid},START")
        elif sub == "stop":
            send(ser, f"GIM,{mid},STOP")
        elif sub == "pause":
            send(ser, f"GIM,{mid},PAUSE")
        elif sub == "torque":
            dur = f",{args[3]}" if len(args) > 3 else ""
            send(ser, f"GIM,{mid},TORQUE,{args[2]}{dur}")
        elif sub == "velocity":
            dur = f",{args[3]}" if len(args) > 3 else ""
            send(ser, f"GIM,{mid},VELOCITY,{args[2]}{dur}")
        elif sub == "position":
            dur = f",{args[3]}" if len(args) > 3 else ""
            send(ser, f"GIM,{mid},POSITION,{args[2]}{dur}")
        elif sub == "fault":
            send(ser, f"GIM,{mid},FAULT")
        elif sub == "ackfault":
            send(ser, f"GIM,{mid},ACKFAULT")
        elif sub == "ind":
            send(ser, f"GIM,{mid},IND,{args[2]}")
        else:
            console.print(f"[red]Unknown gim subcommand: {sub}[/red]")
            return
    except IndexError:
        console.print("[red]Missing argument(s).[/red]")
        return
    for l in read_lines_until(ser, None, timeout=1.0):
        console.print(l)


# ── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Motor_tool interactive ground station")
    parser.add_argument("--port", default=None, help="Serial port (auto-detected)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = open_port(args.port, args.baud)
    try:
        repl(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
