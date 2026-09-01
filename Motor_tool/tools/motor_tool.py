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
import math
import re
import struct
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
        # Short read timeout so read_lines_until()'s quiet-period detection
        # (default 0.25s) actually resolves in close to that long, instead
        # of needing multiple 0.2s blocking reads to accumulate past it.
        ser = serial.Serial(port, baud, timeout=0.05)
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


def _non_mon(lines):
    """Drop raw MON,... frame lines from a captured line list before
    showing it in an error message. CAN,monitor captures BOTH buses
    indiscriminately, and bus 2 (the IMX5 IMU) streams continuously at a
    high rate -- while a monitor session is active for an unrelated bus-1
    diagnostic, its output gets buried in IMU noise otherwise. This is only
    for what gets shown to the user; callers that need the raw MON lines
    for actual bus-1 traffic analysis (poll_hips/poll_gim/poll_wheels) keep
    working from their own unfiltered capture."""
    return [l for l in lines if not l.startswith("MON,")]


# ── Commands ────────────────────────────────────────────────────────────────

# Defaults match this robot's current layout: 4 RMD hip motors (IDs 1-4),
# 2 GIM wheel motors, all on bus 1. Each GIM wheel's CAN ID and Host/Master
# CAN ID are configured as DIFFERENT values (confirmed 2026-08-31 via
# poll_wheels): wheel 1 = CAN ID 15 / reply SID 16 (moved from 10/11),
# wheel 2 = CAN ID 20 / reply SID 21. DEFAULT_GIM_IDS lists
# the CAN IDs (what commands are sent to) -- 16 is NOT a second motor, it's
# wheel 1's reply address.
DEFAULT_RMD_IDS = [1, 2, 3, 4]
DEFAULT_GIM_IDS = [15, 20]
DEFAULT_GIM_MASTER_IDS = {15: 16, 20: 21}   # CAN ID -> reply SID, per gim_set_reply_id()


def ensure_gim_reply_ids(ser, gim_ids, master_map=None):
    """Push gim_set_reply_id() to the firmware for every id in gim_ids that
    has a known Host/Master CAN ID mapping. Without this, the firmware
    listens for a GIM's reply on its own CAN ID, which is wrong whenever
    Master CAN ID differs (confirmed 2026-08-31 via poll_wheels) -- every
    GIM,<id>,... request would ack fine (frame sent OK) but never populate
    STATUS, since the actual reply lands on a SID nothing was watching.
    Cheap and idempotent, so it's safe to call before every poll — this
    also makes the mapping self-healing across firmware reboots/reflashes
    without a separate manual setup step."""
    master_map = master_map or DEFAULT_GIM_MASTER_IDS
    for i in gim_ids:
        master_id = master_map.get(i)
        if master_id is None:
            continue
        send(ser, f"GIM,{i},MASTERID,{master_id}")
        read_lines_until(ser, None, timeout=0.2)


def cmd_poll(ser, rmd_ids, gim_ids, retries=2):
    """Actively ping every RMD id (real 0x9A status request) and every GIM
    id (Get Fault + a couple of runtime indicators — both real, documented,
    non-motion status reads), then show the aggregate table.

    Sending a request only proves the frame reached the CAN TX mailbox (the
    firmware's 'OK' ack) — the actual telemetry lands in its cache
    asynchronously, whenever (and if) a reply frame arrives and gets
    decoded. A motor that just powered up, or a marginal connection, can
    easily miss a single fixed settle window. Rather than silently
    accepting whatever showed up on one pass (which is what made 'poll'
    look flaky — the exact same motor could appear or not depending on
    timing alone), this retries only the ids that are still missing after
    each STATUS snapshot, and reports plainly if an id never answers."""
    ensure_gim_reply_ids(ser, gim_ids)
    pending_rmd = set(rmd_ids)
    pending_gim = set(gim_ids)
    seen_rmd, seen_gim = set(), set()

    for attempt in range(1, retries + 2):
        if not pending_rmd and not pending_gim:
            break
        if attempt > 1:
            console.print(f"[dim]Retry {attempt - 1}: still missing "
                           f"RMD {sorted(pending_rmd)} GIM {sorted(pending_gim)}...[/dim]")
        else:
            console.print(f"[dim]Polling RMD IDs {sorted(pending_rmd)}...[/dim]")

        for i in sorted(pending_rmd):
            for cmd in (f"RMD,{i},STATUS", f"RMD,{i},ENCODER"):
                send(ser, cmd)
                lines = read_lines_until(ser, None, timeout=0.3)
                if not any(f"{cmd}," in l and l.endswith(",OK") for l in lines):
                    console.print(f"[yellow]  RMD {i} {cmd.split(',')[-1]}: request not acked "
                                   f"({_non_mon(lines) or 'no reply at all'})[/yellow]")

        if attempt == 1:
            console.print(f"[dim]Polling GIM IDs {sorted(pending_gim)} (fault + indicators)...[/dim]")
        for i in sorted(pending_gim):
            for cmd in (f"GIM,{i},FAULT", f"GIM,{i},IND,0", f"GIM,{i},IND,2"):
                send(ser, cmd)
                lines = read_lines_until(ser, None, timeout=0.3)
                if not any(",OK" in l for l in lines):
                    console.print(f"[yellow]  GIM {i} {cmd.split(',')[-1]}: request not acked "
                                   f"({_non_mon(lines) or 'no reply at all'})[/yellow]")

        time.sleep(0.15)
        new_rmd, new_gim = cmd_status(ser)
        seen_rmd |= new_rmd
        seen_gim |= new_gim
        pending_rmd -= new_rmd
        pending_gim -= new_gim

    if pending_rmd or pending_gim:
        console.print(f"[red]No reply after {retries + 1} attempt(s) — "
                       f"RMD {sorted(pending_rmd)} GIM {sorted(pending_gim)}. "
                       f"Check power/wiring/termination/ID for these specifically "
                       f"(CAN,monitor or 'monitor' will show whether anything comes back at all).[/red]")


def cmd_status(ser):
    """Print the aggregate status table. Returns (seen_rmd_ids, seen_gim_ids)
    so cmd_poll's retry loop can tell which ids actually had fresh state,
    as opposed to ids that simply didn't answer this round."""
    send(ser, "STATUS")
    lines = read_lines_until(ser, "STATUS,END")
    if not lines:
        console.print("[red]No response — is the firmware running?[/red]")
        return set(), set()

    rmd_tbl = Table(title="RMD (MG8016E-i6)", show_header=True, header_style="bold")
    for col in ["ID", "pos (rad)", "vel (rad/s)", "torque (Nm)", "temp (C)", "volt (V)", "err", "age (ms)"]:
        rmd_tbl.add_column(col, justify="right")
    gim_tbl = Table(title="GIM (GIM6010-6)", show_header=True, header_style="bold")
    for col in ["ID", "pos", "vel", "torque", "temp", "fault", "last indicator", "enabled", "age (ms)"]:
        gim_tbl.add_column(col, justify="right")

    any_rmd = any_gim = any_imu = False
    seen_rmd, seen_gim = set(), set()
    for line in lines:
        m = re.match(r"STATUS,RMD,(\d+),pos=([-\d.]+),vel=([-\d.]+),tq=([-\d.]+),"
                      r"temp=([-\d.]+),volt=([-\d.]+),err=0x([0-9a-fA-F]+),age_ms=(\d+)", line)
        if m:
            any_rmd = True
            seen_rmd.add(int(m.group(1)))
            rmd_tbl.add_row(m.group(1), m.group(2), m.group(3), m.group(4),
                             m.group(5), m.group(6), "0x" + m.group(7), m.group(8))
            continue
        m = re.match(r"STATUS,GIM,(\d+),pos=([-\d.]+),vel=([-\d.]+),tq=([-\d.]+),"
                      r"temp=([-\d.]+),fault=0x([0-9a-fA-F]+),ind(\d+)=([-\d.]+),"
                      r"enabled=(\d),valid=(\d),age_ms=(\d+)", line)
        if m:
            any_gim = True
            seen_gim.add(int(m.group(1)))
            gim_tbl.add_row(m.group(1), m.group(2), m.group(3), m.group(4), m.group(5),
                             "0x" + m.group(6), f"#{m.group(7)}={m.group(8)}",
                             "yes" if m.group(9) == "1" else "no", m.group(11))
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
    return seen_rmd, seen_gim


# Command byte -> name, for the RMD side (see ../CAN_config.md table 1.3).
# Only the ones this firmware actually sends/decodes are named; anything else
# shows up as a bare hex value, which is itself diagnostic (traffic on a
# hip's ID that this tool never asked for).
RMD_CMD_NAMES = {
    0x80: "MotorOff", 0x88: "MotorOn", 0x81: "MotorStop",
    0xA1: "TorqueCtrl", 0xA2: "SpeedCtrl", 0xA3: "AngleCtrl1", 0xA4: "AngleCtrl2",
    0x9A: "ReadState1", 0x9B: "ClearError", 0x90: "ReadEncoder",
    0x30: "ReadPID", 0x33: "ReadAccel",
}


def decode_rmd_9a(data):
    """Independent re-decode of a 0x9A (Read State1) reply — same formula as
    RmdMotor.cpp's rmd_rx_cb(), reimplemented here on purpose so a firmware
    decode bug would show up as this disagreeing with the STATUS table,
    rather than both silently agreeing on the same mistake.

    Voltage is byte2:byte3 at 0.01V/LSB, NOT the vendor doc's byte3:byte4 at
    0.1V/LSB — confirmed empirically via 'poll hips' against a known 41V
    supply (byte3 sat at a constant 0x10 across every unit, while byte2,
    documented as NULL, tracked real ~0.01V sensor noise between reads)."""
    temp = data[1] if data[1] < 128 else data[1] - 256   # int8_t
    volt_raw = data[2] | (data[3] << 8)                  # uint16_t, 0.01V/LSB
    err = data[7]
    return temp, volt_raw * 0.01, err


def decode_rmd_90(data):
    """Independent re-decode of a 0x90 (Read Encoder) reply — same formula
    as RmdMotor.cpp's rmd_rx_cb(). Confirmed against real hardware
    (2026-08-31): byte2-3=encoder, byte4-5=encoderRaw, byte6-7=encoderOffset
    all checked out self-consistently in a real reply. One correction from
    the vendor doc's generic 14-bit (0..16383) example: this drive's own GUI
    reports a 16-bit encoder, and raw values seen on the bus only make sense
    as a single-turn 0..360deg position at 16-bit scale."""
    enc = data[2] | (data[3] << 8)          # uint16_t, 16-bit (0..65535), LSB order
    pos_rad = enc * (2.0 * math.pi / 65536.0)
    return enc, pos_rad


def decode_rmd_30(data):
    """Decode a 0x30 (Read PID) reply per CAN_config.md sec 1.3: each gain
    is a single raw byte (0-255), no documented physical scale — the drive
    uses these internally, so what matters here is mainly whether they're
    zero (no control authority on that loop) or nonzero. NOT independently
    confirmed against real hardware yet — this is a straight vendor-doc
    decode, added specifically to check the 'why doesn't a position command
    produce any torque' question. Cross-check against the LK Motor Tool
    GUI's PID Setting page if the numbers look surprising."""
    return {
        "angle_kp": data[2], "angle_ki": data[3],
        "speed_kp": data[4], "speed_ki": data[5],
        "iq_kp": data[6], "iq_ki": data[7],
    }


def decode_rmd_33(data):
    """Decode a 0x33 (Read Acceleration) reply per CAN_config.md sec 1.3:
    DATA[4:7] = int32_t Accel, 1 dps/s/LSB, LSB byte order. NOT
    independently confirmed against real hardware yet — added for the same
    reason as decode_rmd_30(). This is almost certainly the same value as
    the LK Motor Tool GUI's 'Max Acceleration(dps/s)' field."""
    raw = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def cmd_poll_hips(ser, rmd_ids=None):
    """Poll the RMD hip motors with the CAN monitor running, and print
    exactly what came back on the wire for each one — raw bytes plus an
    independent decode — instead of only the firmware's cached STATUS row.

    This exists for debugging motors that 'aren't working very well': the
    normal STATUS table can't tell you whether a motor never replied at all,
    replied with something unexpected, or is replying fine but to a request
    that structurally can't carry the field you're looking at. Sends 0x9A
    (Read State1: temp/voltage/error), 0x90 (Read Encoder: position), 0x30
    (Read PID) and 0x33 (Read Acceleration) per hip — the last two were
    added specifically to check whether a zeroed position-loop gain or a
    zeroed acceleration limit explains a drive that acks a position command,
    stays enabled (solid LED), and never actually moves.
    """
    ids = rmd_ids or DEFAULT_RMD_IDS
    console.print(f"[bold]poll hips[/bold] — RMD IDs {ids}, watching raw bus-1 traffic\n")

    send(ser, "CAN,monitor,start")
    read_lines_until(ser, None, timeout=0.3)

    all_lines = []
    for i in ids:
        for cmd in (f"RMD,{i},STATUS", f"RMD,{i},ENCODER"):
            send(ser, cmd)
            lines = read_lines_until(ser, None, timeout=0.4)
            all_lines.extend(lines)
            ack = next((l for l in lines if l.startswith(f"{cmd},")), None)
            if ack is None or not ack.endswith(",OK"):
                console.print(f"[red]  RMD {i} {cmd.split(',')[-1]}: request send failed "
                               f"({ack or 'no ack line at all'})[/red]")

        # 0x30 (Read PID) and 0x33 (Read Acceleration) have no high-level
        # RMD,<id>,... wrapper in the firmware — inject them as raw frames
        # via the existing CAN,send primitive (same request/reply address,
        # 0x140+id, as everything else RMD). Added specifically to check
        # whether zeroed PID gains or a zeroed acceleration limit explain a
        # position command that gets acked but produces no motion.
        for label, cmd_byte in (("ReadPID", 0x30), ("ReadAccel", 0x33)):
            raw_id = f"{0x140 + i:x}"
            send(ser, f"CAN,send,{raw_id},0,{cmd_byte:x},0,0,0,0,0,0,0")
            lines = read_lines_until(ser, None, timeout=0.4)
            all_lines.extend(lines)
            ack = next((l for l in lines if l.startswith("CAN,SEND,")), None)
            if ack is None or not ack.endswith(",OK"):
                console.print(f"[red]  RMD {i} {label}: request send failed "
                               f"({ack or 'no ack line at all'})[/red]")

    # No extra settle sleep here: each per-id read_lines_until() above already
    # dwells ~0.25-0.3s after its reply lands (LK-TECH replies land in well
    # under 1ms per the spec), and stacking a further idle sleep on top of
    # that risks crossing the firmware's 500ms host watchdog and triggering
    # a spurious rmd_stop_all() (0x81 MotorStop to every hip) that shows up
    # as confusing extra traffic in the dump below.
    send(ser, "CAN,monitor,stop")
    all_lines.extend(read_lines_until(ser, None, timeout=0.3))

    # Parse every MON,... line captured during the whole window and bucket by
    # which polled hip ID's arbitration ID (0x140+id) it landed on.
    frames_by_id = {i: [] for i in ids}
    other_bus1_ids = {}
    for line in all_lines:
        if not line.startswith("MON,"):
            continue
        parts = line[4:].split(",")
        if len(parts) < 5:
            continue
        t_ms, bus, cid_str, _is_ext, dlc = parts[:5]
        if bus != "1":
            continue   # hips are bus 1; ignore bus-2 IMU traffic here
        try:
            cid = int(cid_str, 16)
            data = [int(x, 16) for x in parts[5:5 + int(dlc)]]
        except ValueError:
            continue
        matched_id = next((i for i in ids if cid == 0x140 + i), None)
        if matched_id is not None:
            frames_by_id[matched_id].append((int(t_ms), data))
        else:
            other_bus1_ids[cid] = other_bus1_ids.get(cid, 0) + 1

    console.print("[bold]Raw CAN traffic per hip[/bold]")
    for i in ids:
        console.print(f"\n[cyan]ID {i}[/cyan] (arbitration 0x{0x140 + i:03x}):")
        frames = frames_by_id[i]
        if not frames:
            console.print(f"  [red]No frame seen on the bus for this ID at all during the "
                           f"request window — check power/CAN wiring/termination/configured "
                           f"ID for drive {i} specifically (a bus-wide problem would show up "
                           f"as every ID missing, not just this one).[/red]")
            continue
        for t_ms, data in frames:
            hexs = " ".join(f"{b:02x}" for b in data)
            cmd = data[0] if data else None
            name = RMD_CMD_NAMES.get(cmd, f"0x{cmd:02x}" if cmd is not None else "?")
            tag = ""
            if len(data) == 8 and all(b == 0 for b in data[1:]):
                tag = "  [dim](all-zero payload after the command byte — this is what OUR " \
                      "own request looks like, not necessarily a reply)[/dim]"
            # Square brackets are Rich markup syntax — a hex dump wrapped in
            # them can look like an unresolved style tag (e.g. "[ff]") and
            # get silently swallowed instead of printed. Angle brackets
            # avoid that regardless of byte content.
            console.print(f"  t={t_ms:>8}ms  dlc={len(data)}  bytes=<{hexs}>  cmd={name}{tag}")
            if cmd == 0x9A and len(data) == 8:
                temp, volt, err = decode_rmd_9a(data)
                flag = ""
                if volt < 20.0:
                    flag = "  [yellow]<- suspiciously low for a ~41V rail; check V+/V- " \
                           "actually reaches this drive, separate from the CAN wiring[/yellow]"
                console.print(f"    decoded: temp={temp}C volt={volt:.1f}V err=0x{err:02x}{flag}")
            elif cmd == 0x90 and len(data) == 8:
                enc, pos_rad = decode_rmd_90(data)
                console.print(f"    decoded: encoder={enc} (0..65535)  "
                               f"pos={pos_rad:.3f} rad ({pos_rad * 180.0 / math.pi:.1f} deg)")
            elif cmd == 0x30 and len(data) == 8:
                pid = decode_rmd_30(data)
                zero_flag = ""
                if pid["angle_kp"] == 0:
                    zero_flag = "  [yellow]<- angle_kp=0 means the position loop applies " \
                                "ZERO corrective torque for any position error — this alone " \
                                "would fully explain an accepted position command producing " \
                                "no motion[/yellow]"
                console.print(f"    decoded (raw bytes, no documented physical scale): "
                               f"angle Kp={pid['angle_kp']} Ki={pid['angle_ki']}  "
                               f"speed Kp={pid['speed_kp']} Ki={pid['speed_ki']}  "
                               f"iq Kp={pid['iq_kp']} Ki={pid['iq_ki']}{zero_flag}")
            elif cmd == 0x33 and len(data) == 8:
                accel = decode_rmd_33(data)
                zero_flag = ""
                if accel == 0:
                    zero_flag = "  [yellow]<- 0 dps/s means the drive isn't allowed to " \
                                "accelerate at all in position/speed mode — this alone would " \
                                "fully explain an accepted position command producing no " \
                                "motion[/yellow]"
                console.print(f"    decoded: accel={accel} dps/s{zero_flag}")
        if len(frames) == 1 and all(b == 0 for b in frames[0][1][1:]):
            console.print(f"  [yellow]Only one frame seen for this ID, and it looks like our "
                           f"own request echoed back — no distinct reply from the drive itself.[/yellow]")

    if other_bus1_ids:
        console.print(f"\n[dim]Other bus-1 traffic seen (not one of the polled hip IDs): "
                       + ", ".join(f"0x{cid:03x} x{n}" for cid, n in other_bus1_ids.items()) + "[/dim]")

    console.print(f"\n[dim]Run 'status' to see these same IDs' cached STATUS,RMD row for "
                  f"comparison against the decode above.[/dim]")


# Command byte -> name, for the GIM side (see ../CAN_config.md §2.2).
GIM_CMD_NAMES = {
    0x91: "StartMotor", 0x92: "StopMotor", 0x97: "StopControl",
    0x93: "TorqueCtrl", 0x94: "SpeedCtrl", 0x95: "PositionCtrl",
    0xB2: "GetFault", 0xB3: "AckFault", 0xB4: "RetrieveIndicator",
}


def decode_gim_b2(data):
    """Decode a 0xB2 (Get Fault) reply per CAN_config.md §2.7:
    DATA[1]=RES, DATA[2]=FaultNo bitmask."""
    return data[1], data[2]


def decode_gim_b4(data):
    """Decode a 0xB4 (Retrieve Indicator) reply per CAN_config.md §2.7:
    DATA[1]=IndID (echoed), DATA[2]=RES, DATA[4:7]=IEEE float value, LSB
    (little-endian) byte order — matches GimMotor.cpp's memcpy decode on
    this little-endian STM32."""
    ind_id, res = data[1], data[2]
    val = struct.unpack("<f", bytes(data[4:8]))[0]
    return ind_id, res, val


def cmd_poll_gim(ser, gim_ids=None):
    """Poll the GIM wheel motors with the CAN monitor running — the same
    raw-byte methodology as poll_hips, applied to GIM. Sends 0xB2 (Get
    Fault) and 0xB4 (Retrieve Indicator: bus voltage, motor temp) per id,
    all non-motion, safe reads, and shows exactly what came back (if
    anything) on each polled CAN ID's arbitration ID *and* its configured
    reply ID (Host/Master CAN ID), since those are confirmed to differ on
    this project's wheels (2026-08-31, via poll_wheels).

    This also calls ensure_gim_reply_ids() first, so the firmware itself
    starts correctly attributing these replies to the right id in its own
    STATUS cache — not just this command's own raw capture.
    """
    ids = gim_ids or DEFAULT_GIM_IDS
    reply_of = {i: DEFAULT_GIM_MASTER_IDS.get(i, i) for i in ids}
    console.print(f"[bold]poll gim[/bold] — GIM IDs {ids} "
                  f"(reply IDs {[reply_of[i] for i in ids]}), watching raw bus-1 traffic\n")

    ensure_gim_reply_ids(ser, ids)

    send(ser, "CAN,monitor,start")
    read_lines_until(ser, None, timeout=0.3)

    all_lines = []
    for i in ids:
        for cmd, label in ((f"GIM,{i},FAULT", "FAULT"),
                            (f"GIM,{i},IND,0", "IND 0 (bus V)"),
                            (f"GIM,{i},IND,2", "IND 2 (motor temp)")):
            send(ser, cmd)
            lines = read_lines_until(ser, None, timeout=0.4)
            all_lines.extend(lines)
            if not any(l.startswith(f"GIM,{i},") and l.endswith(",OK") for l in lines):
                console.print(f"[red]  GIM {i} {label}: request send failed "
                               f"({_non_mon(lines) or 'no reply at all'})[/red]")

    send(ser, "CAN,monitor,stop")
    all_lines.extend(read_lines_until(ser, None, timeout=0.3))

    # Parse every MON,... line captured and bucket by which polled GIM CAN
    # ID's *or its reply ID's* arbitration ID it landed on -- confirmed
    # (2026-08-31) that these can legitimately differ per motor.
    watch_ids = sorted(set(ids) | set(reply_of.values()))
    frames_by_id = {i: [] for i in watch_ids}
    other_bus1_ids = {}
    for line in all_lines:
        if not line.startswith("MON,"):
            continue
        parts = line[4:].split(",")
        if len(parts) < 5:
            continue
        t_ms, bus, cid_str, _is_ext, dlc = parts[:5]
        if bus != "1":
            continue
        try:
            cid = int(cid_str, 16)
            data = [int(x, 16) for x in parts[5:5 + int(dlc)]]
        except ValueError:
            continue
        if cid in frames_by_id:
            frames_by_id[cid].append((int(t_ms), data))
        else:
            other_bus1_ids[cid] = other_bus1_ids.get(cid, 0) + 1

    console.print("[bold]Raw CAN traffic per GIM motor[/bold]")
    for i in ids:
        rid = reply_of[i]
        label = f"CAN ID {i}" if rid == i else f"CAN ID {i} / reply ID {rid}"
        console.print(f"\n[cyan]{label}[/cyan]:")
        frames = frames_by_id[i] + (frames_by_id[rid] if rid != i else [])
        frames.sort(key=lambda td: td[0])
        if not frames:
            console.print(f"  [red]No frame seen on either ID during the request window. "
                           f"Check power/CAN wiring/termination for this drive specifically.[/red]")
            continue
        for t_ms, data in frames:
            hexs = " ".join(f"{b:02x}" for b in data)
            cmd = data[0] if data else None
            name = GIM_CMD_NAMES.get(cmd, f"0x{cmd:02x}" if cmd is not None else "?")
            tag = ""
            if len(data) == 8 and all(b == 0 for b in data[1:]):
                tag = "  [dim](all-zero payload after the command byte — this is what OUR " \
                      "own request looks like, not necessarily a reply)[/dim]"
            console.print(f"  t={t_ms:>8}ms  dlc={len(data)}  bytes=<{hexs}>  cmd={name}{tag}")
            if cmd == 0xB2 and len(data) == 8:
                res, fault = decode_gim_b2(data)
                console.print(f"    decoded: RES=0x{res:02x} fault=0x{fault:02x}")
            elif cmd == 0xB4 and len(data) == 8:
                ind_id, res, val = decode_gim_b4(data)
                console.print(f"    decoded: IndID={ind_id} RES=0x{res:02x} value={val:.3f}")
        if len(frames) == 1 and all(b == 0 for b in frames[0][1][1:]):
            console.print(f"  [yellow]Only one frame seen for this ID, and it looks like our "
                           f"own request echoed back — no distinct reply from the drive itself.[/yellow]")

    if other_bus1_ids:
        console.print(f"\n[dim]Other bus-1 traffic seen (not one of the polled GIM IDs): "
                       + ", ".join(f"0x{cid:03x} x{n}" for cid, n in other_bus1_ids.items()) + "[/dim]")

    console.print(f"\n[dim]Run 'status' to see these same IDs' cached STATUS,GIM row for "
                  f"comparison against the decode above.[/dim]")


def cmd_poll_wheels(ser, pairs=None):
    """Poll GIM wheel motors whose CAN ID and Host/Master CAN ID are
    configured as DIFFERENT values, checking BOTH ids per wheel for a
    reply. Commands are always sent to the CAN ID (that's the motor's own
    listening address, per the protocol) — this reports which of {CAN ID,
    Master CAN ID} actually shows a reply frame, settling empirically
    whether GIM replies land on the id you sent to or a separately
    configured master/host id.

    pairs: list of (can_id, master_id) tuples. Defaults to this project's
    current wheel configuration: wheel 1 = CAN ID 15 / Master CAN ID 16,
    wheel 2 = CAN ID 20 / Master CAN ID 21.
    """
    pairs = pairs or [(15, 16), (20, 21)]
    watch_ids = sorted({i for pair in pairs for i in pair})
    console.print(f"[bold]poll wheels[/bold] — pairs {pairs}, watching IDs {watch_ids} on bus 1\n")

    send(ser, "CAN,monitor,start")
    read_lines_until(ser, None, timeout=0.3)

    all_lines = []
    for can_id, _master_id in pairs:
        for cmd, label in ((f"GIM,{can_id},FAULT", "FAULT"),
                            (f"GIM,{can_id},IND,0", "IND 0 (bus V)"),
                            (f"GIM,{can_id},IND,2", "IND 2 (motor temp)")):
            send(ser, cmd)
            lines = read_lines_until(ser, None, timeout=0.4)
            all_lines.extend(lines)
            if not any(l.startswith(f"GIM,{can_id},") and l.endswith(",OK") for l in lines):
                console.print(f"[red]  CAN ID {can_id} {label}: request send failed "
                               f"({_non_mon(lines) or 'no reply at all'})[/red]")

    send(ser, "CAN,monitor,stop")
    all_lines.extend(read_lines_until(ser, None, timeout=0.3))

    frames_by_id = {i: [] for i in watch_ids}
    other_bus1_ids = {}
    for line in all_lines:
        if not line.startswith("MON,"):
            continue
        parts = line[4:].split(",")
        if len(parts) < 5:
            continue
        t_ms, bus, cid_str, _is_ext, dlc = parts[:5]
        if bus != "1":
            continue
        try:
            cid = int(cid_str, 16)
            data = [int(x, 16) for x in parts[5:5 + int(dlc)]]
        except ValueError:
            continue
        if cid in frames_by_id:
            frames_by_id[cid].append((int(t_ms), data))
        else:
            other_bus1_ids[cid] = other_bus1_ids.get(cid, 0) + 1

    def print_frames(i):
        frames = frames_by_id[i]
        if not frames:
            console.print(f"    [dim]no frames seen[/dim]")
            return frames
        for t_ms, data in frames:
            hexs = " ".join(f"{b:02x}" for b in data)
            cmd = data[0] if data else None
            name = GIM_CMD_NAMES.get(cmd, f"0x{cmd:02x}" if cmd is not None else "?")
            tag = ""
            if len(data) == 8 and all(b == 0 for b in data[1:]):
                tag = "  [dim](looks like our own echoed request)[/dim]"
            console.print(f"    t={t_ms:>8}ms  bytes=<{hexs}>  cmd={name}{tag}")
            if cmd == 0xB2 and len(data) == 8:
                res, fault = decode_gim_b2(data)
                console.print(f"      decoded: RES=0x{res:02x} fault=0x{fault:02x}")
            elif cmd == 0xB4 and len(data) == 8:
                ind_id, res, val = decode_gim_b4(data)
                console.print(f"      decoded: IndID={ind_id} RES=0x{res:02x} value={val:.3f}")
        return frames

    def has_distinct_reply(frames):
        return any(len(d) == 8 and not all(b == 0 for b in d[1:]) for _, d in frames)

    for can_id, master_id in pairs:
        console.print(f"\n[bold cyan]Wheel: CAN ID {can_id} / Master CAN ID {master_id}[/bold cyan]")
        console.print(f"  [CAN ID] 0x{can_id:03x}:")
        can_frames = print_frames(can_id)
        console.print(f"  [Master CAN ID] 0x{master_id:03x}:")
        master_frames = print_frames(master_id)

        can_reply = has_distinct_reply(can_frames)
        master_reply = has_distinct_reply(master_frames)
        if master_reply and not can_reply:
            console.print(f"  [green]VERDICT: replies land on Master CAN ID {master_id}, "
                           f"not CAN ID {can_id}.[/green]")
        elif can_reply and not master_reply:
            console.print(f"  [green]VERDICT: replies land on CAN ID {can_id}, as this tool "
                           f"has been assuming for GIM so far.[/green]")
        elif can_reply and master_reply:
            console.print(f"  [yellow]VERDICT: distinct-looking replies seen on BOTH ids — "
                           f"unusual, look at the raw bytes above.[/yellow]")
        else:
            console.print(f"  [red]VERDICT: no reply seen on either id — check power/wiring "
                           f"for this specific wheel (expected if it's not connected right now).[/red]")

    if other_bus1_ids:
        console.print(f"\n[dim]Other bus-1 traffic seen (not one of the watched IDs): "
                       + ", ".join(f"0x{cid:03x} x{n}" for cid, n in other_bus1_ids.items()) + "[/dim]")


_RMD_STATUS_RE = re.compile(
    r"STATUS,RMD,(\d+),pos=([-\d.]+),vel=([-\d.]+),tq=([-\d.]+),"
    r"temp=([-\d.]+),volt=([-\d.]+),err=0x([0-9a-fA-F]+),age_ms=(\d+)")


def fetch_rmd_state(ser, target_id):
    """Send bare STATUS and parse out just one RMD id's cached row. Returns
    a dict, or None if that id hasn't reported anything yet. This reads
    whatever the firmware already has cached — see refresh_rmd_state() if
    you need a fresh reading first."""
    send(ser, "STATUS")
    lines = read_lines_until(ser, "STATUS,END")
    for line in lines:
        m = _RMD_STATUS_RE.match(line)
        if m and int(m.group(1)) == target_id:
            return {
                "pos": float(m.group(2)), "vel": float(m.group(3)), "tq": float(m.group(4)),
                "temp": float(m.group(5)), "volt": float(m.group(6)),
                "err": int(m.group(7), 16), "age_ms": int(m.group(8)),
            }
    return None


def refresh_rmd_state(ser, target_id):
    """Trigger fresh 0x9A (temp/volt/err) and 0x90 (encoder/position) reads
    for one RMD id, then return its parsed STATUS row. Use this rather than
    fetch_rmd_state() alone whenever the reading needs to reflect the
    motor's *current* state (e.g. right before deciding whether it's safe
    to command motion), since the plain STATUS command only reports
    whatever was last cached — which could be stale."""
    send(ser, f"RMD,{target_id},STATUS")
    read_lines_until(ser, None, timeout=0.3)
    send(ser, f"RMD,{target_id},ENCODER")
    read_lines_until(ser, None, timeout=0.3)
    return fetch_rmd_state(ser, target_id)


def cmd_test_hips(ser, rmd_ids=None, speed_dps=30.0, target_deg=120.0):
    """Move each RMD hip through 0deg then target_deg, ONE AT A TIME,
    reporting its encoder position before/after each move.

    Uses SINGLETURN (0xA6, Single Loop Angle Control 2) — an ABSOLUTE
    target within one revolution (0..360deg), the same reference frame the
    0x90 encoder reads. This command previously used POSITION (0xA4, Multi
    Loop Angle Control), which targets an ABSOLUTE MULTI-TURN position —
    after a session of hand-spinning the shaft, "0deg"/"120deg" in that
    frame could be hundreds of degrees of real travel away, which is
    indistinguishable from "barely moving" if you're comparing against the
    single-turn encoder reading. SINGLETURN removes that ambiguity, at the
    cost of needing an explicit turn direction (0xA6 doesn't auto-pick the
    short way like 0xA4 does) — this always commands CW; if a target is
    behind the motor in the CW sense it'll go most of the way around rather
    than the short way, which is also why the dwell estimate below uses the
    CW-only distance, not a shortest-path guess.

    This sends real motion commands — not a passive read like the other
    'poll'/'status' commands. Safety choices made on purpose:
      - one motor at a time, never all 4 simultaneously, so a problem with
        one doesn't compound with the others and you can watch/react to
        each individually (matches Motor_tool/plan.md's bring-up ordering)
      - a conservative default speed (30 dps ~= 5 RPM)
      - checks for a fresh, fault-free status before moving each motor, and
        skips (doesn't move) any motor that's stale, faulted, or unheard
        from — never commands blind
      - an explicit confirmation prompt before anything moves

    The encoder decode this depends on (decode_rmd_90()/RmdMotor.cpp) is
    confirmed against real hardware. Still worth watching the before/after
    readings critically though: a real, actively-controlled move to a
    target should show up as a large change that stops near the commanded
    value — small, target-independent drift (e.g. the same direction
    regardless of what was commanded) is a sign the drive isn't actually
    applying closed-loop torque, not a sign the position command "sort of"
    worked.
    """
    ids = rmd_ids or DEFAULT_RMD_IDS
    speed_rads = speed_dps * math.pi / 180.0

    console.print(f"[bold yellow]test hips[/bold yellow] will move RMD IDs {ids}, ONE AT A TIME, "
                  f"to 0 deg then {target_deg} deg, at {speed_dps} dps.")
    console.print("[yellow]This is a real motion command. Make sure each hip is free to rotate "
                  "through this range without hitting a hard stop or fouling on the chassis — "
                  "120 degrees is a large sweep for a hip joint.[/yellow]")
    try:
        input("Press Enter to start, Ctrl-C to abort... ")
    except KeyboardInterrupt:
        console.print("\n[dim]Aborted — no motion commands sent.[/dim]")
        return

    for i in ids:
        console.print(f"\n[cyan]Hip {i}[/cyan]")

        before = refresh_rmd_state(ser, i)
        if before is None:
            console.print(f"  [red]No response from hip {i} — skipping "
                           f"(run 'poll hips' first to confirm it's on the bus).[/red]")
            continue
        if before["err"] != 0:
            console.print(f"  [red]err=0x{before['err']:02x} is already set — skipping this "
                           f"motor. Clear it first with 'rmd {i} clearerr' if you want to test it.[/red]")
            continue
        console.print(f"  starting pos={before['pos']:.3f} rad ({before['pos'] * 180.0 / math.pi:.1f} deg), "
                       f"temp={before['temp']}C, volt={before['volt']}V")

        send(ser, f"RMD,{i},RESUME")   # 0x88 motor on — no-op if already on
        resume_ack = read_lines_until(ser, None, timeout=0.3)
        if not any(l.endswith(",OK") for l in resume_ack):
            console.print(f"  [red]RESUME (motor-on) not acked ({resume_ack}) — the drive may "
                           f"still be in the OFF state, in which case it will accept the position "
                           f"command below but never actually move (per the K-TECH manual, OFF "
                           f"state still replies to commands but performs no action).[/red]")

        current_deg = before["pos"] * 180.0 / math.pi
        for label_deg in (0.0, target_deg):
            tgt_deg_wrapped = label_deg % 360.0
            tgt_rad = tgt_deg_wrapped * math.pi / 180.0
            # SINGLETURN always commands CW here (see docstring), so
            # estimate the wait using the CW-only distance under that same
            # convention, not a shortest-path guess — capped at 20s either
            # way as a sanity ceiling.
            delta_deg = (tgt_deg_wrapped - current_deg) % 360.0
            dwell_s = min(20.0, max(2.0, delta_deg / speed_dps * 1.4 + 1.0))

            send(ser, f"RMD,{i},SINGLETURN,{tgt_rad},{speed_rads},1")
            ack = read_lines_until(ser, None, timeout=0.3)
            if not any(l.endswith(",OK") for l in ack):
                console.print(f"  [red]Position command to {label_deg} deg not acked ({ack})[/red]")
                continue

            console.print(f"  -> commanded {label_deg} deg (CW), waiting ~{dwell_s:.1f}s...")
            time.sleep(dwell_s)

            after = refresh_rmd_state(ser, i)
            if after is None:
                console.print(f"  [red]No response after commanding {label_deg} deg.[/red]")
                continue
            achieved_deg = after["pos"] * 180.0 / math.pi
            fault = f"  [red]err=0x{after['err']:02x}[/red]" if after["err"] != 0 else ""
            console.print(f"  reached pos={after['pos']:.3f} rad ({achieved_deg:.1f} deg), "
                          f"target was {label_deg} deg, temp={after['temp']}C{fault}")
            current_deg = achieved_deg

    console.print(f"\n[dim]Done. Motors are left holding their last commanded position — send "
                  f"'rmd <id> stop' or 'rmd <id> off' if you want to release them.[/dim]")


def cmd_test_drift(ser, rmd_ids=None, duration_s=5.0):
    """Passively sample each RMD hip's encoder for duration_s seconds and
    report whether the reading moves — no motion commands sent at all. This
    isolates the encoder/sensor question from the 'can the drive actually
    produce torque' question test_hips answers: if the reading drifts here
    while the motor is physically still, that's sensor noise or a decode
    issue, not anything to do with PID gains, torque limits, or enable
    state (all already ruled out or ruled plausible separately).
    """
    ids = rmd_ids or DEFAULT_RMD_IDS
    console.print(f"[bold]test drift[/bold] — passively sampling RMD IDs {ids} encoder for "
                  f"{duration_s:.1f}s. No motion commands are sent. Leave the motor(s) "
                  f"physically still and undisturbed for the whole window.\n")

    samples = {i: [] for i in ids}   # id -> [(t_s, pos_deg), ...]
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        for i in ids:
            send(ser, f"RMD,{i},ENCODER")
            read_lines_until(ser, None, timeout=0.15)
            st = fetch_rmd_state(ser, i)
            if st is not None:
                samples[i].append((time.monotonic() - start, st["pos"] * 180.0 / math.pi))
        time.sleep(0.05)

    for i in ids:
        s = samples[i]
        console.print(f"[cyan]ID {i}[/cyan] — {len(s)} samples over {duration_s:.1f}s")
        if not s:
            console.print("  [red]No samples captured — id never responded.[/red]\n")
            continue
        for t, deg in s:
            console.print(f"  t={t:5.2f}s  pos={deg:7.2f} deg")

        degs = [deg for _, deg in s]
        first, last, lo, hi = degs[0], degs[-1], min(degs), max(degs)
        span = hi - lo
        console.print(f"  first={first:.2f} deg  last={last:.2f} deg  "
                      f"delta(first->last)={last - first:+.2f} deg  "
                      f"range={lo:.2f}..{hi:.2f} deg (span {span:.2f} deg)")
        if span > 1.0:
            console.print(f"  [yellow]Over 1 degree of movement while nominally still — this is "
                           f"either real sensor drift/noise, or the motor wasn't perfectly "
                           f"stationary the whole window.[/yellow]\n")
        else:
            console.print(f"  [green]Under 1 degree of movement — encoder reading looks "
                           f"stable at rest.[/green]\n")


def cmd_encoder_watch(ser, rmd_id, watch_seconds=15.0, gear_ratio=6.0):
    """Continuously read one RMD hip's encoder and print every sample live
    (not buffered to a summary at the end, unlike test_drift) — built for
    turning the shaft by hand while watching the numbers update in real
    time, e.g. to check the encoder against a known physical rotation.

    0x90 reports SINGLE-TURN position (0-360deg, wraps every revolution) on
    the motor/rotor side, BEFORE the MG8016E-i6's 1:6 gearbox. A 180deg
    output-shaft turn is ~1080deg = 3 full encoder revolutions, i.e. 3 wraps
    of the raw 0-360 reading — printed as raw numbers alone, a fast multi-
    revolution turn looks like chaotic noise even though the individual
    samples are fine (confirmed 2026-08-31: reconstructing a real ~180deg
    hand turn by hand-unwrapping the wraps landed within a few degrees of
    the reported motion). The primary number shown is now the gearbox-
    corrected OUTPUT-shaft angle: unwrap the raw encoder (stitch across
    360deg wraps, assuming no single sample-to-sample step exceeds 180deg —
    true at normal hand speed with this tool's ~5Hz sample rate, but a very
    fast spin could alias the wrong way), then divide by gear_ratio. This
    is a RELATIVE angle from wherever the shaft was when the command
    started — a single-turn encoder reading can't recover an absolute
    output-shaft position on its own, since the same raw value corresponds
    to `gear_ratio` different possible output positions 360/gear_ratio deg
    apart, without also tracking which one you started in.
    """
    console.print(f"[bold]encoder watch[/bold] — RMD ID {rmd_id}, {watch_seconds:.1f}s "
                  f"(Ctrl-C to stop early). Turn the shaft by hand now.\n")
    start = time.monotonic()
    first_deg = None
    unwrapped = None
    try:
        while time.monotonic() - start < watch_seconds:
            send(ser, f"RMD,{rmd_id},ENCODER")
            read_lines_until(ser, None, timeout=0.15)
            st = fetch_rmd_state(ser, rmd_id)
            if st is not None:
                raw_deg = (st["pos"] * 180.0 / math.pi) % 360.0
                if unwrapped is None:
                    unwrapped = raw_deg
                    first_deg = raw_deg
                else:
                    # Standard phase-unwrap: pick whichever of raw+k*360
                    # (k in {-1,0,+1}) is closest to the running total,
                    # i.e. assume the true step was under half a revolution.
                    step = raw_deg - (unwrapped % 360.0)
                    if step > 180.0:
                        step -= 360.0
                    elif step < -180.0:
                        step += 360.0
                    unwrapped += step
                encoder_total = unwrapped - first_deg
                output_deg = encoder_total / gear_ratio
                t = time.monotonic() - start
                console.print(f"  t={t:5.2f}s  output={output_deg:+7.2f} deg  "
                               f"(encoder raw={raw_deg:7.2f} deg, unwrapped total={encoder_total:+8.2f} deg)")
            time.sleep(0.05)
    except KeyboardInterrupt:
        console.print("\n[dim]Stopped early (Ctrl-C).[/dim]")

    console.print(f"\n[dim]'output' (gearbox-corrected, /{gear_ratio:.0f}) is the number to compare "
                  f"against how far you actually turned the shaft by hand — it's a RELATIVE angle "
                  f"from the start of this command, not an absolute output-shaft position (a "
                  f"single-turn encoder can't recover that on its own). 'encoder raw'/'unwrapped "
                  f"total' are the underlying motor-side numbers, shown for reference.[/dim]")


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
  poll hips [rmd_id ...]                  poll RMD hips with CAN monitor running; show raw bytes + independent decode per ID
  poll gim [gim_id ...]                   poll GIM wheels with CAN monitor running; show raw bytes + independent decode per ID
  poll wheels [can_id master_id ...]      check BOTH CAN ID and Master CAN ID per wheel for replies (default: 15/16, 20/21)
  test hips [rmd_id ...]                  MOVES hips (one at a time) to 0deg then 120deg; confirms before moving anything
  test drift [rmd_id ...]                 no motion — samples encoder for 5s, reports if it moves while nominally still
  scan [seconds]                          ID scanner on the active bus
  monitor [seconds]                       live raw frame dump (Ctrl-C to stop)
  bus <1|2>                               select active CAN bus
  send <id_hex> <ext0|1> <b0>..<b7>       inject a raw 8-byte CAN frame
  selftest                                internal-loopback test of the active bus (no wiring needed)
  imu [seconds]                           IMX5 (bus 2) reading, once or watched live
  encoder <rmd_id> [seconds] [gear_ratio] live encoder read (default 15s, ratio 6) — shows gearbox-corrected OUTPUT-shaft angle

  rmd <id> torque <Nm>                    MG8016E-i6 torque command
  rmd <id> torqueraw <ratio>              MG8016E-i6 torque, raw -2048..2048 (confirmed, -33..33A for MG series)
  rmd scale [ratio_per_Nm]                get/set the Nm->ratio scale for 'rmd torque'
  rmd <id> vel <rad/s>                    MG8016E-i6 velocity command
  rmd <id> pos <rad> <maxspeed_rad/s>     MG8016E-i6 position command — ABSOLUTE multi-turn target (0xA4)
  rmd <id> singleturn <rad> <maxspd> [cw0/1]  MG8016E-i6 position command — ABSOLUTE single-turn 0..360deg (0xA6, default cw)
  rmd <id> increment <rad>                MG8016E-i6 position command — RELATIVE to current position (0xA7)
  rmd <id> stop | off | resume            MG8016E-i6 zero-out / disable / re-enable
  rmd <id> status                         MG8016E-i6 request status1 read
  rmd <id> clearerr                       MG8016E-i6 clear latched error
  rmd <id> encoder                        MG8016E-i6 request current encoder position

  gim <id> start | stop | pause            GIM6010-6 enter/exit running state / pause current command
  gim <id> torque <Nm> [duration_ms]       GIM6010-6 torque command (real Nm, confirmed)
  gim <id> velocity <rad/s> [duration_ms]  GIM6010-6 velocity command
  gim <id> position <rad> [duration_ms]    GIM6010-6 position command
  gim <id> fault | ackfault                GIM6010-6 request/clear fault status
  gim <id> ind <ind_id>                    GIM6010-6 request one runtime indicator (0=bus V, 2=motor temp, 14=speed RPM, ...)
  gim <id> masterid <reply_id>             set the SID this id's replies actually arrive on (Host/Master CAN ID, confirmed to differ per motor)
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
        elif cmd == "poll" and len(parts) > 1 and parts[1].lower() == "hips":
            rmd_ids = [int(p) for p in parts[2:]] if len(parts) > 2 else DEFAULT_RMD_IDS
            cmd_poll_hips(ser, rmd_ids)
        elif cmd == "poll" and len(parts) > 1 and parts[1].lower() == "gim":
            gim_ids = [int(p) for p in parts[2:]] if len(parts) > 2 else DEFAULT_GIM_IDS
            cmd_poll_gim(ser, gim_ids)
        elif cmd == "poll" and len(parts) > 1 and parts[1].lower() == "wheels":
            nums = [int(p) for p in parts[2:]]
            if len(nums) >= 2 and len(nums) % 2 == 0:
                pairs = [(nums[i], nums[i + 1]) for i in range(0, len(nums), 2)]
            else:
                if nums:
                    console.print("[yellow]poll wheels needs pairs (can_id master_id ...) — "
                                   "using default pairs instead.[/yellow]")
                pairs = None
            cmd_poll_wheels(ser, pairs)
        elif cmd == "poll":
            rmd_ids = [int(p) for p in parts[1:]] if len(parts) > 1 else DEFAULT_RMD_IDS
            cmd_poll(ser, rmd_ids, DEFAULT_GIM_IDS)
        elif cmd == "test" and len(parts) > 1 and parts[1].lower() == "hips":
            rmd_ids = [int(p) for p in parts[2:]] if len(parts) > 2 else DEFAULT_RMD_IDS
            cmd_test_hips(ser, rmd_ids)
        elif cmd == "test" and len(parts) > 1 and parts[1].lower() == "drift":
            rmd_ids = [int(p) for p in parts[2:]] if len(parts) > 2 else DEFAULT_RMD_IDS
            cmd_test_drift(ser, rmd_ids)
        elif cmd == "scan":
            cmd_scan(ser, float(parts[1]) if len(parts) > 1 else 1.0)
        elif cmd == "monitor":
            cmd_monitor(ser, float(parts[1]) if len(parts) > 1 else 5.0)
        elif cmd == "imu":
            cmd_imu(ser, float(parts[1]) if len(parts) > 1 else None)
        elif cmd == "encoder" and len(parts) > 1:
            secs = float(parts[2]) if len(parts) > 2 else 15.0
            gear = float(parts[3]) if len(parts) > 3 else 6.0
            cmd_encoder_watch(ser, int(parts[1]), secs, gear)
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
        console.print("[red]Usage: rmd <id> <torque|torqueraw|vel|pos|singleturn|increment|stop|off|resume|status|clearerr|encoder> ...[/red]")
        return
    mid, sub = args[0], args[1].lower()
    cmd_map = {
        "torque":     lambda: f"RMD,{mid},TORQUE,{args[2]}",
        "torqueraw":  lambda: f"RMD,{mid},TORQUERAW,{args[2]}",
        "vel":        lambda: f"RMD,{mid},VELOCITY,{args[2]}",
        "pos":        lambda: f"RMD,{mid},POSITION,{args[2]},{args[3]}",
        "singleturn": lambda: f"RMD,{mid},SINGLETURN,{args[2]},{args[3]}"
                               + (f",{args[4]}" if len(args) > 4 else ""),
        "increment":  lambda: f"RMD,{mid},INCREMENT,{args[2]}",
        "stop":       lambda: f"RMD,{mid},STOP",
        "off":        lambda: f"RMD,{mid},OFF",
        "resume":     lambda: f"RMD,{mid},RESUME",
        "status":     lambda: f"RMD,{mid},STATUS",
        "clearerr":   lambda: f"RMD,{mid},CLEARERR",
        "encoder":    lambda: f"RMD,{mid},ENCODER",
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
        console.print("[red]Usage: gim <id> <start|stop|pause|torque|velocity|position|fault|ackfault|ind|masterid> ...[/red]")
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
        elif sub == "masterid":
            send(ser, f"GIM,{mid},MASTERID,{args[2]}")
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
