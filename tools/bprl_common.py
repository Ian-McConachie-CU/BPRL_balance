#!/usr/bin/env python3
"""
bprl_common — shared utilities for BPRL ground tools.

Imported by all bprl_*.py scripts. Not run directly.
"""

import glob
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed.  Run: pip install pyserial rich")
    raise

try:
    from rich.console import Console
except ImportError:
    print("ERROR: rich not installed.  Run: pip install pyserial rich")
    raise

console = Console()

# ── Serial port auto-detection ────────────────────────────────────────────────

BPRL_VID = 0x0483
BPRL_PID = 0x5740


def find_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        if p.vid == BPRL_VID and p.pid == BPRL_PID:
            return p.device
    candidates = glob.glob("/dev/ttyACM*")
    return candidates[0] if candidates else None


def open_port(port: Optional[str], baud: int) -> serial.Serial:
    if port is None:
        port = find_port()
    if port is None:
        console.print("[red]ERROR: No BPRL device found. Use --port to specify.")
        raise SystemExit(1)
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        console.print(f"[green]Connected: {port}")
        time.sleep(0.15)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        console.print(f"[red]ERROR: Cannot open {port}: {e}")
        raise SystemExit(1)


def add_port_args(parser):
    """Add --port and --baud arguments to an argparse parser."""
    parser.add_argument("--port", default=None, help="Serial port (auto-detected)")
    parser.add_argument("--baud", type=int, default=115200)


def send_cmd(ser: serial.Serial, cmd: str):
    ser.write((cmd + "\n").encode())
    ser.flush()


# ── Background serial reader ──────────────────────────────────────────────────

class SerialReader:
    """Reads lines from serial in a background thread."""
    def __init__(self, ser: serial.Serial):
        self._ser    = ser
        self._lock   = threading.Lock()
        self._lines  = []
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if text:
                    with self._lock:
                        self._lines.append(text)

    def pop_lines(self) -> list:
        with self._lock:
            lines, self._lines = self._lines, []
        return lines

    def stop(self):
        self._stop.set()


# ── Line readers (blocking, for command-response flows) ───────────────────────

def _read_line(ser: serial.Serial, timeout: float = 5.0) -> Optional[str]:
    line, _ = _read_line_ex(ser, timeout)
    return line


def _read_line_ex(ser: serial.Serial, timeout: float = 5.0):
    """Read one line; return (line, remainder_bytes)."""
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                idx  = buf.index(b"\n")
                line = buf[:idx].decode("ascii", errors="replace").strip()
                return line, buf[idx + 1:]
        time.sleep(0.01)
    return None, b""


# ── Shared telemetry state ────────────────────────────────────────────────────

@dataclass
class TelState:
    # $TEL,time_ms,roll°,pitch°,yaw°,p,q,r,thr,rc_roll,rc_pitch,rc_yaw,armed,
    #      imu0_v,imu1_v,imu2_v,can_v,can_quat_hz,can_rate_hz,
    #      leg_L,leg_L_dot,leg_pitch°,leg_pitch_dot,wheel_vel_L,wheel_vel_R,
    #      body_u,body_w
    time_ms:      float = 0.0
    roll:         float = 0.0
    pitch:        float = 0.0
    yaw:          float = 0.0
    p:            float = 0.0
    q:            float = 0.0
    r:            float = 0.0
    thr:          float = 0.0
    rc_roll:      float = 0.0
    rc_pitch:     float = 0.0
    rc_yaw:       float = 0.0
    armed:        bool  = False
    imu_valid:    list  = field(default_factory=lambda: [False, False, False])
    can_valid:    bool  = False
    can_quat_hz:  int   = 0
    can_rate_hz:  int   = 0
    leg_L:         float = 0.0   # m, averaged virtual leg length (Kalman/lowpass state)
    leg_L_dot:     float = 0.0   # m/s
    leg_pitch:     float = 0.0   # deg, averaged leg pitch (theta = phi - thL)
    leg_pitch_dot: float = 0.0   # rad/s
    wheel_vel:     list  = field(default_factory=lambda: [0.0, 0.0])  # rad/s, L/R
    body_u:        float = 0.0   # m/s, body-frame forward velocity
    body_w:        float = 0.0   # m/s, body-frame vertical velocity
    received_any: bool  = False
    usb_rx_any:   bool  = False
    last_rx:      float = field(default_factory=time.monotonic)
    lines_rx:     int   = 0


def parse_tel_line(line: str, state: TelState) -> bool:
    """Parse a $TEL CSV line into state. Returns True on success."""
    if not line.startswith("$TEL,"):
        return False
    try:
        parts = line[5:].split(",")
        if len(parts) < 26:
            return False
        state.time_ms     = float(parts[0])
        state.roll        = float(parts[1])
        state.pitch       = float(parts[2])
        state.yaw         = float(parts[3])
        state.p           = float(parts[4])
        state.q           = float(parts[5])
        state.r           = float(parts[6])
        state.thr         = float(parts[7])
        state.rc_roll     = float(parts[8])
        state.rc_pitch    = float(parts[9])
        state.rc_yaw      = float(parts[10])
        state.armed       = bool(int(parts[11]))
        state.imu_valid   = [bool(int(parts[12])), bool(int(parts[13])),
                             bool(int(parts[14]))]
        state.can_valid   = bool(int(parts[15]))
        state.can_quat_hz = int(parts[16])
        state.can_rate_hz = int(parts[17])
        state.leg_L         = float(parts[18])
        state.leg_L_dot     = float(parts[19])
        state.leg_pitch     = float(parts[20])
        state.leg_pitch_dot = float(parts[21])
        state.wheel_vel     = [float(parts[22]), float(parts[23])]
        state.body_u        = float(parts[24])
        state.body_w        = float(parts[25])
        state.received_any = True
        state.usb_rx_any   = True
        state.last_rx      = time.monotonic()
        return True
    except (ValueError, IndexError):
        return False


# ── IMU sample ────────────────────────────────────────────────────────────────

@dataclass
class ImuSample:
    time_ms: int
    accel:   list  # [3][3] body-frame m/s²
    gyro:    list  # [3][3] body-frame rad/s
    valid:   list  # [3] bool


def parse_imu_line(line: str) -> Optional[ImuSample]:
    """Parse a $IMU CSV line. Returns None on malformed input."""
    if not line.startswith("$IMU,"):
        return None
    try:
        parts = line[5:].split(",")
        if len(parts) < 22:
            return None
        ms = int(parts[0])
        accel, gyro, valid = [], [], []
        for i in range(3):
            base = 1 + i * 7
            accel.append([float(parts[base]),
                          float(parts[base + 1]),
                          float(parts[base + 2])])
            gyro.append([float(parts[base + 3]),
                         float(parts[base + 4]),
                         float(parts[base + 5])])
            vf = parts[base + 6]
            valid.append(bool(int(vf.split("(")[0])))
        return ImuSample(time_ms=ms, accel=accel, gyro=gyro, valid=valid)
    except (ValueError, IndexError):
        return None


# ── EKF per-lane state ────────────────────────────────────────────────────────

@dataclass
class EkfLaneState:
    time_ms:      float = 0.0
    primary:      int   = 0
    roll:         list  = field(default_factory=lambda: [0.0] * 4)
    pitch:        list  = field(default_factory=lambda: [0.0] * 4)
    yaw:          list  = field(default_factory=lambda: [0.0] * 4)
    p:            list  = field(default_factory=lambda: [0.0] * 4)
    q:            list  = field(default_factory=lambda: [0.0] * 4)
    r:            list  = field(default_factory=lambda: [0.0] * 4)
    imu_valid:    list  = field(default_factory=lambda: [False, False, False])
    can_valid:    bool  = False
    received_any: bool  = False
    usb_rx_any:   bool  = False
    last_rx:      float = field(default_factory=time.monotonic)
    lines_rx:     int   = 0


def parse_ekfl_line(line: str, state: EkfLaneState) -> bool:
    """Parse a $EKFL line into state. Returns True on success."""
    if not line.startswith("$EKFL,"):
        return False
    try:
        parts = line[6:].split(",")
        if len(parts) < 30:
            return False
        state.time_ms = float(parts[0])
        state.primary = int(parts[1])
        for i in range(4):
            base = 2 + i * 6
            state.roll[i]  = float(parts[base])
            state.pitch[i] = float(parts[base + 1])
            state.yaw[i]   = float(parts[base + 2])
            state.p[i]     = float(parts[base + 3])
            state.q[i]     = float(parts[base + 4])
            state.r[i]     = float(parts[base + 5])
        state.imu_valid = [bool(int(parts[26])), bool(int(parts[27])),
                           bool(int(parts[28]))]
        state.can_valid = bool(int(parts[29]))
        state.received_any = True
        state.usb_rx_any   = True
        state.last_rx      = time.monotonic()
        return True
    except (ValueError, IndexError):
        return False
