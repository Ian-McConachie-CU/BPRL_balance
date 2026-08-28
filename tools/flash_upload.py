#!/usr/bin/env python3
"""
tools/flash_upload.py — BPRL firmware uploader
Uploads a raw .bin firmware file to a Cube flight controller via the
PX4/ArduPilot ChibiOS bootloader protocol over USB/UART.

Usage:
    python3 tools/flash_upload.py --port /dev/ttyACM0 build/BPRL.bin
    python3 tools/flash_upload.py build/BPRL.bin       (auto-detects port)

Requirements:
    pip install pyserial

Adapted from ArduPilot Tools/scripts/uploader.py (BSD licence, PX4 team).
Stripped to: raw .bin loading, core bootloader protocol, Linux/macOS support.
"""

import sys
import argparse
import array
import binascii
import glob
import struct
import time
import serial


# ── CRC-32 table (same polynomial as AP_Math/crc.cpp) ────────────────────────
_CRCTAB = array.array('I', [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
])


def _crc32(data, state=0):
    for byte in data:
        state = _CRCTAB[(state ^ byte) & 0xff] ^ (state >> 8)
    return state


# ── Firmware image ─────────────────────────────────────────────────────────────
class FirmwareImage:
    """Wraps a raw .bin file for upload."""

    _CRC_PAD = bytearray(b'\xff\xff\xff\xff')

    def __init__(self, path):
        with open(path, 'rb') as f:
            self.data = bytearray(f.read())
        # Pad to 4-byte boundary
        while len(self.data) % 4:
            self.data += b'\xff'
        self.size = len(self.data)

    def crc(self, flash_size):
        """CRC32 of image padded to flash_size with 0xFF words."""
        state = _crc32(self.data)
        for _ in range(len(self.data), flash_size - 1, 4):
            state = _crc32(self._CRC_PAD, state)
        return state


# ── Bootloader uploader ────────────────────────────────────────────────────────
class FirmwareUploader:
    """
    Communicates with the PX4/ArduPilot ChibiOS bootloader.

    Protocol constants from the bootloader source (Bootloader.h):
      https://github.com/ArduPilot/Bootloader
    """

    # Protocol framing
    INSYNC = b'\x12'
    EOC    = b'\x20'

    # Replies
    OK              = b'\x10'
    FAILED          = b'\x11'
    INVALID         = b'\x13'
    BAD_SILICON_REV = b'\x14'

    # Commands
    GET_SYNC    = b'\x21'
    GET_DEVICE  = b'\x22'
    CHIP_ERASE  = b'\x23'
    PROG_MULTI  = b'\x27'
    GET_CRC     = b'\x29'
    REBOOT      = b'\x30'

    # GET_DEVICE info params
    INFO_BL_REV     = b'\x01'
    INFO_BOARD_ID   = b'\x02'
    INFO_BOARD_REV  = b'\x03'
    INFO_FLASH_SIZE = b'\x04'

    BL_REV_MIN  = 2
    BL_REV_MAX  = 5

    PROG_CHUNK  = 252   # bytes per PROG_MULTI packet (≤255, multiple of 4)

    # Hardcoded MAVLink reboot-to-bootloader packets (sysid=255, compid=1).
    # These are sent when the board is running flight software and we need
    # to trigger a reboot into the bootloader.
    _MAVLINK_REBOOT_ID1 = bytearray(
        b'\xfe\x21\x72\xff\x00\x4c\x00\x00\x40\x40\x00\x00\x00\x00'
        b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        b'\x00\x00\x00\x00\x00\x00\xf6\x00\x01\x00\x00\x53\x6b')
    _MAVLINK_REBOOT_ID0 = bytearray(
        b'\xfe\x21\x45\xff\x00\x4c\x00\x00\x40\x40\x00\x00\x00\x00'
        b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        b'\x00\x00\x00\x00\x00\x00\xf6\x00\x00\x00\x00\xcc\x37')

    def __init__(self, port, baud=115200):
        self.port = serial.Serial(port, baud, timeout=2.0, write_timeout=2.0)
        self.bl_rev     = 0
        self.board_id   = 0
        self.board_rev  = 0
        self.flash_size = 0

    def close(self):
        if self.port:
            self.port.close()

    def open(self):
        deadline = time.time() + 0.2
        while time.time() < deadline:
            if not self.port.isOpen():
                self.port.open()
            if self.port.isOpen():
                return

    # ── Low-level I/O ──────────────────────────────────────────────────────────
    def _send(self, data):
        self.port.write(data)

    def _recv(self, n=1):
        data = self.port.read(n)
        if len(data) < n:
            raise RuntimeError("timeout waiting for %d bytes (got %d)" % (n, len(data)))
        return data

    def _recv_int(self):
        raw = self._recv(4)
        return struct.unpack('<I', raw)[0]

    def _get_sync(self):
        self.port.flush()
        b = self._recv()
        if b != self.INSYNC:
            raise RuntimeError("expected INSYNC, got 0x%02x" % ord(b))
        b = self._recv()
        if b == self.INVALID:
            raise RuntimeError("bootloader: INVALID OPERATION")
        if b == self.FAILED:
            raise RuntimeError("bootloader: OPERATION FAILED")
        if b != self.OK:
            raise RuntimeError("expected OK, got 0x%02x" % ord(b))

    def _try_sync(self):
        try:
            self.port.flush()
            if self._recv() != self.INSYNC:
                return False
            b = self._recv()
            if b == self.BAD_SILICON_REV:
                raise RuntimeError("unsupported silicon revision")
            return b == self.OK
        except RuntimeError:
            raise
        except Exception:
            return False

    def _sync(self):
        self.port.flushInput()
        self._send(self.GET_SYNC + self.EOC)
        self._get_sync()

    def _get_info(self, param):
        self._send(self.GET_DEVICE + param + self.EOC)
        value = self._recv_int()
        self._get_sync()
        return value

    # ── Progress bar ───────────────────────────────────────────────────────────
    @staticmethod
    def _progress(label, done, total):
        pct = min(100.0, 100.0 * done / total)
        bar = '=' * int(pct / 5)
        sys.stdout.write("\r%-12s [%-20s] %5.1f%%" % (label, bar, pct))
        sys.stdout.flush()

    # ── Bootloader operations ──────────────────────────────────────────────────
    def _erase(self):
        print()
        self._send(self.CHIP_ERASE + self.EOC)
        deadline = time.time() + 20.0
        while time.time() < deadline:
            elapsed = 20.0 - (deadline - time.time())
            self._progress("Erasing", min(elapsed, 9.0), 9.0)
            if self._try_sync():
                self._progress("Erasing", 10.0, 10.0)
                return
        raise RuntimeError("timed out waiting for erase to complete")

    def _program(self, fw):
        print()
        chunks = [fw.data[i:i + self.PROG_CHUNK]
                  for i in range(0, len(fw.data), self.PROG_CHUNK)]
        for i, chunk in enumerate(chunks):
            length = len(chunk).to_bytes(1, byteorder='big')
            self._send(self.PROG_MULTI + length + chunk + self.EOC)
            self._get_sync()
            if i % 64 == 0:
                self._progress("Programming", i + 1, len(chunks))
        self._progress("Programming", 100, 100)

    def _verify(self, fw):
        print()
        self._progress("Verifying", 1, 100)
        expected = fw.crc(self.flash_size)
        self._send(self.GET_CRC + self.EOC)
        reported = self._recv_int()
        self._get_sync()
        if reported != expected:
            raise RuntimeError(
                "CRC mismatch: expected 0x%08x, got 0x%08x" % (expected, reported))
        self._progress("Verifying", 100, 100)

    def _reboot(self):
        self._send(self.REBOOT + self.EOC)
        self.port.flush()
        if self.bl_rev >= 3:
            self._get_sync()

    # ── Public API ─────────────────────────────────────────────────────────────
    def identify(self):
        """Connect to the bootloader and read board info."""
        self._sync()
        self.bl_rev = self._get_info(self.INFO_BL_REV)
        if not (self.BL_REV_MIN <= self.bl_rev <= self.BL_REV_MAX):
            raise RuntimeError("unsupported bootloader revision %d" % self.bl_rev)
        self.board_id   = self._get_info(self.INFO_BOARD_ID)
        self.board_rev  = self._get_info(self.INFO_BOARD_REV)
        self.flash_size = self._get_info(self.INFO_FLASH_SIZE)

    def send_reboot_request(self, flight_baud=57600):
        """
        Try to reboot a running board into the bootloader.
        Sends BPRL 'BOOT' command first; falls back to MAVLink/NSH for
        boards running ArduPilot or PX4.
        """
        try:
            # BPRL firmware: send our BOOT command (USB CDC — baud ignored)
            self.port.baudrate = 115200
            self.port.write(b'BOOT\n')
            self.port.flush()
            time.sleep(0.1)
        except Exception:
            pass
        try:
            # Fallback: MAVLink PREFLIGHT_REBOOT + NSH 'reboot -b'
            self.port.baudrate = flight_baud
            self.port.flush()
            self._send(self._MAVLINK_REBOOT_ID1)
            self._send(self._MAVLINK_REBOOT_ID0)
            self._send(b'\x0d\x0d\x0d')
            self._send(b'reboot -b\n')
            self._send(b'\x0d\x0d\x0d')
            self._send(b'reboot\n')
            self.port.flush()
            self.port.baudrate = 115200
        except Exception:
            try:
                self.port.baudrate = 115200
            except Exception:
                pass
        return True

    def upload(self, fw):
        """Erase, program, verify, and reboot the board."""
        if fw.size > self.flash_size:
            raise RuntimeError(
                "firmware (%d B) exceeds flash (%d B)" % (fw.size, self.flash_size))

        print("Board ID=0x%02x rev=0x%02x  flash=%d B  BL rev=%d" % (
            self.board_id, self.board_rev, self.flash_size, self.bl_rev))
        print("Firmware: %d B" % fw.size)

        self._erase()
        self._program(fw)
        self._verify(fw)
        print("\nRebooting board...")
        self._reboot()
        self.port.close()
        print("Done.")


# ── Port discovery ─────────────────────────────────────────────────────────────
def _candidate_ports(port_arg):
    """Return a list of serial ports to try."""
    if port_arg:
        patterns = port_arg.split(',')
    else:
        patterns = [
            '/dev/serial/by-id/usb-*Hex*',
            '/dev/serial/by-id/usb-*Cube*',
            '/dev/serial/by-id/usb-*ArduPilot*',
            '/dev/ttyACM*',
        ]
    ports = []
    for p in patterns:
        ports.extend(sorted(glob.glob(p)))
    return ports or [port_arg] if port_arg else []


def _find_bootloader(port, baud=115200):
    """
    Open port, attempt to talk to the bootloader.
    If the board is running flight software, send a reboot request first.
    Returns a connected FirmwareUploader or None.
    """
    for attempt in range(20):
        try:
            up = FirmwareUploader(port, baud)
            up.open()
            try:
                up.identify()
                return up
            except Exception:
                pass
            up.send_reboot_request()
            time.sleep(0.25)
            up.close()
        except Exception:
            time.sleep(0.1)
    return None


# ── CLI entry point ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Upload BPRL firmware to a Cube flight controller.")
    parser.add_argument('firmware',
                        help="Path to firmware .bin file (e.g. build/BPRL.bin)")
    parser.add_argument('--port', default=None,
                        help="Serial port (e.g. /dev/ttyACM0). "
                             "Auto-detected if omitted.")
    parser.add_argument('--baud', type=int, default=115200,
                        help="Bootloader baud rate (default 115200)")
    args = parser.parse_args()

    # Warn about ModemManager on Linux (interferes with serial ports)
    import os
    if os.path.exists('/usr/sbin/ModemManager'):
        print("WARNING: ModemManager is installed — it may interfere with upload.\n"
              "         Run: sudo systemctl stop ModemManager")

    fw = FirmwareImage(args.firmware)
    print("Loaded %s  (%d bytes)" % (args.firmware, fw.size))
    print("Waiting for board on %s ..." % (args.port or "auto-detected port"))
    print("If no response within 5 s, unplug and re-plug the USB cable.\n")

    try:
        while True:
            for port in _candidate_ports(args.port):
                up = _find_bootloader(port, args.baud)
                if up is None:
                    continue
                try:
                    up.upload(fw)
                except RuntimeError as ex:
                    sys.exit("ERROR: %s" % ex)
                sys.exit(0)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nUpload cancelled.")
        sys.exit(1)


if __name__ == '__main__':
    main()
