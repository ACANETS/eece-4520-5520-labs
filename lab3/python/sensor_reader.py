"""
sensor_reader.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Shared utility module provided to students.

Contains:
  - ParkingFrame   : dataclass representing one decoded sensor frame
  - FrameParser    : state-machine byte parser for the 7-byte binary protocol
  - SensorReader   : manages serial connection, handshake, and frame parsing
  - SensorSimulator: generates synthetic parking data for offline testing
  - make_reader()  : factory that returns SensorReader or SensorSimulator

Usage (hardware):
    reader = SensorReader(port='/dev/tty.usbmodem1101')
    with reader:
        frame = reader.read()   # returns ParkingFrame or None

Usage (simulator):
    reader = SensorSimulator()
    with reader:
        frame = reader.read()

Usage (factory, recommended in step scripts):
    reader = make_reader(port=args.port, simulate=args.simulate)
    with reader:
        frame = reader.read()
"""

import argparse
import glob
import math
import random
import time
from dataclasses import dataclass
from typing import Dict, Optional, Union

import serial
from serial import SerialException

# ─── Constants ────────────────────────────────────────────────────────────────
BAUD_RATE: int       = 115200
CONNECT_TIMEOUT: float = 5.0    # seconds to wait for handshake response
READ_TIMEOUT: float  = 0.1      # serial read timeout (seconds)

# Binary frame parameters
FRAME_START: int = 0xAA         # start byte marker
FRAME_LEN: int   = 7            # total frame length in bytes

# ─── Named alarm/zone states ──────────────────────────────────────────────────
ALARM_NAMES: Dict[int, str] = {
    0: "Silent",
    1: "Beeping",
    2: "Continuous",
}

ZONE_NAMES: Dict[int, str] = {
    0: "Full Speed",
    1: "3/4 Speed",
    2: "Half Speed",
    3: "Stop",
}


# =============================================================================
# ParkingFrame — one decoded sensor reading
# =============================================================================

@dataclass
class ParkingFrame:
    """
    Represents one decoded parking sensor reading.

    Attributes:
        distance_cm: Measured distance in centimetres (0.0–100.0+ cm).
        motor_pwm:   Motor PWM duty cycle (0–255).
        alarm_state: Buzzer alarm state (0=Silent, 1=Beeping, 2=Continuous).
        zone_id:     Distance zone (0=Full Speed, 1=3/4, 2=Half, 3=Stop).
        timestamp:   Unix timestamp when the frame was received/created.
    """
    distance_cm: float
    motor_pwm: int
    alarm_state: int
    zone_id: int
    timestamp: float


# =============================================================================
# FrameParser — state-machine binary protocol parser
# =============================================================================

class FrameParser:
    """
    State-machine parser for the 7-byte parking sensor binary protocol.

    Frame layout:
        Byte 0:   0xAA  (start byte)
        Bytes 1-2: distance × 10, uint16 big-endian
        Byte 3:   motor_pwm (0–255)
        Byte 4:   alarm_state (0/1/2)
        Byte 5:   zone_id (0–3)
        Byte 6:   checksum = XOR of bytes 1–5

    The parser accumulates incoming bytes one at a time.  When it sees 0xAA it
    starts buffering.  Once 7 bytes are collected it validates the checksum.
    If valid → return ParkingFrame; if not → discard and resync.

    Example:
        parser = FrameParser()
        for byte in stream:
            frame = parser.feed(byte)
            if frame is not None:
                process(frame)
    """

    def __init__(self) -> None:
        self._buf: bytearray = bytearray()
        self._synced: bool = False
        self._frames_ok: int = 0
        self._frames_bad: int = 0
        self._bytes_processed: int = 0

    # ------------------------------------------------------------------
    def feed(self, b: int) -> Optional[ParkingFrame]:
        """
        Feed one byte into the parser.

        Args:
            b: An integer byte value (0–255).

        Returns:
            A ParkingFrame if a complete, valid frame was just completed,
            or None if still accumulating / frame was invalid.
        """
        self._bytes_processed += 1

        # Resync: if we're not in a frame and we see the start byte, begin
        if not self._synced:
            if b == FRAME_START:
                self._buf = bytearray([b])
                self._synced = True
            return None

        # Accumulate bytes
        self._buf.append(b)

        if len(self._buf) < FRAME_LEN:
            return None  # not enough bytes yet

        # We have a full candidate frame — validate checksum
        frame = self._buf[:]
        self._buf = bytearray()
        self._synced = False

        # Checksum: XOR of bytes 1–5 must equal byte 6
        computed = 0
        for byte in frame[1:6]:
            computed ^= byte

        if computed != frame[6]:
            self._frames_bad += 1
            return None

        # Decode
        self._frames_ok += 1
        distance_cm = ((frame[1] << 8) | frame[2]) / 10.0
        return ParkingFrame(
            distance_cm=distance_cm,
            motor_pwm=frame[3],
            alarm_state=frame[4],
            zone_id=frame[5],
            timestamp=time.time(),
        )

    # ------------------------------------------------------------------
    @property
    def stats(self) -> Dict[str, int]:
        """
        Return parser statistics.

        Returns:
            dict with keys: frames_ok, frames_bad, bytes_processed
        """
        return {
            "frames_ok": self._frames_ok,
            "frames_bad": self._frames_bad,
            "bytes_processed": self._bytes_processed,
        }


# =============================================================================
# SensorReader — serial connection + handshake + frame parsing
# =============================================================================

class SensorReader:
    """
    Manages the serial connection to the Arduino running parking_sensor.ino.

    Performs the handshake (sends 's', waits for 'r'), then provides a simple
    read() interface that returns parsed ParkingFrame objects.

    Args:
        port (str): Serial port path, e.g. '/dev/tty.usbmodem1101' or 'COM3'.
        baud (int): Baud rate. Must match the Arduino sketch (default 115200).
    """

    def __init__(self, port: str, baud: int = BAUD_RATE) -> None:
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._parser: FrameParser = FrameParser()

    # ------------------------------------------------------------------
    def connect(self) -> None:
        """
        Open the serial port and perform the parking sensor handshake.

        Sends 's' to the Arduino; waits up to CONNECT_TIMEOUT seconds for
        the 'r' response before raising TimeoutError.

        Raises:
            SerialException: If the port cannot be opened.
            TimeoutError:    If 'r' is not received in time.
        """
        try:
            self._serial = serial.Serial(
                self.port,
                self.baud,
                timeout=READ_TIMEOUT,
            )
        except SerialException as exc:
            raise SerialException(
                f"Cannot open port '{self.port}'. "
                "Check that the Arduino is connected and the port is correct.\n"
                "On macOS/Linux, run:  ls /dev/tty.*  to list available ports.\n"
                f"Original error: {exc}"
            ) from exc

        # Arduino Mega resets when the serial port opens (DTR toggle).
        # Wait ~2.2 s for the bootloader + sketch setup() to finish.
        print(
            f"[SensorReader] Port {self.port} opened — waiting for Arduino boot … ",
            end="",
            flush=True,
        )
        time.sleep(2.2)
        self._serial.reset_input_buffer()
        print("ready.")

        # Send 's' repeatedly until we receive 'r' or we time out.
        deadline = time.time() + CONNECT_TIMEOUT
        last_send = 0.0
        while time.time() < deadline:
            now = time.time()
            if now - last_send >= 0.5:
                self._serial.write(b's')
                self._serial.flush()
                last_send = now

            raw = self._serial.read(1)
            if raw == b'r':
                print(
                    f"[SensorReader] Connected on {self.port} at {self.baud} baud — handshake OK."
                )
                # Drain any trailing bytes (newline after 'r')
                time.sleep(0.05)
                self._serial.reset_input_buffer()
                return

        raise TimeoutError(
            f"Did not receive 'r' from Arduino within {CONNECT_TIMEOUT}s.\n"
            "Common causes:\n"
            "  1. Wrong sketch — make sure parking_sensor.ino is flashed.\n"
            "  2. Wrong baud rate — sketch and Python must both use 115200.\n"
            "  3. Wrong port — run  ls /dev/tty.*  (macOS) or  ls /dev/ttyACM*  (Linux).\n"
            "  4. Port in use — close Arduino Serial Monitor if it is open."
        )

    # ------------------------------------------------------------------
    def read(self) -> Optional[ParkingFrame]:
        """
        Read bytes from the serial port and return a ParkingFrame when complete.

        Reads one byte at a time and feeds it into the FrameParser.  Returns
        immediately with None if no byte is available or the frame is not yet
        complete.

        Returns:
            A ParkingFrame if a valid frame just completed, None otherwise.
        """
        if self._serial is None:
            raise RuntimeError("Call connect() before read().")

        raw = self._serial.read(1)
        if not raw:
            return None
        return self._parser.feed(raw[0])

    # ------------------------------------------------------------------
    @property
    def stats(self) -> Dict[str, int]:
        """Delegated to FrameParser.stats."""
        return self._parser.stats

    # ------------------------------------------------------------------
    def close(self) -> None:
        """Close the serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("[SensorReader] Serial port closed.")

    # ------------------------------------------------------------------
    def __enter__(self) -> "SensorReader":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()


# =============================================================================
# SensorSimulator — synthetic parking data for offline testing
# =============================================================================

class SensorSimulator:
    """
    Synthetic parking sensor data generator for offline testing.

    Simulates a car approaching and receding from a wall using a sinusoidal
    cycle (60-second period, distance range 5–120 cm).  All derived fields
    (motor_pwm, alarm_state, zone_id) are computed using the same formulas
    as the Arduino firmware, so the simulator faithfully represents what the
    real system would produce.

    Occasional "bad frame" scenarios are simulated via FrameParser injection
    to exercise checksum error handling in downstream code.

    The interface mirrors SensorReader exactly — all step scripts work
    unchanged when --simulate is passed.
    """

    # Simulation parameters
    _PERIOD_S: float  = 60.0    # approach/recede cycle period (seconds)
    _DIST_MIN: float  = 5.0     # cm — closest simulated distance
    _DIST_MAX: float  = 120.0   # cm — farthest simulated distance
    _SAMPLE_DT: float = 0.1     # seconds between readings (10 Hz)

    def __init__(self) -> None:
        self._t: float = 0.0
        self._last_real: float = 0.0
        self._parser: FrameParser = FrameParser()

    # ------------------------------------------------------------------
    def connect(self) -> None:
        """No-op for simulator; mirrors SensorReader.connect()."""
        self._last_real = time.time()
        self._t = 0.0
        print("[SensorSimulator] Simulator mode active — no hardware required.")

    # ------------------------------------------------------------------
    def read(self) -> Optional[ParkingFrame]:
        """
        Return one synthetic ParkingFrame, throttled to 10 Hz.

        Occasionally injects a simulated "bad frame" (checksum failure) so
        that FrameParser statistics show realistic error counts.

        Returns:
            A ParkingFrame, or None if not enough time has elapsed.
        """
        now = time.time()
        if now - self._last_real < self._SAMPLE_DT:
            return None
        self._last_real = now
        self._t += self._SAMPLE_DT

        # Sinusoidal distance: oscillates between _DIST_MIN and _DIST_MAX
        dist = self._DIST_MIN + (self._DIST_MAX - self._DIST_MIN) * (
            0.5 + 0.5 * math.sin(2.0 * math.pi * self._t / self._PERIOD_S)
        )

        # Derive fields using same formulas as Arduino
        zone_id    = self._compute_zone(dist)
        motor_pwm  = self._compute_pwm(zone_id)
        alarm_state = self._compute_alarm(dist)

        # Occasionally simulate a bad frame to exercise FrameParser error path
        if random.random() < 0.02:   # ~2% bad frame rate
            # Feed a corrupt frame through the parser (checksum wrong)
            bad = bytes([FRAME_START, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00])
            for b in bad:
                self._parser.feed(b)
            return None

        return ParkingFrame(
            distance_cm=round(dist, 1),
            motor_pwm=motor_pwm,
            alarm_state=alarm_state,
            zone_id=zone_id,
            timestamp=time.time(),
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _compute_zone(dist: float) -> int:
        """Mirror of Arduino getZone() — must stay in sync."""
        if dist > 75: return 0
        if dist > 50: return 1
        if dist > 25: return 2
        return 3

    @staticmethod
    def _compute_pwm(zone: int) -> int:
        """Mirror of Arduino getPWM() — must stay in sync."""
        return {0: 255, 1: 191, 2: 128, 3: 0}.get(zone, 0)

    @staticmethod
    def _compute_alarm(dist: float) -> int:
        """Mirror of Arduino getAlarmState() — must stay in sync."""
        if dist >= 60: return 0
        if dist >= 20: return 1
        return 2

    # ------------------------------------------------------------------
    @property
    def stats(self) -> Dict[str, int]:
        """Return parser statistics (tracks simulated bad frames)."""
        return self._parser.stats

    # ------------------------------------------------------------------
    def close(self) -> None:
        """No-op for simulator; mirrors SensorReader.close()."""
        print("[SensorSimulator] Simulator stopped.")

    # ------------------------------------------------------------------
    def __enter__(self) -> "SensorSimulator":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()


# =============================================================================
# Factory helpers
# =============================================================================

def auto_detect_port() -> Optional[str]:
    """
    Auto-detect the first available Arduino serial port.

    Tries common patterns on macOS, Linux, and Windows.

    Returns:
        First matching port string, or None if none found.
    """
    patterns = [
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            return sorted(matches)[0]

    # Windows COM ports
    for i in range(1, 20):
        port = f"COM{i}"
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except (SerialException, OSError):
            pass

    return None


def make_reader(
    port: Optional[str],
    simulate: bool,
) -> Union[SensorReader, SensorSimulator]:
    """
    Return a SensorReader or SensorSimulator depending on flags.

    This is the recommended way for step scripts to obtain a reader — it
    handles auto-detection and error messages in one place.

    Args:
        port:     Serial port string, or None to auto-detect.
        simulate: If True, return a SensorSimulator regardless of port.

    Returns:
        An object with connect() / read() / close() / stats interface.
    """
    if simulate:
        return SensorSimulator()

    resolved_port = port or auto_detect_port()
    if resolved_port is None:
        raise RuntimeError(
            "No serial port specified and none detected automatically.\n"
            "Connect the Arduino and retry, or pass --simulate for offline mode.\n"
            "On macOS/Linux:  ls /dev/tty.*\n"
            "On Windows:      check Device Manager for COMx"
        )
    if port is None:
        print(f"[sensor_reader] Auto-detected port: {resolved_port}")
    return SensorReader(port=resolved_port)


def add_args(parser: argparse.ArgumentParser) -> None:
    """
    Add standard --port and --simulate arguments to an argparse parser.

    Call this helper in each step script to keep argument names consistent.

    Args:
        parser: An argparse.ArgumentParser instance to add arguments to.
    """
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help=(
            "Serial port (e.g. /dev/tty.usbmodem1101 or COM3). "
            "Auto-detected if not specified."
        ),
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Use synthetic parking data instead of real hardware.",
    )
