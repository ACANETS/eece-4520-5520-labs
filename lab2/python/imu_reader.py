"""
imu_reader.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Shared utility module provided to students.

Contains:
  - IMUReader  : handles serial connection, handshake, and packet parsing
  - IMUSimulator: generates synthetic IMU data for offline testing

Usage (hardware):
    reader = IMUReader(port='/dev/tty.usbmodem1101')
    reader.connect()
    pkt = reader.read_packet()   # returns dict or None

Usage (simulator):
    reader = IMUSimulator()
    reader.connect()
    pkt = reader.read_packet()
"""

import glob
import math
import random
import time
from typing import Dict, Optional

import serial
from serial import SerialException

# ─── Constants ────────────────────────────────────────────────────────────────
BAUD_RATE: int = 115200
CONNECT_TIMEOUT: float = 5.0    # seconds to wait for IMU_READY
READ_TIMEOUT: float = 0.1       # serial readline timeout (seconds)
BAD_PACKET_WARN: int = 10       # warn after this many consecutive bad packets
EXPECTED_FIELDS: int = 7        # timestamp + 6 sensor values

# Simulator parameters
SIM_SAMPLE_RATE: float = 50.0   # Hz
SIM_ACCEL_NOISE: float = 0.01   # σ of Gaussian noise on accelerometer (g)
SIM_GYRO_NOISE: float = 0.05    # σ of Gaussian noise on gyroscope (°/s)
SIM_GYRO_DRIFT: float = 0.05    # gyro bias drift rate (°/s²)
SIM_TILT_RATE: float = 1.0      # degrees per second during simulated tilt
SIM_TILT_HOLD: float = 10.0     # seconds in each tilt direction
SIM_SHAKE_INTERVAL: float = 15.0  # seconds between simulated shake events
SIM_SHAKE_DURATION: float = 1.2   # seconds each shake lasts
SIM_SHAKE_ACCEL: float = 1.8      # extra accel magnitude added during shake (g)


def auto_detect_port() -> Optional[str]:
    """
    Auto-detect the first available Arduino serial port.

    Tries common patterns on macOS (/dev/tty.usbmodem*, /dev/tty.usbserial*),
    Linux (/dev/ttyACM*, /dev/ttyUSB*), and Windows (COM*).

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


# =============================================================================
# IMUReader
# =============================================================================

class IMUReader:
    """
    Manages the serial connection to the Arduino running imu_stream.ino.

    Performs the handshake (sends 's', waits for 'IMU_READY'), then provides
    a simple read_packet() interface that returns parsed sensor data.

    Args:
        port (str): Serial port path, e.g. '/dev/tty.usbmodem1101' or 'COM3'.
        baud (int): Baud rate. Must match the Arduino sketch (default 115200).
    """

    def __init__(self, port: str, baud: int = BAUD_RATE) -> None:
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._bad_count: int = 0

    # ------------------------------------------------------------------
    def connect(self) -> None:
        """
        Open the serial port and perform the IMU handshake.

        Sends 's' to the Arduino; waits up to CONNECT_TIMEOUT seconds for
        the 'IMU_READY' response before raising TimeoutError.

        Raises:
            SerialException: If the port cannot be opened.
            TimeoutError: If 'IMU_READY' is not received in time.
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

        # Arduino Mega resets automatically when the serial port is opened
        # (the DTR line toggles, triggering the auto-reset circuit).
        # The bootloader + sketch startup takes ~2 seconds — wait it out
        # before sending the handshake byte, otherwise the Arduino misses 's'.
        print(f"[IMUReader] Port {self.port} opened — waiting for Arduino boot … ", end="", flush=True)
        time.sleep(2.2)
        self._serial.reset_input_buffer()
        print("ready.")

        # Retry sending 's' every 0.5 s until we receive 'IMU_READY' or time out.
        # Retrying handles the rare case where the first byte arrives while the
        # sketch's Serial.begin() is still initialising.
        deadline  = time.time() + CONNECT_TIMEOUT
        last_send = 0.0
        while time.time() < deadline:
            now = time.time()
            if now - last_send >= 0.5:
                self._serial.write(b's')
                self._serial.flush()
                last_send = now
            line = self._serial.readline().decode("ascii", errors="ignore").strip()
            if line == "IMU_READY":
                print(f"[IMUReader] Connected on {self.port} at {self.baud} baud — IMU_READY received.")
                return

        raise TimeoutError(
            f"Did not receive 'IMU_READY' from Arduino within {CONNECT_TIMEOUT}s.\n"
            "Common causes:\n"
            "  1. Wrong sketch — make sure arduino/imu_stream/imu_stream.ino is flashed.\n"
            "  2. Wrong baud rate — sketch and Python must both use 115200.\n"
            "  3. Wrong port — run  ls /dev/tty.*  (macOS) or  ls /dev/ttyACM*  (Linux).\n"
            "  4. Port in use — close Arduino Serial Monitor if it is open.\n"
            "  5. Wiring — check SDA→pin20, SCL→pin21, VCC, GND on the MPU-6050."
        )

    # ------------------------------------------------------------------
    def read_packet(self) -> Optional[Dict[str, float]]:
        """
        Read and parse one CSV line from the Arduino.

        Expected format:
            timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps

        Returns:
            dict with keys {ts, ax, ay, az, gx, gy, gz}, or None if the
            line is malformed or no data is available yet.
        """
        if self._serial is None:
            raise RuntimeError("Call connect() before read_packet().")

        raw = self._serial.readline()
        if not raw:
            return None

        line = raw.decode("ascii", errors="ignore").strip()
        parts = line.split(",")

        if len(parts) != EXPECTED_FIELDS:
            self._bad_count += 1
            self._maybe_warn()
            return None

        try:
            values = [float(p) for p in parts]
        except ValueError:
            self._bad_count += 1
            self._maybe_warn()
            return None

        self._bad_count = 0   # reset on success
        return {
            "ts": values[0] / 1000.0,   # ms → seconds
            "ax": values[1],
            "ay": values[2],
            "az": values[3],
            "gx": values[4],
            "gy": values[5],
            "gz": values[6],
        }

    # ------------------------------------------------------------------
    def close(self) -> None:
        """Close the serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("[IMUReader] Serial port closed.")

    # ------------------------------------------------------------------
    def _maybe_warn(self) -> None:
        """Print a warning after BAD_PACKET_WARN consecutive bad packets."""
        if self._bad_count == BAD_PACKET_WARN:
            print(
                f"[IMUReader] WARNING: {BAD_PACKET_WARN} consecutive malformed packets. "
                "Possible baud-rate mismatch or wrong firmware."
            )

    # ------------------------------------------------------------------
    def __enter__(self) -> "IMUReader":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()


# =============================================================================
# IMUSimulator
# =============================================================================

class IMUSimulator:
    """
    Synthetic IMU data generator for offline testing.

    Simulates:
      - A board lying flat (az ≈ 1.0g), with small Gaussian noise
      - A slow sinusoidal tilt: roll sweeps ±(SIM_TILT_RATE × SIM_TILT_HOLD) degrees
      - Gyro readings derived from the derivative of simulated angles, with
        additive noise and a slowly accumulating bias (simulating real gyro drift)

    The interface mirrors IMUReader exactly, so all step scripts work without
    any changes when --simulate is passed.

    Args:
        sample_rate (float): Simulated output rate in Hz (default 50).
    """

    def __init__(self, sample_rate: float = SIM_SAMPLE_RATE) -> None:
        self._dt: float = 1.0 / sample_rate
        self._t: float = 0.0                # simulation time (seconds)
        self._gyro_bias_x: float = 0.0      # accumulating bias on GX
        self._gyro_bias_y: float = 0.0
        self._last_real: float = 0.0        # real-clock last send time
        self._next_shake: float = SIM_SHAKE_INTERVAL  # time of next shake event

    # ------------------------------------------------------------------
    def connect(self) -> None:
        """No-op for simulator; mimics IMUReader.connect()."""
        self._last_real = time.time()
        print("[IMUSimulator] Simulator mode active — no hardware required.")

    # ------------------------------------------------------------------
    def read_packet(self) -> Optional[Dict[str, float]]:
        """
        Return one synthetic IMU packet, throttled to the simulated sample rate.

        Returns:
            dict with keys {ts, ax, ay, az, gx, gy, gz}, or None if not enough
            time has elapsed since the last packet (mimics real serial timing).
        """
        now = time.time()
        if now - self._last_real < self._dt:
            return None
        self._last_real = now

        self._t += self._dt

        # Simulated true roll: slow sinusoidal oscillation
        # Sweeps from 0° to +30°, back to 0°, to -30°, etc.
        true_roll_deg = 30.0 * math.sin(2.0 * math.pi * self._t / (2.0 * SIM_TILT_HOLD))
        true_pitch_deg = 10.0 * math.sin(2.0 * math.pi * self._t / (4.0 * SIM_TILT_HOLD))

        true_roll_rad = math.radians(true_roll_deg)
        true_pitch_rad = math.radians(true_pitch_deg)

        # Accelerometer: gravity vector rotated by roll/pitch
        ax = -math.sin(true_pitch_rad)
        ay = math.sin(true_roll_rad) * math.cos(true_pitch_rad)
        az = math.cos(true_roll_rad) * math.cos(true_pitch_rad)

        # Add Gaussian noise to accel
        ax += random.gauss(0.0, SIM_ACCEL_NOISE)
        ay += random.gauss(0.0, SIM_ACCEL_NOISE)
        az += random.gauss(0.0, SIM_ACCEL_NOISE)

        # Simulated shake: every SIM_SHAKE_INTERVAL seconds, inject SIM_SHAKE_DURATION
        # seconds of high-magnitude random acceleration (mimics vigorous shaking).
        if self._t >= self._next_shake - SIM_SHAKE_DURATION and self._t < self._next_shake:
            ax += random.gauss(0.0, SIM_SHAKE_ACCEL)
            ay += random.gauss(0.0, SIM_SHAKE_ACCEL)
            az += random.gauss(0.0, SIM_SHAKE_ACCEL)
        elif self._t >= self._next_shake:
            self._next_shake += SIM_SHAKE_INTERVAL

        # Simulated true yaw: slow sinusoidal sweep ±60° over 3× the tilt period
        true_yaw_deg = 60.0 * math.sin(2.0 * math.pi * self._t / (3.0 * SIM_TILT_HOLD))

        # True angular velocity (derivatives of simulated angles)
        omega_roll  = 2.0 * math.pi / (2.0 * SIM_TILT_HOLD)
        omega_pitch = 2.0 * math.pi / (4.0 * SIM_TILT_HOLD)
        omega_yaw   = 2.0 * math.pi / (3.0 * SIM_TILT_HOLD)

        true_gx = math.degrees(30.0 * omega_roll  * math.cos(2.0 * math.pi * self._t / (2.0 * SIM_TILT_HOLD)))
        true_gy = math.degrees(10.0 * omega_pitch * math.cos(2.0 * math.pi * self._t / (4.0 * SIM_TILT_HOLD)))
        true_gz = math.degrees(60.0 * omega_yaw   * math.cos(2.0 * math.pi * self._t / (3.0 * SIM_TILT_HOLD)))

        # Accumulate gyro bias drift
        self._gyro_bias_x += random.gauss(0.0, SIM_GYRO_DRIFT * self._dt)
        self._gyro_bias_y += random.gauss(0.0, SIM_GYRO_DRIFT * self._dt)

        gx = true_gx + self._gyro_bias_x + random.gauss(0.0, SIM_GYRO_NOISE)
        gy = true_gy + self._gyro_bias_y + random.gauss(0.0, SIM_GYRO_NOISE)
        gz = true_gz               + random.gauss(0.0, SIM_GYRO_NOISE)

        return {
            "ts": self._t,
            "ax": ax,
            "ay": ay,
            "az": az,
            "gx": gx,
            "gy": gy,
            "gz": gz,
        }

    # ------------------------------------------------------------------
    def close(self) -> None:
        """No-op for simulator; mimics IMUReader.close()."""
        print("[IMUSimulator] Simulator stopped.")

    # ------------------------------------------------------------------
    def __enter__(self) -> "IMUSimulator":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()


# =============================================================================
# Factory helper
# =============================================================================

def make_reader(port: Optional[str], simulate: bool) -> "IMUReader | IMUSimulator":
    """
    Return an IMUReader or IMUSimulator depending on flags.

    Args:
        port:     Serial port string, or None to auto-detect.
        simulate: If True, return an IMUSimulator regardless of port.

    Returns:
        An object with connect() / read_packet() / close() interface.
    """
    if simulate:
        return IMUSimulator()

    resolved_port = port or auto_detect_port()
    if resolved_port is None:
        raise RuntimeError(
            "No serial port specified and none detected automatically.\n"
            "Connect the Arduino and retry, or pass --simulate for offline mode.\n"
            "On macOS/Linux:  ls /dev/tty.*\n"
            "On Windows:      check Device Manager for COMx"
        )
    if port is None:
        print(f"[imu_reader] Auto-detected port: {resolved_port}")
    return IMUReader(port=resolved_port)
