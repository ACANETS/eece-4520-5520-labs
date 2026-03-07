"""
step3_accel_angles.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 3: Roll and Pitch from the Accelerometer
───────────────────────────────────────────────
Uses only accelerometer data to estimate the board's roll and pitch angles.
Displays them as a real-time plot and prints numeric values to the terminal.

Background — Why does this work?
─────────────────────────────────
When the board is stationary, the accelerometer measures Earth's gravity vector
(approximately 1 g downward). By measuring how that gravity vector projects onto
each body axis, we can infer the orientation of the board:

    Roll  (rotation around X-axis):
        roll = atan2(ay, az)
        — If the board rolls right (Y-axis goes up), az decreases and ay increases.

    Pitch (rotation around Y-axis):
        pitch = atan2(-ax, sqrt(ay² + az²))
        — Robust to roll; uses the magnitude of the lateral plane as denominator.

Limitations:
  • Only works when the board is stationary (gravity must dominate the signal).
  • During linear acceleration or vibration, the measurement becomes noisy.
  • Cannot estimate yaw (rotation around vertical axis) — gravity is symmetric.

This motivates combining accelerometer with gyroscope data (Step 5).

Usage:
    python step3_accel_angles.py                    # auto-detect port
    python step3_accel_angles.py --port /dev/ttyUSB0
    python step3_accel_angles.py --simulate
"""

import argparse
import collections
import math
import threading
import time
from typing import Deque, Dict, Optional

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from imu_reader import make_reader

# Auto-select the best available backend (MacOSX native on Mac — no Tk required)
import sys as _sys
for _backend in (["MacOSX"] if _sys.platform == "darwin" else []) + ["Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass

# ─── Constants ────────────────────────────────────────────────────────────────
BUFFER_SIZE: int = 200           # ~4 seconds of data at 50 Hz
ANGLE_YLIM: tuple = (-90.0, 90.0)   # degrees
ANIMATION_INTERVAL_MS: int = 40
PRINT_EVERY_N: int = 10          # print numeric values every N packets


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 3 — Roll and pitch from accelerometer only."
    )
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port. Auto-detected if not specified.")
    parser.add_argument("--simulate", action="store_true",
                        help="Use synthetic IMU data instead of real hardware.")
    return parser.parse_args()


# =============================================================================
# accel_angles
# =============================================================================

def accel_angles(ax: float, ay: float, az: float) -> tuple:
    """
    Compute roll and pitch from accelerometer readings.

    Uses the standard gravity-projection formulas. These assume the board is
    stationary so that the measured acceleration equals gravity only.

    Args:
        ax: Acceleration in X-axis (g)
        ay: Acceleration in Y-axis (g)
        az: Acceleration in Z-axis (g)

    Returns:
        (roll_deg, pitch_deg) in degrees, range ≈ (-180, 180) for roll,
        (-90, 90) for pitch.
    """
    # ── TODO ──────────────────────────────────────────────────────────────
    # Compute roll and pitch in degrees from the accelerometer readings.
    #
    # When the board is stationary, the accelerometer measures Earth's gravity
    # vector.  The angle that vector makes with each body axis tells us the
    # board's orientation:
    #
    #   roll  = atan2(ay, az)
    #   pitch = atan2(-ax, sqrt(ay² + az²))
    #
    # Use math.atan2() and math.degrees() to return values in degrees.
    # ──────────────────────────────────────────────────────────────────────
    raise NotImplementedError("TODO Step 3: implement accel_angles()")


# =============================================================================
# DataBuffer
# =============================================================================

class DataBuffer:
    """Circular buffer holding computed roll/pitch angles."""

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.roll:  Deque[float] = collections.deque(maxlen=maxlen)
        self.pitch: Deque[float] = collections.deque(maxlen=maxlen)
        self.ts:    Deque[float] = collections.deque(maxlen=maxlen)

    def push(self, ts: float, roll: float, pitch: float) -> None:
        self.ts.append(ts)
        self.roll.append(roll)
        self.pitch.append(pitch)


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(reader, buf: DataBuffer, stop_event: threading.Event) -> None:
    """Background thread: reads IMU packets, computes angles, pushes to buf."""
    count = 0
    while not stop_event.is_set():
        pkt = reader.read_packet()
        if pkt is None:
            time.sleep(0.001)
            continue

        roll, pitch = accel_angles(pkt["ax"], pkt["ay"], pkt["az"])
        buf.push(pkt["ts"], roll, pitch)

        # Print to terminal periodically
        count += 1
        if count % PRINT_EVERY_N == 0:
            print(
                f"[{pkt['ts']:>8.3f}s]  "
                f"Roll={roll:>7.2f}°  Pitch={pitch:>7.2f}°"
            )


# =============================================================================
# setup_figure
# =============================================================================

def setup_figure():
    """Create the real-time roll/pitch plot."""
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.suptitle(
        "Step 3 — Accelerometer-Based Roll & Pitch",
        fontsize=13, fontweight="bold",
    )

    ax.set_ylabel("Angle (°)")
    ax.set_xlabel("Samples (newest on right)")
    ax.set_ylim(*ANGLE_YLIM)
    ax.set_xlim(0, BUFFER_SIZE)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.axhline(0, color="black", linewidth=0.5)

    line_roll,  = ax.plot([], [], color="tab:red",  label="Roll",  linewidth=1.5)
    line_pitch, = ax.plot([], [], color="tab:blue", label="Pitch", linewidth=1.5)
    ax.legend(loc="upper right", fontsize=10)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, (line_roll, line_pitch)


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 3."""
    args = parse_args()

    reader = make_reader(port=args.port, simulate=args.simulate)
    reader.connect()

    buf = DataBuffer()
    stop_event = threading.Event()

    t = threading.Thread(
        target=serial_reader_thread,
        args=(reader, buf, stop_event),
        daemon=True,
    )
    t.start()
    print("[Step 3] Accelerometer angle estimation running. Close window to stop.")

    fig, (line_roll, line_pitch) = setup_figure()

    def update(_frame):
        n = len(buf.roll)
        if n == 0:
            return line_roll, line_pitch
        xs = list(range(n))
        line_roll.set_data(xs,  list(buf.roll))
        line_pitch.set_data(xs, list(buf.pitch))
        return line_roll, line_pitch

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=ANIMATION_INTERVAL_MS,
        blit=True,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except (KeyboardInterrupt, AttributeError):
        # AttributeError: 'TimerMac' object has no attribute 'callbacks'
        # — known matplotlib MacOSX backend cleanup bug; safe to ignore.
        pass
    finally:
        try:
            ani.event_source.stop()  # type: ignore[union-attr]
        except Exception:
            pass
        stop_event.set()
        reader.close()
        print("[Step 3] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
