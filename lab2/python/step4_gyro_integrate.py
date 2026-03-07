"""
step4_gyro_integrate.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 4: Angle Estimation via Gyroscope Integration
─────────────────────────────────────────────────────
Estimates roll and pitch by numerically integrating the gyroscope's angular
velocity measurements over time, then compares them side-by-side with the
accelerometer-based angles from Step 3.

Background — Gyroscope Integration
────────────────────────────────────
The gyroscope measures angular velocity (°/s). To get an angle, we integrate:

    roll(t)  = roll(t-1)  + gx * dt
    pitch(t) = pitch(t-1) + gy * dt

where dt is the time elapsed since the previous sample.

Strengths of gyroscope integration:
  • Smooth output — unaffected by short-duration linear accelerations or vibration.
  • High bandwidth — responds instantly to fast motion.

Weaknesses:
  • Drift — any small constant bias in the gyro reading accumulates over time,
    causing the estimated angle to slowly walk away from the true angle.
  • No absolute reference — angle is relative to the initial pose at startup.

This motivates combining both sensors (Step 5: complementary filter).

Usage:
    python step4_gyro_integrate.py                    # auto-detect port
    python step4_gyro_integrate.py --port /dev/ttyUSB0
    python step4_gyro_integrate.py --simulate
"""

import argparse
import collections
import math
import threading
import time
from typing import Deque, Optional

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
BUFFER_SIZE: int = 200            # ~4 seconds at 50 Hz
ANGLE_YLIM: tuple = (-90.0, 90.0) # degrees
ANIMATION_INTERVAL_MS: int = 40
PRINT_EVERY_N: int = 10           # terminal print frequency


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 4 — Side-by-side comparison of accel angles vs. gyro integration."
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
    Compute roll and pitch from accelerometer readings (same as Step 3).

    Args:
        ax: Acceleration in X-axis (g)
        ay: Acceleration in Y-axis (g)
        az: Acceleration in Z-axis (g)

    Returns:
        (roll_deg, pitch_deg) in degrees.
    """
    roll_deg  = math.degrees(math.atan2(ay, az))
    pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))
    return roll_deg, pitch_deg


# =============================================================================
# GyroIntegrator — stateful integrator
# =============================================================================

class GyroIntegrator:
    """
    Integrates gyroscope angular rates to estimate roll and pitch over time.

    The first call to update() initialises the angle estimates from the
    accelerometer reading so both methods start at the same reference pose.

    Attributes:
        roll (float):  Current gyro-integrated roll estimate (degrees).
        pitch (float): Current gyro-integrated pitch estimate (degrees).
    """

    def __init__(self) -> None:
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self._prev_ts: Optional[float] = None
        self._initialised: bool = False

    def update(
        self,
        ts: float,
        gx: float,
        gy: float,
        init_roll: float = 0.0,
        init_pitch: float = 0.0,
    ) -> tuple:
        """
        Update the gyro-integrated angle estimate.

        On the first call, initialises roll/pitch from the accelerometer values
        so both estimates share the same starting point.

        Args:
            ts:          Current timestamp in seconds.
            gx:          Gyro X angular velocity (°/s → roll rate).
            gy:          Gyro Y angular velocity (°/s → pitch rate).
            init_roll:   Accelerometer roll for initialisation (degrees).
            init_pitch:  Accelerometer pitch for initialisation (degrees).

        Returns:
            (roll_deg, pitch_deg) as floats.
        """
        if not self._initialised:
            self.roll = init_roll
            self.pitch = init_pitch
            self._prev_ts = ts
            self._initialised = True
            return self.roll, self.pitch

        dt = ts - self._prev_ts
        self._prev_ts = ts

        if dt <= 0:
            return self.roll, self.pitch

        # ── TODO ──────────────────────────────────────────────────────────
        # Integrate angular velocity to update the angle estimates.
        #
        # The gyroscope gives angular velocity in °/s.  Multiplying by the
        # elapsed time dt (seconds) gives the change in angle:
        #
        #   angle(t) = angle(t-1) + angular_velocity * dt
        #
        # Apply this to both self.roll (using gx) and self.pitch (using gy).
        # Then return (self.roll, self.pitch).
        # ──────────────────────────────────────────────────────────────────
        raise NotImplementedError("TODO Step 4: implement gyro integration")


# =============================================================================
# DataBuffer
# =============================================================================

class DataBuffer:
    """Circular buffer holding accel angles and gyro-integrated angles."""

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.accel_roll:  Deque[float] = collections.deque(maxlen=maxlen)
        self.accel_pitch: Deque[float] = collections.deque(maxlen=maxlen)
        self.gyro_roll:   Deque[float] = collections.deque(maxlen=maxlen)
        self.gyro_pitch:  Deque[float] = collections.deque(maxlen=maxlen)

    def push(
        self,
        a_roll: float, a_pitch: float,
        g_roll: float, g_pitch: float,
    ) -> None:
        self.accel_roll.append(a_roll)
        self.accel_pitch.append(a_pitch)
        self.gyro_roll.append(g_roll)
        self.gyro_pitch.append(g_pitch)


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(
    reader,
    buf: DataBuffer,
    stop_event: threading.Event,
) -> None:
    """
    Background thread: reads packets, computes accel and gyro angles, pushes to buf.

    Args:
        reader:     IMUReader or IMUSimulator (already connected).
        buf:        Shared DataBuffer.
        stop_event: Threading event to signal shutdown.
    """
    integrator = GyroIntegrator()
    count = 0

    while not stop_event.is_set():
        pkt = reader.read_packet()
        if pkt is None:
            time.sleep(0.001)
            continue

        a_roll, a_pitch = accel_angles(pkt["ax"], pkt["ay"], pkt["az"])
        g_roll, g_pitch = integrator.update(
            pkt["ts"], pkt["gx"], pkt["gy"],
            init_roll=a_roll, init_pitch=a_pitch,
        )
        buf.push(a_roll, a_pitch, g_roll, g_pitch)

        count += 1
        if count % PRINT_EVERY_N == 0:
            print(
                f"[{pkt['ts']:>8.3f}s]  "
                f"Accel → Roll={a_roll:>7.2f}°  Pitch={a_pitch:>7.2f}°   "
                f"Gyro  → Roll={g_roll:>7.2f}°  Pitch={g_pitch:>7.2f}°"
            )


# =============================================================================
# setup_figure
# =============================================================================

def setup_figure():
    """
    Create a two-panel figure comparing accelerometer and gyro-integrated angles.

    Returns:
        fig and the four line artists (accel_roll, accel_pitch, gyro_roll, gyro_pitch).
    """
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(14, 5), sharey=True)
    fig.suptitle(
        "Step 4 — Accelerometer Angles vs. Gyroscope Integration",
        fontsize=13, fontweight="bold",
    )

    for ax, title in [(ax_left, "Accelerometer Angles"), (ax_right, "Gyro-Integrated Angles")]:
        ax.set_title(title)
        ax.set_xlabel("Samples (newest on right)")
        ax.set_ylim(*ANGLE_YLIM)
        ax.set_xlim(0, BUFFER_SIZE)
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.axhline(0, color="black", linewidth=0.5)

    ax_left.set_ylabel("Angle (°)")

    line_ar, = ax_left.plot([], [], color="tab:red",   label="Roll",  linewidth=1.5)
    line_ap, = ax_left.plot([], [], color="tab:blue",  label="Pitch", linewidth=1.5)
    ax_left.legend(loc="upper right", fontsize=9)

    line_gr, = ax_right.plot([], [], color="tab:orange", label="Roll (gyro)",  linewidth=1.5)
    line_gp, = ax_right.plot([], [], color="tab:cyan",   label="Pitch (gyro)", linewidth=1.5)
    ax_right.legend(loc="upper right", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, (line_ar, line_ap, line_gr, line_gp)


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 4."""
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
    print("[Step 4] Gyro integration running. Close the window or press Ctrl+C to stop.")
    print("[Step 4] TIP: hold the board still for 60 s and watch the gyro angle drift.")

    fig, (line_ar, line_ap, line_gr, line_gp) = setup_figure()

    def update(_frame):
        n = len(buf.accel_roll)
        if n == 0:
            return line_ar, line_ap, line_gr, line_gp
        xs = list(range(n))
        line_ar.set_data(xs, list(buf.accel_roll))
        line_ap.set_data(xs, list(buf.accel_pitch))
        line_gr.set_data(xs, list(buf.gyro_roll))
        line_gp.set_data(xs, list(buf.gyro_pitch))
        return line_ar, line_ap, line_gr, line_gp

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
        print("[Step 4] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
