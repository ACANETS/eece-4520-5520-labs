"""
step5_comp_filter.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 5: Complementary Filter — Sensor Fusion
─────────────────────────────────────────────
Fuses accelerometer and gyroscope data to produce a stable, drift-free angle
estimate that combines the best properties of both sensors.

Background — Complementary Filter
────────────────────────────────────
The accelerometer is accurate in the long run but noisy during motion.
The gyroscope is smooth and responsive but drifts over time.

A complementary filter is a simple, computationally cheap fusion:

    fused_angle = ALPHA * (fused_angle + gyro * dt) + (1 - ALPHA) * accel_angle

Intuition:
  • The term  ALPHA * (... + gyro * dt)  uses the gyro for short-term dynamics.
    ALPHA close to 1 means "mostly trust the gyro."
  • The term  (1 - ALPHA) * accel_angle  slowly pulls the estimate back toward
    the gravity-based reference, correcting gyro drift over time.

Choosing ALPHA:
  • ALPHA = 0.98  is a common starting point for 50 Hz systems.
    It applies a time constant of ~1 / ((1-0.98) * 50) ≈ 1 second.
  • ALPHA → 1.0 : pure gyro integration (drifts, but smooth)
  • ALPHA → 0.0 : pure accelerometer (noisy during motion, no drift)

Student exercise: Try ALPHA = 0.80, 0.95, 0.98, 0.999. Observe the trade-off
between responsiveness and drift correction. 
TODO: Describe your observation in your lab report. 

Usage:
    python step5_comp_filter.py                    # auto-detect port
    python step5_comp_filter.py --port /dev/ttyUSB0
    python step5_comp_filter.py --simulate
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

# ─── Tunable Constants ────────────────────────────────────────────────────────
# Change ALPHA to experiment with the filter behaviour.
ALPHA: float = 0.98              # complementary filter coefficient [0, 1]

# ─── Display Constants ────────────────────────────────────────────────────────
BUFFER_SIZE: int = 200            # ~4 seconds at 50 Hz
ANGLE_YLIM: tuple = (-90.0, 90.0) # degrees
ANIMATION_INTERVAL_MS: int = 40
PRINT_EVERY_N: int = 10


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 5 — Complementary filter sensor fusion."
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

    Args:
        ax, ay, az: Acceleration (g) in body-frame X, Y, Z.

    Returns:
        (roll_deg, pitch_deg) in degrees.
    """
    roll_deg  = math.degrees(math.atan2(ay, az))
    pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))
    return roll_deg, pitch_deg


# =============================================================================
# ComplementaryFilter
# =============================================================================

class ComplementaryFilter:
    """
    Stateful complementary filter that fuses accelerometer and gyroscope data.

    Attributes:
        roll (float):  Current fused roll estimate (degrees).
        pitch (float): Current fused pitch estimate (degrees).
        alpha (float): Filter coefficient (higher = trust gyro more).
    """

    def __init__(self, alpha: float = ALPHA) -> None:
        self.alpha: float = alpha
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self._prev_ts: Optional[float] = None
        self._initialised: bool = False

    def update(
        self,
        ts: float,
        ax: float, ay: float, az: float,
        gx: float, gy: float,
    ) -> tuple:
        """
        Update the complementary filter with a new IMU packet.

        On first call, bootstraps roll/pitch from the accelerometer so the
        filter starts at the true orientation rather than 0°.

        Args:
            ts:         Timestamp in seconds.
            ax, ay, az: Accelerometer readings (g).
            gx, gy:     Gyro roll/pitch rates (°/s).

        Returns:
            (roll_deg, pitch_deg) fused angle estimate in degrees.
        """
        a_roll, a_pitch = accel_angles(ax, ay, az)

        if not self._initialised:
            self.roll = a_roll
            self.pitch = a_pitch
            self._prev_ts = ts
            self._initialised = True
            return self.roll, self.pitch

        dt = ts - self._prev_ts
        self._prev_ts = ts

        if dt <= 0:
            return self.roll, self.pitch

        # ── TODO ──────────────────────────────────────────────────────────
        # Apply the complementary filter update for roll and pitch.
        #
        # The complementary filter fuses the gyroscope (short-term accuracy)
        # with the accelerometer (long-term stability):
        #
        #   angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
        #
        # Variables available:
        #   self.alpha        — filter coefficient (e.g. 0.98)
        #   self.roll         — current roll estimate (degrees)
        #   self.pitch        — current pitch estimate (degrees)
        #   gx                — gyro roll rate  (°/s)
        #   gy                — gyro pitch rate (°/s)
        #   dt                — elapsed time    (seconds)
        #   a_roll, a_pitch   — accelerometer-only angle estimates (degrees)
        #
        # Update self.roll and self.pitch, then return (self.roll, self.pitch).
        # ──────────────────────────────────────────────────────────────────
        raise NotImplementedError("TODO Step 5: implement complementary filter update")


# =============================================================================
# DataBuffer
# =============================================================================

class DataBuffer:
    """Circular buffer for accel, gyro, and fused angle estimates."""

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.accel_roll:  Deque[float] = collections.deque(maxlen=maxlen)
        self.accel_pitch: Deque[float] = collections.deque(maxlen=maxlen)
        self.gyro_roll:   Deque[float] = collections.deque(maxlen=maxlen)
        self.gyro_pitch:  Deque[float] = collections.deque(maxlen=maxlen)
        self.fused_roll:  Deque[float] = collections.deque(maxlen=maxlen)
        self.fused_pitch: Deque[float] = collections.deque(maxlen=maxlen)

        # Track gyro-integrated angles independently for panel 2
        self._gyro_roll: float = 0.0
        self._gyro_pitch: float = 0.0
        self._gyro_prev_ts: Optional[float] = None
        self._gyro_init: bool = False

    def push(
        self,
        ts: float,
        a_roll: float, a_pitch: float,
        gx: float, gy: float,
        f_roll: float, f_pitch: float,
    ) -> None:
        """Append one sample of all three angle estimates."""
        self.accel_roll.append(a_roll)
        self.accel_pitch.append(a_pitch)
        self.fused_roll.append(f_roll)
        self.fused_pitch.append(f_pitch)

        # Simple gyro integration for display panel
        if not self._gyro_init:
            self._gyro_roll = a_roll
            self._gyro_pitch = a_pitch
            self._gyro_prev_ts = ts
            self._gyro_init = True
        else:
            dt = ts - self._gyro_prev_ts
            self._gyro_prev_ts = ts
            if dt > 0:
                self._gyro_roll  += gx * dt
                self._gyro_pitch += gy * dt

        self.gyro_roll.append(self._gyro_roll)
        self.gyro_pitch.append(self._gyro_pitch)


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(
    reader,
    buf: DataBuffer,
    stop_event: threading.Event,
) -> None:
    """
    Background thread: reads packets, runs all three angle estimators, pushes
    results to the shared buffer.
    """
    filt = ComplementaryFilter(alpha=ALPHA)
    count = 0

    while not stop_event.is_set():
        pkt = reader.read_packet()
        if pkt is None:
            time.sleep(0.001)
            continue

        a_roll, a_pitch = accel_angles(pkt["ax"], pkt["ay"], pkt["az"])
        f_roll, f_pitch = filt.update(
            pkt["ts"],
            pkt["ax"], pkt["ay"], pkt["az"],
            pkt["gx"], pkt["gy"],
        )

        buf.push(
            pkt["ts"],
            a_roll, a_pitch,
            pkt["gx"], pkt["gy"],
            f_roll, f_pitch,
        )

        count += 1
        if count % PRINT_EVERY_N == 0:
            print(
                f"[{pkt['ts']:>8.3f}s]  "
                f"Accel Roll={a_roll:>7.2f}°  "
                f"Fused Roll={f_roll:>7.2f}°  "
                f"Accel Pitch={a_pitch:>7.2f}°  "
                f"Fused Pitch={f_pitch:>7.2f}°"
            )


# =============================================================================
# setup_figure
# =============================================================================

def setup_figure():
    """
    Create a three-panel figure: accel | gyro | fused.

    Returns:
        fig and a tuple of six line artists.
    """
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    fig.suptitle(
        f"Step 5 — Complementary Filter (ALPHA = {ALPHA})",
        fontsize=13, fontweight="bold",
    )

    panels = [
        (ax1, "Accelerometer Angles",   "tab:red",    "tab:blue"),
        (ax2, "Gyro-Integrated Angles", "tab:orange", "tab:cyan"),
        (ax3, "Fused (Comp. Filter)",   "tab:green",  "tab:purple"),
    ]

    lines = []
    for i, (ax, title, c_roll, c_pitch) in enumerate(panels):
        ax.set_title(title)
        ax.set_xlabel("Samples")
        ax.set_ylim(*ANGLE_YLIM)
        ax.set_xlim(0, BUFFER_SIZE)
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.axhline(0, color="black", linewidth=0.5)
        if i == 0:
            ax.set_ylabel("Angle (°)")
        lr, = ax.plot([], [], color=c_roll,  label="Roll",  linewidth=1.5)
        lp, = ax.plot([], [], color=c_pitch, label="Pitch", linewidth=1.5)
        ax.legend(loc="upper right", fontsize=9)
        lines.extend([lr, lp])

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return fig, tuple(lines)  # (ar, ap, gr, gp, fr, fp)


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 5."""
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
    print(f"[Step 5] Complementary filter running (ALPHA={ALPHA}). Close window to stop.")
    print("[Step 5] Edit ALPHA at the top of this file and restart to experiment.")

    fig, (l_ar, l_ap, l_gr, l_gp, l_fr, l_fp) = setup_figure()

    def update(_frame):
        n = len(buf.accel_roll)
        if n == 0:
            return l_ar, l_ap, l_gr, l_gp, l_fr, l_fp
        xs = list(range(n))
        l_ar.set_data(xs, list(buf.accel_roll))
        l_ap.set_data(xs, list(buf.accel_pitch))
        l_gr.set_data(xs, list(buf.gyro_roll))
        l_gp.set_data(xs, list(buf.gyro_pitch))
        l_fr.set_data(xs, list(buf.fused_roll))
        l_fp.set_data(xs, list(buf.fused_pitch))
        return l_ar, l_ap, l_gr, l_gp, l_fr, l_fp

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
        print("[Step 5] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
