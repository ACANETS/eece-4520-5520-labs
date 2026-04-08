"""
step2_live_plot.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 2: Motor Proportional Control — Live Plot
───────────────────────────────────────────────
Displays a rolling 60-sample live plot of:
  - Distance (cm) with zone color bands
  - Motor PWM on a dual Y-axis

A background thread reads the serial port continuously and pushes frames into
a thread-safe deque. The main thread only renders — no blocking I/O in the
plot loop. (matplotlib is NOT thread-safe; data acquisition and rendering must
be decoupled.)

Usage:
    python step2_live_plot.py                    # auto-detect serial port
    python step2_live_plot.py --port /dev/ttyUSB0
    python step2_live_plot.py --simulate         # no hardware needed

Press Ctrl+C or close the window to stop.

Background — Proportional Control:
    The motor speed is NOT simply ON/OFF — it is proportional to distance.
    Closer objects → slower motor → more time to react.
    This is the foundation of proportional (P) control, a key concept in
    feedback control theory.

    The L293D Half-H driver translates the Arduino PWM signal (0–255) into a
    The ENA pin on the L293D connects directly to Arduino PWM — no jumper removal
    always runs at full voltage regardless of the PWM signal.
"""

import argparse
import collections
import sys
import threading
import time
from typing import Deque

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from sensor_reader import ParkingFrame, add_args, make_reader

# ─── Backend auto-selection (same pattern as Lab 2) ──────────────────────────
for _backend in (["MacOSX"] if sys.platform == "darwin" else []) + ["Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass

# ─── Constants ────────────────────────────────────────────────────────────────
BUFFER_SIZE: int = 60             # rolling 60 samples (≈ 6 s at 10 Hz)
ANIMATION_INTERVAL_MS: int = 100  # 10 fps refresh

# Zone color bands (distance in cm)
ZONE_BANDS = [
    (0,  25,  "red",    0.15, "Stop"),
    (25, 50,  "orange", 0.12, "Half"),
    (50, 75,  "yellow", 0.10, "3/4"),
    (75, 150, "green",  0.08, "Full"),
]


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 2 — Live distance + motor PWM plot."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================
# DataBuffer — thread-safe circular buffer
# =============================================================================

class DataBuffer:
    """
    Thread-safe circular buffer for ParkingFrames.

    The serial-reader thread writes; the matplotlib animation thread reads.
    Using collections.deque with fixed maxlen is sufficient for CPython's GIL.
    """

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.distance: Deque[float] = collections.deque(maxlen=maxlen)
        self.motor_pwm: Deque[int]  = collections.deque(maxlen=maxlen)

    def push(self, frame: ParkingFrame) -> None:
        """Append one frame to all channel buffers."""
        self.distance.append(frame.distance_cm)
        self.motor_pwm.append(frame.motor_pwm)


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(
    reader,
    buf: DataBuffer,
    stop_event: threading.Event,
) -> None:
    """
    Background daemon thread: reads ParkingFrames and pushes into buf.

    Args:
        reader:      A SensorReader or SensorSimulator (already connected).
        buf:         Shared DataBuffer.
        stop_event:  Set to signal exit.
    """
    while not stop_event.is_set():
        frame = reader.read()
        if frame is not None:
            buf.push(frame)
        else:
            time.sleep(0.001)


# =============================================================================
# setup_figure
# =============================================================================

def setup_figure():
    """
    Create the matplotlib figure with distance plot and dual-axis motor PWM.

    Returns:
        fig, ax_dist, ax_pwm, line_dist, line_pwm
    """
    fig, ax_dist = plt.subplots(figsize=(12, 6))
    fig.suptitle(
        "Lab 3 — Distance & Motor PWM (rolling 60 samples)",
        fontsize=13,
        fontweight="bold",
    )

    # Zone color bands
    for lo, hi, color, alpha, label in ZONE_BANDS:
        ax_dist.axhspan(lo, hi, color=color, alpha=alpha, label=f"{label} zone")

    ax_dist.set_title("Distance with Zone Bands")
    ax_dist.set_ylabel("Distance (cm)", color="tab:blue")
    ax_dist.set_ylim(0, 130)
    ax_dist.set_xlim(0, BUFFER_SIZE)
    ax_dist.set_xlabel("Samples (newest on right)")
    ax_dist.grid(True, linestyle="--", alpha=0.4)
    ax_dist.tick_params(axis="y", labelcolor="tab:blue")

    (line_dist,) = ax_dist.plot([], [], color="tab:blue", linewidth=2.0, label="Distance")
    ax_dist.legend(loc="upper left", fontsize=8)

    # Dual Y-axis for motor PWM
    ax_pwm = ax_dist.twinx()
    ax_pwm.set_ylabel("Motor PWM (0–255)", color="tab:red")
    ax_pwm.set_ylim(0, 280)
    ax_pwm.tick_params(axis="y", labelcolor="tab:red")

    (line_pwm,) = ax_pwm.plot([], [], color="tab:red", linewidth=1.5, linestyle="--", label="Motor PWM")
    ax_pwm.legend(loc="upper right", fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig, ax_dist, ax_pwm, line_dist, line_pwm


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 2."""
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
    print("[Step 2] Live plot running. Close the window or press Ctrl+C to stop.")

    fig, ax_dist, ax_pwm, line_dist, line_pwm = setup_figure()

    def update(_frame):
        """Animation callback — runs on the main thread."""
        n = len(buf.distance)
        if n == 0:
            return line_dist, line_pwm

        xs = list(range(n))
        line_dist.set_data(xs, list(buf.distance))
        line_pwm.set_data(xs, list(buf.motor_pwm))

        return line_dist, line_pwm

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
        pass
    finally:
        try:
            ani.event_source.stop()  # type: ignore[union-attr]
        except Exception:
            pass
        stop_event.set()
        reader.close()
        print("[Step 2] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
