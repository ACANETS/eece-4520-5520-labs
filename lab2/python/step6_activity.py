"""
step6_activity.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 6: Activity Detection State Machine
──────────────────────────────────────────
Classifies the board's motion state in real time using the accelerometer
magnitude as a single feature, then displays a live plot with color-coded
state background regions.

Background — Accelerometer Magnitude
──────────────────────────────────────
The vector magnitude of the accelerometer output is:

    mag = sqrt(ax² + ay² + az²)

When the board is stationary, gravity dominates and mag ≈ 1.0 g regardless of
orientation.  When the board moves, linear accelerations add to (or subtract
from) gravity, causing mag to deviate from 1.0 g.

Your Task
─────────
Implement `ActivityDetector._classify()` to distinguish three states —
STATIONARY, MOVING, and SHAKING — based on the recent history of `|mag − 1.0|`
values stored in `self._recent_dev`.

Think about: What feature of the signal distinguishes each state?  How many
recent samples should you look at?  What threshold values work best on real data?

Tune the constants at the top of this file and justify your choices in the report.

Usage:
    python step6_activity.py                    # auto-detect port
    python step6_activity.py --port /dev/ttyUSB0
    python step6_activity.py --simulate
"""

import argparse
import collections
import math
import threading
import time
from enum import Enum
from typing import Deque

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from imu_reader import make_reader

# Auto-select the best available backend (MacOSX native on Mac — no Tk required)
import sys as _sys
for _backend in (["MacOSX"] if _sys.platform == "darwin" else []) + ["Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass

# ─── Tunable Thresholds ───────────────────────────────────────────────────────
THRESH_MOVING: float   = 0.12   # |mag-1.0| above this → not stationary  (g)
THRESH_SHAKING: float  = 0.40   # |mag-1.0| above this → shaking          (g)
STAT_WINDOW: int       = 5      # window size for stationary detection (samples)
SHAKE_COUNT: int       = 3      # window size for shake detection      (samples)

# ─── Display Constants ────────────────────────────────────────────────────────
BUFFER_SIZE: int = 200
MAG_YLIM: tuple = (0.0, 3.0)   # g — 3g leaves room for vigorous shaking
ANIMATION_INTERVAL_MS: int = 40

# ─── State Colors ─────────────────────────────────────────────────────────────
STATE_COLOR = {
    "STATIONARY": "#c8f7c5",   # light green
    "MOVING":     "#fef9c3",   # light yellow
    "SHAKING":    "#fdd5d5",   # light red
}


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 6 — Activity detection state machine."
    )
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port. Auto-detected if not specified.")
    parser.add_argument("--simulate", action="store_true",
                        help="Use synthetic IMU data instead of real hardware.")
    return parser.parse_args()


# =============================================================================
# ActivityState
# =============================================================================

class ActivityState(str, Enum):
    """Motion activity states."""
    STATIONARY = "STATIONARY"
    MOVING     = "MOVING"
    SHAKING    = "SHAKING"


# =============================================================================
# ActivityDetector
# =============================================================================

class ActivityDetector:
    """
    Classifies motion activity from accelerometer magnitude.

    Each call to update() appends the latest deviation |mag − 1.0| to a
    rolling history buffer, then calls _classify() to decide the current state.

    Attributes:
        state (ActivityState): Current state.
    """

    def __init__(self) -> None:
        self.state: ActivityState = ActivityState.STATIONARY
        self._recent_dev: collections.deque = collections.deque(maxlen=max(STAT_WINDOW, SHAKE_COUNT))

    def update(self, mag: float) -> ActivityState:
        """
        Feed one magnitude sample and return the (possibly updated) state.

        Args:
            mag: Accelerometer vector magnitude in g.

        Returns:
            Current ActivityState after processing this sample.
        """
        dev = abs(mag - 1.0)
        self._recent_dev.append(dev)

        # Determine candidate state from the last N samples
        new_state = self._classify()
        if new_state != self.state:
            self.state = new_state
        return self.state

    def _classify(self) -> ActivityState:
        """Apply the state-machine rules to recent deviation history."""
        devs = list(self._recent_dev)

        # ── TODO (EECE 5520) ───────────────────────────────────────────────
        # Implement your activity classification algorithm here.
        #
        # You have access to:
        #   devs            — list of recent |mag - 1.0| values (newest at end)
        #   THRESH_MOVING   — deviation threshold constant (g)
        #   THRESH_SHAKING  — deviation threshold constant (g)
        #   STAT_WINDOW     — window size constant (samples)
        #   SHAKE_COUNT     — window size constant (samples)
        #   self.state      — the current state (useful if not enough data yet)
        #
        # Think about what signal properties — magnitude, duration, consistency —
        # best distinguish STATIONARY, MOVING, and SHAKING.  There is more than
        # one valid approach.  Return the appropriate ActivityState enum value.
        # ──────────────────────────────────────────────────────────────────
        raise NotImplementedError("TODO Step 6: implement _classify()")


# =============================================================================
# DataBuffer
# =============================================================================

class DataBuffer:
    """Circular buffer for magnitude values and state labels."""

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.mag:   Deque[float] = collections.deque(maxlen=maxlen)
        self.state: Deque[str]   = collections.deque(maxlen=maxlen)

    def push(self, mag: float, state: ActivityState) -> None:
        self.mag.append(mag)
        self.state.append(state.value)


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(
    reader,
    buf: DataBuffer,
    stop_event: threading.Event,
) -> None:
    """
    Background thread: reads packets, computes magnitude, classifies activity.

    State-change events are printed to stdout with timestamps.
    """
    detector = ActivityDetector()
    prev_state: ActivityState = ActivityState.STATIONARY
    ts_last_change: float = 0.0

    while not stop_event.is_set():
        pkt = reader.read_packet()
        if pkt is None:
            time.sleep(0.001)
            continue

        mag = math.sqrt(pkt["ax"] ** 2 + pkt["ay"] ** 2 + pkt["az"] ** 2)
        state = detector.update(mag)
        buf.push(mag, state)

        if state != prev_state:
            print(
                f"[{pkt['ts']:>8.3f}s]  State change: "
                f"{prev_state.value} → {state.value}  "
                f"(mag={mag:.3f}g)"
            )
            prev_state = state


# =============================================================================
# setup_figure
# =============================================================================

def setup_figure():
    """
    Create the live magnitude plot with a color-coded state background.

    Returns:
        fig, ax, line_mag, and the background span artist.
    """
    fig, ax = plt.subplots(figsize=(13, 5))
    fig.suptitle("Step 6 — Activity Detection", fontsize=13, fontweight="bold")

    ax.set_ylabel("Accel Magnitude (g)")
    ax.set_xlabel("Samples (newest on right)")
    ax.set_ylim(*MAG_YLIM)
    ax.set_xlim(0, BUFFER_SIZE)
    ax.grid(True, linestyle="--", alpha=0.4, zorder=0)

    # Reference lines
    ax.axhline(1.0, color="gray", linewidth=0.8, linestyle="--", label="1.0 g (rest)")
    ax.axhline(1.0 + THRESH_MOVING,   color="orange", linewidth=0.8, linestyle=":")
    ax.axhline(1.0 - THRESH_MOVING,   color="orange", linewidth=0.8, linestyle=":")
    ax.axhline(1.0 + THRESH_SHAKING,  color="red",    linewidth=0.8, linestyle=":")
    ax.axhline(1.0 - THRESH_SHAKING,  color="red",    linewidth=0.8, linestyle=":")

    line_mag, = ax.plot([], [], color="steelblue", linewidth=1.5, label="mag", zorder=3)

    # Legend for state colors
    patches = [
        mpatches.Patch(color=STATE_COLOR["STATIONARY"], label="STATIONARY"),
        mpatches.Patch(color=STATE_COLOR["MOVING"],     label="MOVING"),
        mpatches.Patch(color=STATE_COLOR["SHAKING"],    label="SHAKING"),
    ]
    ax.legend(handles=patches + [line_mag], loc="upper right", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, ax, line_mag


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 6."""
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
    print("[Step 6] Activity detection running. Close the window or press Ctrl+C to stop.")
    print(f"[Step 6] Thresholds: MOVING>{THRESH_MOVING}g  SHAKING>{THRESH_SHAKING}g")

    fig, ax, line_mag = setup_figure()

    def update(_frame):
        """Animation callback — draw magnitude line and color-code background."""
        n = len(buf.mag)
        if n == 0:
            return (line_mag,)

        xs = list(range(n))
        mags = list(buf.mag)
        states = list(buf.state)

        line_mag.set_data(xs, mags)

        # Remove old background patches
        for patch in list(ax.patches):
            patch.remove()

        # Color background by state for each sample region
        # Group consecutive same-state samples into spans for efficiency
        if n > 1:
            i = 0
            while i < n:
                current = states[i]
                j = i + 1
                while j < n and states[j] == current:
                    j += 1
                ax.axvspan(
                    i, j,
                    color=STATE_COLOR.get(current, "white"),
                    alpha=0.4,
                    zorder=1,
                )
                i = j

        return (line_mag,)

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=ANIMATION_INTERVAL_MS,
        blit=False,          # blit=False because we modify ax.patches
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
        print("[Step 6] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
