"""
step6_dashboard.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 6: Python Dashboard  *** EECE 5520 ONLY ***
──────────────────────────────────────────────────
This script is provided as-is — no student TODOs.

Requires: step5_protocol.py to be complete (imports encode_frame, decode_frame).

Displays a 4-panel matplotlib dashboard updating at 10 Hz via animation:
  - Top-left:     Rolling 30s distance plot with zone color bands
  - Top-right:    Motor PWM over time
  - Bottom-left:  Alarm state timeline (color-coded blocks)
  - Bottom-right: Protocol stats (frames OK, bad frames, frame rate, error rate)

Usage:
    python step6_dashboard.py --simulate      # no hardware needed
    python step6_dashboard.py                 # live hardware

Press Ctrl+C or close the window to stop.
"""

import argparse
import collections
import sys
import threading
import time
from typing import Deque

import matplotlib
import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

# ─── Backend auto-selection ───────────────────────────────────────────────────
for _backend in (["MacOSX"] if sys.platform == "darwin" else []) + ["Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass

from sensor_reader import ALARM_NAMES, ZONE_NAMES, ParkingFrame, add_args, make_reader

# Import student's protocol (Step 5 must be complete)
try:
    from step5_protocol import decode_frame, encode_frame
    _PROTOCOL_OK = True
except (ImportError, Exception):
    _PROTOCOL_OK = False

# ─── Constants ────────────────────────────────────────────────────────────────
BUFFER_SECS: int       = 30               # rolling window in seconds
SAMPLE_RATE_HZ: float  = 10.0            # expected samples/second
BUFFER_SIZE: int       = int(BUFFER_SECS * SAMPLE_RATE_HZ)
ANIMATION_INTERVAL_MS: int = 100         # 10 fps

# Zone colors for distance bands
ZONE_BANDS = [
    (0,  25,  "red",    0.15, "Stop"),
    (25, 50,  "orange", 0.12, "Half"),
    (50, 75,  "yellow", 0.10, "3/4"),
    (75, 150, "green",  0.08, "Full"),
]

ALARM_COLORS = {0: "green", 1: "orange", 2: "red"}


# =============================================================================
# DataBuffer
# =============================================================================

class DataBuffer:
    """Thread-safe circular buffer for dashboard data."""

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.distance:    Deque[float] = collections.deque(maxlen=maxlen)
        self.motor_pwm:   Deque[int]   = collections.deque(maxlen=maxlen)
        self.alarm_state: Deque[int]   = collections.deque(maxlen=maxlen)
        self.timestamps:  Deque[float] = collections.deque(maxlen=maxlen)

    def push(self, frame: ParkingFrame) -> None:
        self.distance.append(frame.distance_cm)
        self.motor_pwm.append(frame.motor_pwm)
        self.alarm_state.append(frame.alarm_state)
        self.timestamps.append(frame.timestamp)


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(reader, buf: DataBuffer, stop_event: threading.Event) -> None:
    """Background thread: reads frames and pushes to buffer."""
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
    """Create the 2×2 dashboard figure."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Lab 3 Dashboard — Smart Parking Sensor  [EECE 5520]",
        fontsize=14, fontweight="bold",
    )

    ax_dist, ax_pwm, ax_alarm, ax_stats = (
        axes[0][0], axes[0][1], axes[1][0], axes[1][1]
    )

    # ── Top-left: distance with zone bands ────────────────────────────────────
    for lo, hi, color, alpha, label in ZONE_BANDS:
        ax_dist.axhspan(lo, hi, color=color, alpha=alpha)
    ax_dist.set_title("Distance (30 s rolling)")
    ax_dist.set_ylabel("Distance (cm)")
    ax_dist.set_ylim(0, 130)
    ax_dist.set_xlim(0, BUFFER_SIZE)
    ax_dist.set_xlabel("Samples")
    ax_dist.grid(True, linestyle="--", alpha=0.4)
    (line_dist,) = ax_dist.plot([], [], color="tab:blue", linewidth=2.0, label="Distance")
    ax_dist.legend(loc="upper left", fontsize=8)

    # ── Top-right: motor PWM ──────────────────────────────────────────────────
    ax_pwm.set_title("Motor PWM")
    ax_pwm.set_ylabel("PWM (0–255)")
    ax_pwm.set_ylim(-10, 270)
    ax_pwm.set_xlim(0, BUFFER_SIZE)
    ax_pwm.set_xlabel("Samples")
    ax_pwm.grid(True, linestyle="--", alpha=0.4)
    ax_pwm.axhline(255, color="green",  linestyle=":", alpha=0.7, label="Full (255)")
    ax_pwm.axhline(191, color="yellow", linestyle=":", alpha=0.7, label="3/4  (191)")
    ax_pwm.axhline(128, color="orange", linestyle=":", alpha=0.7, label="Half (128)")
    ax_pwm.axhline(0,   color="red",    linestyle=":", alpha=0.7, label="Stop (0)")
    (line_pwm,) = ax_pwm.plot([], [], color="tab:red", linewidth=1.5, label="Motor PWM")
    ax_pwm.legend(loc="upper right", fontsize=7)

    # ── Bottom-left: alarm state timeline ────────────────────────────────────
    ax_alarm.set_title("Alarm State Timeline")
    ax_alarm.set_ylabel("Alarm State")
    ax_alarm.set_ylim(-0.5, 2.5)
    ax_alarm.set_xlim(0, BUFFER_SIZE)
    ax_alarm.set_xlabel("Samples")
    ax_alarm.set_yticks([0, 1, 2])
    ax_alarm.set_yticklabels(["0: Silent", "1: Beeping", "2: Continuous"])
    ax_alarm.grid(True, linestyle="--", alpha=0.4)
    (line_alarm,) = ax_alarm.plot([], [], color="tab:purple", linewidth=1.5, drawstyle="steps-post")
    patches = [
        mpatches.Patch(color="green",  label="Silent"),
        mpatches.Patch(color="orange", label="Beeping"),
        mpatches.Patch(color="red",    label="Continuous"),
    ]
    ax_alarm.legend(handles=patches, loc="upper right", fontsize=8)

    # ── Bottom-right: protocol stats ──────────────────────────────────────────
    ax_stats.set_title("Protocol Statistics")
    ax_stats.axis("off")
    stats_text = ax_stats.text(
        0.05, 0.5, "Waiting for data…",
        transform=ax_stats.transAxes,
        fontsize=13, verticalalignment="center",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="whitesmoke", alpha=0.8),
    )

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig, line_dist, line_pwm, line_alarm, stats_text


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 6."""
    args = parse_args()

    if not _PROTOCOL_OK:
        print(
            "[Step 6] ERROR: Could not import from step5_protocol.py.\n"
            "         Complete Step 5 (encode_frame + decode_frame) first,\n"
            "         then re-run step6_dashboard.py."
        )
        return

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
    print("[Step 6] Dashboard running. Close the window or Ctrl+C to stop.")

    fig, line_dist, line_pwm, line_alarm, stats_text = setup_figure()

    _frame_times: collections.deque = collections.deque(maxlen=100)
    _last_ok = [0]
    _last_bad = [0]

    def update(_frame_idx):
        n = len(buf.distance)
        xs = list(range(n))

        if n > 0:
            line_dist.set_data(xs, list(buf.distance))
            line_pwm.set_data(xs, list(buf.motor_pwm))
            line_alarm.set_data(xs, list(buf.alarm_state))

            # Track frame rate
            now = time.time()
            _frame_times.append(now)

        # Protocol stats
        stats = reader.stats
        frames_ok  = stats.get("frames_ok", 0)
        frames_bad = stats.get("frames_bad", 0)
        total = frames_ok + frames_bad
        error_rate = (frames_bad / total * 100) if total > 0 else 0.0

        fps = 0.0
        if len(_frame_times) >= 2:
            elapsed = _frame_times[-1] - _frame_times[0]
            if elapsed > 0:
                fps = (len(_frame_times) - 1) / elapsed

        stats_str = (
            f"  Frames OK:    {frames_ok:>6d}\n"
            f"  Frames bad:   {frames_bad:>6d}\n"
            f"  Total:        {total:>6d}\n"
            f"  Error rate:   {error_rate:>5.1f}%\n"
            f"  Frame rate:   {fps:>5.1f} Hz\n"
            f"\n"
            f"  Protocol: 7-byte binary\n"
            f"  Start byte: 0xAA\n"
            f"  Checksum: XOR payload\n"
        )
        stats_text.set_text(stats_str)

        return line_dist, line_pwm, line_alarm, stats_text

    ani = animation.FuncAnimation(
        fig, update,
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
        print("[Step 6] Done.")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 6 (5520 only) — 4-panel parking sensor dashboard."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================

if __name__ == "__main__":
    main()
