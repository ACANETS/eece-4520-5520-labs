"""
step2_live_plot.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 2: Real-Time 6-Axis IMU Plot
───────────────────────────────────
Displays a live scrolling plot of all six IMU axes:
  - Top subplot:    Accelerometer X, Y, Z  (g)
  - Bottom subplot: Gyroscope X, Y, Z      (°/s)

A background thread reads the serial port continuously and pushes packets into
a thread-safe deque. The main thread only renders — no blocking I/O in the
plot loop. This pattern is important: matplotlib is NOT thread-safe, so data
acquisition and rendering must be decoupled.

Usage:
    python step2_live_plot.py                    # auto-detect serial port
    python step2_live_plot.py --port /dev/ttyUSB0
    python step2_live_plot.py --simulate         # no hardware needed

Press Ctrl+C or close the window to stop.
"""

import argparse
import collections
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
BUFFER_SIZE: int = 200           # samples in circular buffer (~4 s at 50 Hz)
ACCEL_YLIM: tuple = (-2.0, 2.0)  # g
GYRO_YLIM: tuple = (-250.0, 250.0)  # °/s
ANIMATION_INTERVAL_MS: int = 40  # ~25 fps refresh rate for matplotlib


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 2 — Real-time 6-axis IMU plot."
    )
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port. Auto-detected if not specified.")
    parser.add_argument("--simulate", action="store_true",
                        help="Use synthetic IMU data instead of real hardware.")
    return parser.parse_args()


# =============================================================================
# DataBuffer — thread-safe shared state
# =============================================================================

class DataBuffer:
    """
    Thread-safe circular buffer for IMU packets.

    The serial-reader thread writes to this buffer; the matplotlib animation
    thread reads from it. Using collections.deque with a fixed maxlen is
    sufficient for thread safety of append/read operations in CPython.

    Attributes:
        ts, ax, ay, az, gx, gy, gz: Individual deques for each channel.
    """

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.ts: Deque[float] = collections.deque(maxlen=maxlen)
        self.ax: Deque[float] = collections.deque(maxlen=maxlen)
        self.ay: Deque[float] = collections.deque(maxlen=maxlen)
        self.az: Deque[float] = collections.deque(maxlen=maxlen)
        self.gx: Deque[float] = collections.deque(maxlen=maxlen)
        self.gy: Deque[float] = collections.deque(maxlen=maxlen)
        self.gz: Deque[float] = collections.deque(maxlen=maxlen)

    def push(self, pkt: Dict[str, float]) -> None:
        """Append one packet to all channel buffers."""
        self.ts.append(pkt["ts"])
        self.ax.append(pkt["ax"])
        self.ay.append(pkt["ay"])
        self.az.append(pkt["az"])
        self.gx.append(pkt["gx"])
        self.gy.append(pkt["gy"])
        self.gz.append(pkt["gz"])


# =============================================================================
# serial_reader_thread
# =============================================================================

def serial_reader_thread(
    reader,
    buf: DataBuffer,
    stop_event: threading.Event,
) -> None:
    """
    Background daemon thread: reads packets from the IMU and pushes into buf.

    Args:
        reader:      An IMUReader or IMUSimulator (already connected).
        buf:         Shared DataBuffer to push packets into.
        stop_event:  Set this Event to signal the thread to exit.
    """
    while not stop_event.is_set():
        pkt = reader.read_packet()
        if pkt is not None:
            buf.push(pkt)
        else:
            time.sleep(0.001)


# =============================================================================
# setup_figure
# =============================================================================

def setup_figure():
    """
    Create and configure the matplotlib figure with two subplots.

    Returns:
        fig, ax_accel, ax_gyro, line objects for each of the 6 channels.
    """
    fig, (ax_accel, ax_gyro) = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    fig.suptitle("MPU-6050 Real-Time IMU Data", fontsize=14, fontweight="bold")

    # ── Accelerometer subplot ─────────────────────────────────────────────────
    ax_accel.set_title("Accelerometer")
    ax_accel.set_ylabel("Acceleration (g)")
    ax_accel.set_ylim(*ACCEL_YLIM)
    ax_accel.set_xlim(0, BUFFER_SIZE)
    ax_accel.grid(True, linestyle="--", alpha=0.5)
    ax_accel.axhline(0, color="black", linewidth=0.5)

    line_ax, = ax_accel.plot([], [], color="tab:red",   label="ax", linewidth=1.2)
    line_ay, = ax_accel.plot([], [], color="tab:green", label="ay", linewidth=1.2)
    line_az, = ax_accel.plot([], [], color="tab:blue",  label="az", linewidth=1.2)
    ax_accel.legend(loc="upper right", fontsize=9)

    # ── Gyroscope subplot ─────────────────────────────────────────────────────
    ax_gyro.set_title("Gyroscope")
    ax_gyro.set_xlabel("Samples (newest on right)")
    ax_gyro.set_ylabel("Angular Velocity (°/s)")
    ax_gyro.set_ylim(*GYRO_YLIM)
    ax_gyro.set_xlim(0, BUFFER_SIZE)
    ax_gyro.grid(True, linestyle="--", alpha=0.5)
    ax_gyro.axhline(0, color="black", linewidth=0.5)

    line_gx, = ax_gyro.plot([], [], color="tab:red",    label="gx", linewidth=1.2)
    line_gy, = ax_gyro.plot([], [], color="tab:green",  label="gy", linewidth=1.2)
    line_gz, = ax_gyro.plot([], [], color="tab:blue",   label="gz", linewidth=1.2)
    ax_gyro.legend(loc="upper right", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig, (line_ax, line_ay, line_az, line_gx, line_gy, line_gz)


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

    # Start background reader thread (daemon=True → auto-killed when main exits)
    t = threading.Thread(
        target=serial_reader_thread,
        args=(reader, buf, stop_event),
        daemon=True,
    )
    t.start()
    print("[Step 2] Live plot running. Close the window or press Ctrl+C to stop.")

    fig, (line_ax, line_ay, line_az, line_gx, line_gy, line_gz) = setup_figure()
    x_data = list(range(BUFFER_SIZE))

    def update(_frame):
        """Animation callback — runs on the main thread."""
        n = len(buf.ax)
        if n == 0:
            return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

        xs = list(range(n))

        line_ax.set_data(xs, list(buf.ax))
        line_ay.set_data(xs, list(buf.ay))
        line_az.set_data(xs, list(buf.az))
        line_gx.set_data(xs, list(buf.gx))
        line_gy.set_data(xs, list(buf.gy))
        line_gz.set_data(xs, list(buf.gz))

        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

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
        print("[Step 2] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
