"""
step4_buzzer_alarm.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 4: Buzzer Alarm Logic
───────────────────────────
Student Task:
    Implement classify_alarm(distance_cm) to classify buzzer alarm state
    from a distance reading.

Alarm states (must match Arduino firmware):
    distance ≥ 60 cm → 0 (Silent)
    20 ≤ distance < 60 → 1 (Beeping)
    distance < 20 cm   → 2 (Continuous alarm)

After implementing classify_alarm(), run this script to:
  1. Show alarm state vs distance bar visualization
  2. Compare your Python classification against live Arduino alarm_state values

Usage:
    python step4_buzzer_alarm.py --simulate      # offline visualization
    python step4_buzzer_alarm.py                 # compare against hardware
"""

import argparse
import sys
import time

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from sensor_reader import ALARM_NAMES, add_args, make_reader

# ─── Backend auto-selection ───────────────────────────────────────────────────
for _backend in (["MacOSX"] if sys.platform == "darwin" else []) + ["Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass

ALARM_COLORS = {0: "green", 1: "orange", 2: "red"}


# =============================================================================
# classify_alarm  ← STUDENTS IMPLEMENT THIS FUNCTION
# =============================================================================

def classify_alarm(distance_cm: float) -> int:
    """
    Classify buzzer alarm state from a distance measurement.

    The buzzer behavior depends on how close an object is to the sensor:
      - Far away → silent (no need to alarm)
      - Getting closer → beeping (increasing urgency)
      - Very close → continuous alarm (danger!)

    Alarm state codes:
        0 = Silent      (distance ≥ 60 cm)
        1 = Beeping     (20 cm ≤ distance < 60 cm)
        2 = Continuous  (distance < 20 cm)

    Args:
        distance_cm: Distance in centimetres (float, ≥ 0).

    Returns:
        Integer alarm state: 0, 1, or 2.
    """
    raise NotImplementedError(
        "TODO: classify alarm state from distance. Consider zone boundaries."
    )


# =============================================================================
# Reference implementation (used for comparison — do not modify)
# =============================================================================

def _reference_classify_alarm(distance_cm: float) -> int:
    if distance_cm >= 60:
        return 0
    elif distance_cm >= 20:
        return 1
    else:
        return 2


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 4 — Buzzer alarm classification: visualize and compare."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================
# plot_alarm_zones
# =============================================================================

def plot_alarm_zones(use_student: bool = True) -> None:
    distances = np.linspace(0, 130, 131)
    fn = classify_alarm if use_student else _reference_classify_alarm
    alarm_states = [fn(float(d)) for d in distances]
    colors = [ALARM_COLORS[s] for s in alarm_states]

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.bar(distances, alarm_states, color=colors, width=1.0, align="edge")
    ax.axvline(20, color="darkred",  linestyle="--", alpha=0.8, label="Alarm threshold 20 cm")
    ax.axvline(60, color="darkblue", linestyle="--", alpha=0.8, label="Silent threshold 60 cm")

    patches = [
        mpatches.Patch(color="green",  label="0 — Silent"),
        mpatches.Patch(color="orange", label="1 — Beeping"),
        mpatches.Patch(color="red",    label="2 — Continuous"),
    ]
    ax.legend(handles=patches, loc="upper right", fontsize=9)
    ax.set_xlabel("Distance (cm)")
    ax.set_ylabel("Alarm State (0/1/2)")
    ax.set_title("Step 4 — Buzzer Alarm State vs Distance")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["0: Silent", "1: Beeping", "2: Continuous"])
    ax.set_xlim(0, 130)
    ax.grid(True, axis="x", linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.show(block=False)


# =============================================================================
# live_comparison
# =============================================================================

def live_comparison(reader) -> None:
    print(
        f"\n{'Distance':>10s}  {'Your Alarm':>12s}  {'Arduino Alarm':>14s}  {'Match':>6s}"
    )
    print("─" * 54)

    frame_count = 0
    mismatch_count = 0

    try:
        with reader:
            while True:
                frame = reader.read()
                if frame is None:
                    time.sleep(0.001)
                    continue

                your_alarm = classify_alarm(frame.distance_cm)
                your_name  = ALARM_NAMES.get(your_alarm, str(your_alarm))
                hw_name    = ALARM_NAMES.get(frame.alarm_state, str(frame.alarm_state))
                match = "✓" if your_alarm == frame.alarm_state else "✗ ← check thresholds"
                if your_alarm != frame.alarm_state:
                    mismatch_count += 1

                print(
                    f"{frame.distance_cm:>9.1f}  "
                    f"{your_name:>12s}  "
                    f"{hw_name:>14s}  "
                    f"{match}"
                )
                frame_count += 1

    except KeyboardInterrupt:
        pass

    print(
        f"\n[Step 4] {frame_count} frames. Mismatches: {mismatch_count} "
        f"({'perfect!' if mismatch_count == 0 else 'check your alarm thresholds'})"
    )


# =============================================================================
# main
# =============================================================================

def main() -> None:
    args = parse_args()

    try:
        classify_alarm(50.0)
        student_ready = True
    except NotImplementedError:
        student_ready = False
        print(
            "[Step 4] classify_alarm() is not yet implemented.\n"
            "         Showing reference visualization.\n"
            "         Implement the function, then re-run."
        )
        plot_alarm_zones(use_student=False)
        plt.show()
        return

    plot_alarm_zones(use_student=True)

    if not args.simulate:
        reader = make_reader(port=args.port, simulate=False)
        print("[Step 4] Comparing your classify_alarm() against live Arduino values.")
        print("         Press Ctrl+C to stop.")
        live_comparison(reader)
    else:
        print("[Step 4] --simulate: no live comparison. Connect hardware to compare.")
        plt.show()


# =============================================================================

if __name__ == "__main__":
    main()
