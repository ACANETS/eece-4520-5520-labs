"""
step3_motor_map.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 3: Motor Speed Mapping
────────────────────────────
Student Task:
    Implement distance_to_pwm(distance_cm) to map sensor distance to motor
    PWM duty cycle using the same zone logic as the Arduino firmware.

Zone table (must match Arduino):
    > 75 cm   → 255  (Full speed)
    50–75 cm  → 191  (3/4 speed)
    25–50 cm  → 128  (Half speed)
    0–25 cm   →   0  (Stop)

After implementing distance_to_pwm(), run this script to:
  1. Plot the PWM step function vs distance
  2. Compare your Python implementation against live Arduino motor_pwm values

Usage:
    python step3_motor_map.py --simulate      # offline: show step plot only
    python step3_motor_map.py                 # compare against hardware
"""

import argparse
import sys
import time

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from sensor_reader import add_args, make_reader

# ─── Backend auto-selection ───────────────────────────────────────────────────
for _backend in (["MacOSX"] if sys.platform == "darwin" else []) + ["Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass


# =============================================================================
# distance_to_pwm  ← STUDENTS IMPLEMENT THIS FUNCTION
# =============================================================================

def distance_to_pwm(distance_cm: float) -> int:
    """
    Map a distance measurement to a motor PWM value (0–255).

    The motor speed is inversely proportional to distance — when an object is
    far away the motor runs faster; as it approaches the motor slows and stops.

    Zone boundaries (must match the Arduino firmware exactly):
        distance > 75 cm  →  PWM = 255  (Full speed)
        50 < distance ≤ 75 → PWM = 191  (3/4 speed)
        25 < distance ≤ 50 → PWM = 128  (Half speed)
        distance ≤ 25 cm  →  PWM = 0    (Stop)

    Args:
        distance_cm: Distance in centimetres (float, ≥ 0).

    Returns:
        Integer PWM value in range 0–255.
    """
    raise NotImplementedError(
        "TODO: map distance_cm to motor PWM (0–255). Closer = slower."
    )


# =============================================================================
# Reference implementation (used for comparison plot — do not modify)
# =============================================================================

def _reference_distance_to_pwm(distance_cm: float) -> int:
    """Reference implementation for comparison."""
    if distance_cm > 75:
        return 255
    elif distance_cm > 50:
        return 191
    elif distance_cm > 25:
        return 128
    else:
        return 0


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 3 — Motor PWM mapping: plot and live comparison."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================
# plot_pwm_curve
# =============================================================================

def plot_pwm_curve(use_student: bool = True) -> None:
    distances = np.linspace(0, 130, 1000)
    fn = distance_to_pwm if use_student else _reference_distance_to_pwm
    pwm_values = [fn(float(d)) for d in distances]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.step(distances, pwm_values, where="post", color="tab:blue", linewidth=2.5,
            label="Your distance_to_pwm()")
    ax.set_xlabel("Distance (cm)")
    ax.set_ylabel("Motor PWM (0–255)")
    ax.set_title("Step 3 — Motor PWM vs Distance (step function)")
    ax.set_ylim(-10, 270)
    ax.set_xlim(0, 130)
    ax.axvline(25, color="red",    linestyle="--", alpha=0.6, label="Zone boundary 25 cm")
    ax.axvline(50, color="orange", linestyle="--", alpha=0.6, label="Zone boundary 50 cm")
    ax.axvline(75, color="green",  linestyle="--", alpha=0.6, label="Zone boundary 75 cm")
    ax.legend(fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.show(block=False)


# =============================================================================
# live_comparison
# =============================================================================

def live_comparison(reader) -> None:
    print(
        f"\n{'Distance':>10s}  {'Your PWM':>10s}  {'Arduino PWM':>12s}  {'Match':>6s}"
    )
    print("─" * 48)

    frame_count = 0
    mismatch_count = 0

    try:
        with reader:
            while True:
                frame = reader.read()
                if frame is None:
                    time.sleep(0.001)
                    continue

                your_pwm = distance_to_pwm(frame.distance_cm)
                match = "✓" if your_pwm == frame.motor_pwm else "✗ ← check boundaries"
                if your_pwm != frame.motor_pwm:
                    mismatch_count += 1

                print(
                    f"{frame.distance_cm:>9.1f}  "
                    f"{your_pwm:>10d}  "
                    f"{frame.motor_pwm:>12d}  "
                    f"{match}"
                )
                frame_count += 1

    except KeyboardInterrupt:
        pass

    print(
        f"\n[Step 3] {frame_count} frames. Mismatches: {mismatch_count} "
        f"({'perfect!' if mismatch_count == 0 else 'check your zone boundaries'})"
    )


# =============================================================================
# main
# =============================================================================

def main() -> None:
    args = parse_args()

    try:
        distance_to_pwm(50.0)
        student_ready = True
    except NotImplementedError:
        student_ready = False
        print(
            "[Step 3] distance_to_pwm() is not yet implemented.\n"
            "         Showing reference plot for reference.\n"
            "         Implement the function, then re-run."
        )
        distances = np.linspace(0, 130, 1000)
        ref_pwm = [_reference_distance_to_pwm(float(d)) for d in distances]
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.step(distances, ref_pwm, where="post", color="gray", linewidth=2.5,
                linestyle="--", label="Reference (target)")
        ax.set_xlabel("Distance (cm)")
        ax.set_ylabel("Motor PWM (0–255)")
        ax.set_title("Step 3 — Target PWM curve (implement distance_to_pwm to match)")
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.4)
        plt.tight_layout()
        plt.show()
        return

    plot_pwm_curve(use_student=True)

    if not args.simulate:
        reader = make_reader(port=args.port, simulate=False)
        print("[Step 3] Comparing your distance_to_pwm() against live Arduino values.")
        print("         Press Ctrl+C to stop.")
        live_comparison(reader)
    else:
        print("[Step 3] --simulate: no live comparison. Connect hardware to compare.")
        plt.show()


# =============================================================================

if __name__ == "__main__":
    main()
