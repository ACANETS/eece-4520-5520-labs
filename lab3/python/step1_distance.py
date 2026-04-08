"""
step1_distance.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 1: Distance Sensing — Live Terminal Table
───────────────────────────────────────────────
Connects to the Arduino (or simulator), reads ParkingFrames, and prints a
formatted live terminal table showing distance, zone, motor speed, and alarm
state at 10 Hz.

Usage:
    python step1_distance.py                    # auto-detect serial port
    python step1_distance.py --port /dev/ttyUSB0
    python step1_distance.py --simulate         # no hardware needed

Press Ctrl+C to stop.

Background — HC-SR04 Pulse Timing:
    The HC-SR04 measures distance using ultrasound:
      1. Arduino pulls TRIG HIGH for 10 µs → sensor fires 8× 40 kHz pulses
      2. ECHO pin goes HIGH when pulses leave the sensor
      3. ECHO goes LOW when reflected pulses return
      4. echo_duration_µs = 2 × distance / speed_of_sound
         ⟹  distance_cm = echo_duration × 0.034 / 2
    Timeout at 30 000 µs prevents blocking when no obstacle is present.
"""

import argparse
import time

from sensor_reader import ALARM_NAMES, ZONE_NAMES, add_args, make_reader


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 1 — Live distance table from the parking sensor."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================
# print_header / print_frame
# =============================================================================

def print_header() -> None:
    """Print the column header for the terminal table."""
    print(
        "\n"
        f"{'Distance':>12s}  "
        f"{'Zone':>14s}  "
        f"{'Motor':>12s}  "
        f"{'Alarm':>12s}"
    )
    print("─" * 58)


def print_frame(frame) -> None:
    """
    Print one ParkingFrame as a formatted table row.

    Args:
        frame: A ParkingFrame instance.
    """
    zone_name  = ZONE_NAMES.get(frame.zone_id, "Unknown")
    alarm_name = ALARM_NAMES.get(frame.alarm_state, "Unknown")
    motor_str  = f"{frame.motor_pwm}/255"

    print(
        f"Distance: {frame.distance_cm:>6.1f} cm  "
        f"Zone: {zone_name:<14s}  "
        f"Motor: {motor_str:>8s}  "
        f"Alarm: {alarm_name}"
    )


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 1."""
    args = parse_args()

    reader = make_reader(port=args.port, simulate=args.simulate)

    print_header()
    frame_count = 0

    try:
        with reader:
            while True:
                frame = reader.read()
                if frame is None:
                    time.sleep(0.001)
                    continue

                frame_count += 1
                print_frame(frame)

    except KeyboardInterrupt:
        print(f"\n[Step 1] Stopped. {frame_count} frames received.")


# =============================================================================

if __name__ == "__main__":
    main()
