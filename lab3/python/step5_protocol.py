"""
step5_protocol.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 5: Serial Protocol Framing  *** EECE 5520 ONLY ***
──────────────────────────────────────────────────────────
Student Task (5520 only):
    Design and implement a binary framing protocol by completing
    encode_frame() and decode_frame().

Why binary framing?
    CSV text ("47.3,128,1,2\\n") is human-readable but fragile:
      - A missing newline corrupts the entire packet
      - No way to detect partial or corrupted data
      - Newline could appear inside a text value

    A binary framing protocol solves these problems:
      - A start byte lets the receiver find frame boundaries
      - Fixed frame length means we know exactly where each frame ends
      - A checksum lets us detect and reject corrupted frames

Your protocol must:
    1. Begin with a start byte so the receiver can find frame boundaries
    2. Encode distance_cm with enough precision
       (Hint: integer cm×10 fits in a uint16 — no floats in binary protocols)
    3. Include motor_pwm, alarm_state, zone_id
    4. End with a checksum so corrupted frames can be rejected

Document your design choices in your lab report:
    - What start byte did you choose and why?
    - What checksum algorithm? What are the trade-offs?
    - What happens if the start byte value appears in the payload?
    - How would you make the protocol more robust? (e.g. escape sequences, CRC)

Run this script to test your implementation:
    python step5_protocol.py

Tests:
    - Roundtrip: encode → decode 1000 random values, verify correctness
    - Corruption: flip random bits, verify decode_frame returns None
"""

import argparse
import random
import time
from typing import Optional

from sensor_reader import ParkingFrame, add_args

# ─── Protocol constants — you may change FRAME_START (document your choice) ──
FRAME_START: int = 0xAA   # Provided: start byte (you may change this value)
FRAME_LEN: int   = 7      # Provided: total frame length in bytes


# =============================================================================
# encode_frame  ← STUDENTS IMPLEMENT (5520 only)
# =============================================================================

def encode_frame(
    distance_cm: float,
    motor_pwm: int,
    alarm_state: int,
    zone_id: int,
) -> bytes:
    """
    Encode sensor data into a fixed-length binary frame.

    TODO (5520): Design and implement a binary framing protocol.

    Your protocol must:
    - Begin with a start byte so the receiver can find frame boundaries
    - Encode distance_cm with enough precision
      (Hint: integer cm×10 fits in uint16 — big-endian gives two bytes)
    - Include motor_pwm, alarm_state, zone_id as individual bytes
    - End with a checksum so corrupted frames can be rejected

    The encoded bytes object must be exactly FRAME_LEN (7) bytes long.

    Document your design choices in your lab report:
    - What start byte did you choose and why?
    - What checksum algorithm? (Sum mod 256? Something else?)
    - What happens if the start byte appears in the payload?

    Args:
        distance_cm:  Distance in cm (float; encode with enough precision).
        motor_pwm:    Motor PWM 0–255 (int).
        alarm_state:  Alarm state 0/1/2 (int).
        zone_id:      Zone 0–3 (int).

    Returns:
        A bytes object of exactly FRAME_LEN bytes.
    """
    raise NotImplementedError("Design and implement your framing protocol.")


# =============================================================================
# decode_frame  ← STUDENTS IMPLEMENT (5520 only)
# =============================================================================

def decode_frame(data: bytes) -> Optional[ParkingFrame]:
    """
    Decode a binary frame produced by encode_frame().

    TODO (5520): Implement the decoder to match your encoder.

    Your decoder must:
    - Verify the frame length equals FRAME_LEN
    - Verify the start byte matches FRAME_START
    - Verify the checksum (return None if invalid)
    - Extract and return all fields as a ParkingFrame

    Args:
        data: A bytes object to decode (may be incomplete or corrupt).

    Returns:
        A ParkingFrame if the frame is valid, None if invalid or checksum fails.
    """
    raise NotImplementedError("Implement the decoder to match your encoder.")


# =============================================================================
# Test harness — provided, do not modify
# =============================================================================

def run_roundtrip_test(n: int = 1000) -> None:
    """
    Test that encode_frame → decode_frame roundtrips correctly for n random values.
    """
    print(f"\n[Step 5] Roundtrip test ({n} random frames) ...")
    passed = 0
    failed = 0

    for _ in range(n):
        distance_cm  = round(random.uniform(0.0, 120.0), 1)
        motor_pwm    = random.choice([0, 128, 191, 255])
        alarm_state  = random.randint(0, 2)
        zone_id      = random.randint(0, 3)

        try:
            encoded = encode_frame(distance_cm, motor_pwm, alarm_state, zone_id)
        except NotImplementedError:
            print("[Step 5] encode_frame() not implemented yet.")
            return

        if len(encoded) != FRAME_LEN:
            failed += 1
            continue

        try:
            decoded = decode_frame(encoded)
        except NotImplementedError:
            print("[Step 5] decode_frame() not implemented yet.")
            return

        if decoded is None:
            failed += 1
            continue

        if (
            abs(decoded.distance_cm - distance_cm) > 0.05
            or decoded.motor_pwm != motor_pwm
            or decoded.alarm_state != alarm_state
            or decoded.zone_id != zone_id
        ):
            failed += 1
        else:
            passed += 1

    status = "✓ PASS" if failed == 0 else f"✗ FAIL ({failed} errors)"
    print(f"[Step 5] Roundtrip {status}: {passed}/{n}")


def run_corruption_test(n: int = 1000) -> None:
    """
    Test that decode_frame rejects corrupted frames.
    """
    print(f"\n[Step 5] Corruption test ({n} frames with random bit flips) ...")

    try:
        base = encode_frame(47.3, 128, 1, 2)
    except NotImplementedError:
        print("[Step 5] encode_frame() not implemented yet.")
        return

    rejected = 0
    for _ in range(n):
        corrupted = bytearray(base)
        byte_idx = random.randint(1, FRAME_LEN - 1)
        bit_mask = random.randint(1, 255)
        corrupted[byte_idx] ^= bit_mask

        try:
            result = decode_frame(bytes(corrupted))
        except NotImplementedError:
            print("[Step 5] decode_frame() not implemented yet.")
            return

        if result is None:
            rejected += 1

    pct = 100 * rejected / n
    status = "✓ PASS" if pct > 95 else f"✗ FAIL (only {pct:.1f}% rejected)"
    print(f"[Step 5] Corruption {status}: {rejected}/{n} rejected (expected: ~100%)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 5 (5520 only) — Binary protocol encode/decode tests."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================
# main
# =============================================================================

def main() -> None:
    _args = parse_args()

    print("=" * 58)
    print("Step 5 — Binary Framing Protocol Test Harness")
    print("EECE 5520 only: implement encode_frame() and decode_frame()")
    print("=" * 58)

    run_roundtrip_test(1000)
    run_corruption_test(1000)

    print("\n[Step 5] Done. Check results above.")
    print("         Document your protocol design in your lab report.")


# =============================================================================

if __name__ == "__main__":
    main()
