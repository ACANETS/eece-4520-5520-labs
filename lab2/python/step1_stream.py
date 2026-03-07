"""
step1_stream.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 1: Raw Data Streaming & Logging
─────────────────────────────────────
Connects to the Arduino (or simulator), prints a formatted live table of IMU
values to the terminal, and simultaneously logs every packet to a CSV file.

Usage:
    python step1_stream.py                    # auto-detect serial port
    python step1_stream.py --port /dev/ttyUSB0
    python step1_stream.py --simulate         # no hardware needed

Press Ctrl+C to stop.

Output file:
    imu_log.csv   (created in the current working directory)
"""

import argparse
import csv
import sys
import time
from pathlib import Path

from imu_reader import make_reader

# ─── Constants ────────────────────────────────────────────────────────────────
OUTPUT_CSV: str = "imu_log.csv"
CSV_HEADER: list = ["ts_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]
PRINT_EVERY_N: int = 1   # print every Nth packet (1 = every packet at 50 Hz)


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 1 — Stream and log raw IMU data from the Arduino."
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (e.g. /dev/tty.usbmodem1101 or COM3). "
             "Auto-detected if not specified.",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Use synthetic IMU data instead of real hardware.",
    )
    return parser.parse_args()


# =============================================================================
# print_header / print_packet
# =============================================================================

def print_header() -> None:
    """Print the column header for the terminal table."""
    print(
        "\n"
        f"{'Time':>10s}  "
        f"{'ax':>10s}  {'ay':>10s}  {'az':>10s}  "
        f"{'gx':>10s}  {'gy':>10s}  {'gz':>10s}"
    )
    print("─" * 78)


def print_packet(pkt: dict) -> None:
    """
    Print one IMU packet as a formatted table row.

    Args:
        pkt: Dict with keys ts, ax, ay, az, gx, gy, gz.
    """
    print(
        f"[{pkt['ts']:>8.3f}s]  "
        f"ax={pkt['ax']:>7.4f}g  "
        f"ay={pkt['ay']:>7.4f}g  "
        f"az={pkt['az']:>7.4f}g  |  "
        f"gx={pkt['gx']:>7.2f}°/s  "
        f"gy={pkt['gy']:>7.2f}°/s  "
        f"gz={pkt['gz']:>7.2f}°/s"
    )


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 1."""
    args = parse_args()

    # Create / open CSV log file
    csv_path = Path(OUTPUT_CSV)
    csv_file = open(csv_path, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADER)
    writer.writeheader()
    print(f"[Step 1] Logging to: {csv_path.resolve()}")

    # Connect to sensor (real or simulated)
    reader = make_reader(port=args.port, simulate=args.simulate)
    reader.connect()

    print_header()
    packet_count = 0
    try:
        while True:
            pkt = reader.read_packet()
            if pkt is None:
                time.sleep(0.001)
                continue

            # Log to CSV (every packet)
            writer.writerow({
                "ts_s":   pkt["ts"],
                "ax_g":   pkt["ax"],
                "ay_g":   pkt["ay"],
                "az_g":   pkt["az"],
                "gx_dps": pkt["gx"],
                "gy_dps": pkt["gy"],
                "gz_dps": pkt["gz"],
            })

            # Print to terminal (every PRINT_EVERY_N packets)
            packet_count += 1
            if packet_count % PRINT_EVERY_N == 0:
                print_packet(pkt)

    except KeyboardInterrupt:
        print(f"\n[Step 1] Stopped. {packet_count} packets logged to {OUTPUT_CSV}.")

    finally:
        csv_file.flush()
        csv_file.close()
        reader.close()


# =============================================================================

if __name__ == "__main__":
    main()
