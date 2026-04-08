"""
step7_demo.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 7: Full System Demo  — provided, no student TODOs
──────────────────────────────────────────────────────────
A pygame 900×600 window with two panels:

LEFT (450×600) — Parking View:
  - Top-down parking spot; car rectangle moves toward wall as distance decreases
  - Large distance text overlay
  - Zone-colored background (green/yellow/orange/red)
  - Motor speed dial (4-level indicator)
  - Buzzer alarm icon (flashing red when alarm=2, pulsing orange when alarm=1)
  - LED matrix visualization (8-bar, mirroring MAX7219 hardware)

RIGHT (450×600) — Graphs:
  - Top 60%: rolling 10s distance plot with zone bands
  - Bottom 40%: bar graph of current motor PWM level

Controls: Q or ESC = quit.
argparse: --port, --simulate

Usage:
    python step7_demo.py --simulate      # no hardware needed
    python step7_demo.py                 # live hardware
"""

import argparse
import collections
import math
import sys
import threading
import time
from typing import Deque, List, Optional, Tuple

import pygame

from sensor_reader import ParkingFrame, add_args, make_reader

# ─── Window & Layout ──────────────────────────────────────────────────────────
WIN_W, WIN_H   = 900, 600
LEFT_W         = 450
RIGHT_W        = 450
FPS            = 30
BUFFER_SIZE    = 100   # rolling 10 s at 10 Hz

# ─── Zone colors (R, G, B) ────────────────────────────────────────────────────
ZONE_COLORS = {
    0: (80, 180, 80),    # green  — Full speed
    1: (200, 200, 50),   # yellow — 3/4 speed
    2: (220, 140, 40),   # orange — Half speed
    3: (200, 60, 60),    # red    — Stop
}
ALARM_COLORS = {
    0: (60, 60, 60),     # dark grey — silent
    1: (220, 140, 40),   # orange    — beeping
    2: (220, 40, 40),    # bright red — continuous
}

# ─── Graph zone band colors (as fractions 0–1 for distance 0–130 cm) ─────────
GRAPH_BANDS = [
    (0,  25,  (220, 100, 100)),
    (25, 50,  (220, 160, 80)),
    (50, 75,  (220, 220, 80)),
    (75, 130, (100, 200, 100)),
]


# =============================================================================
# DataBuffer
# =============================================================================

class DataBuffer:
    """Thread-safe circular buffer for pygame rendering."""

    def __init__(self, maxlen: int = BUFFER_SIZE) -> None:
        self.distance:    Deque[float] = collections.deque(maxlen=maxlen)
        self.motor_pwm:   Deque[int]   = collections.deque(maxlen=maxlen)
        self.alarm_state: Deque[int]   = collections.deque(maxlen=maxlen)
        self.zone_id:     Deque[int]   = collections.deque(maxlen=maxlen)
        self._latest: Optional[ParkingFrame] = None
        self._lock = threading.Lock()

    def push(self, frame: ParkingFrame) -> None:
        with self._lock:
            self.distance.append(frame.distance_cm)
            self.motor_pwm.append(frame.motor_pwm)
            self.alarm_state.append(frame.alarm_state)
            self.zone_id.append(frame.zone_id)
            self._latest = frame

    def latest(self) -> Optional[ParkingFrame]:
        with self._lock:
            return self._latest


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
# Drawing helpers
# =============================================================================

def draw_left_panel(
    surface: pygame.Surface,
    frame: Optional[ParkingFrame],
    t: float,
) -> None:
    """Draw the parking view panel (left side)."""
    surf = pygame.Surface((LEFT_W, WIN_H))

    zone_id = frame.zone_id if frame else 0
    bg_color = ZONE_COLORS.get(zone_id, (80, 80, 80))
    surf.fill(bg_color)

    # ── Parking spot lines ────────────────────────────────────────────────────
    pygame.draw.line(surf, (255, 255, 255), (60, 50), (60, WIN_H - 80), 3)
    pygame.draw.line(surf, (255, 255, 255), (LEFT_W - 60, 50), (LEFT_W - 60, WIN_H - 80), 3)

    # ── Wall at bottom ────────────────────────────────────────────────────────
    pygame.draw.rect(surf, (100, 100, 120), (20, WIN_H - 80, LEFT_W - 40, 30))

    # ── Car rectangle (moves toward wall as distance decreases) ───────────────
    dist_cm = frame.distance_cm if frame else 80.0
    dist_clamped = max(0.0, min(100.0, dist_cm))
    # y=50 when far, y=WIN_H-160 when close
    car_y = int(50 + (100.0 - dist_clamped) / 100.0 * (WIN_H - 210))
    car_rect = pygame.Rect(LEFT_W // 2 - 50, car_y, 100, 60)
    pygame.draw.rect(surf, (40, 80, 180), car_rect, border_radius=8)
    pygame.draw.rect(surf, (200, 200, 255), car_rect, 2, border_radius=8)

    # ── Distance text ─────────────────────────────────────────────────────────
    font_large = pygame.font.SysFont("monospace", 36, bold=True)
    font_small = pygame.font.SysFont("monospace", 18)

    dist_str = f"{dist_cm:.1f} cm" if frame else "-- cm"
    txt = font_large.render(dist_str, True, (255, 255, 255))
    surf.blit(txt, (LEFT_W // 2 - txt.get_width() // 2, 10))

    # ── Zone label ────────────────────────────────────────────────────────────
    zone_labels = {0: "FULL SPEED", 1: "3/4 SPEED", 2: "HALF SPEED", 3: "STOP"}
    zone_str = zone_labels.get(zone_id, "—")
    zt = font_small.render(zone_str, True, (255, 255, 255))
    surf.blit(zt, (LEFT_W // 2 - zt.get_width() // 2, 52))

    # ── Motor speed dial (4-level bar) ────────────────────────────────────────
    pwm = frame.motor_pwm if frame else 255
    speed_levels = [0, 128, 191, 255]
    dial_x, dial_y = 20, WIN_H - 200
    dt = font_small.render("Motor:", True, (240, 240, 240))
    surf.blit(dt, (dial_x, dial_y - 22))
    for i, lvl in enumerate(speed_levels):
        color = (80, 200, 80) if pwm >= lvl else (60, 60, 60)
        pygame.draw.rect(surf, color, (dial_x + i * 28, dial_y, 22, 18), border_radius=3)

    # ── Alarm icon ────────────────────────────────────────────────────────────
    alarm = frame.alarm_state if frame else 0
    alarm_x, alarm_y = LEFT_W - 90, WIN_H - 200
    at = font_small.render("Alarm:", True, (240, 240, 240))
    surf.blit(at, (alarm_x, alarm_y - 22))

    flash_on = (int(t * 4) % 2 == 0)
    if alarm == 0:
        icon_color = (60, 60, 60)
    elif alarm == 1:
        icon_color = (220, 140, 40) if flash_on else (80, 60, 20)
    else:
        icon_color = (220, 40, 40) if flash_on else (80, 20, 20)

    pygame.draw.circle(surf, icon_color, (alarm_x + 30, alarm_y + 10), 18)
    pygame.draw.circle(surf, (220, 220, 220), (alarm_x + 30, alarm_y + 10), 18, 2)

    # ── LED matrix visualization (8 bars) ─────────────────────────────────────
    mat_x, mat_y = LEFT_W // 2 - 52, WIN_H - 150
    rows_lit = int(round((1.0 - min(dist_clamped, 100.0) / 100.0) * 8))
    for row in range(8):
        lit = row < rows_lit
        r_color = (220, 60, 60) if (row < 3 and lit) else (60, 220, 60) if lit else (30, 30, 30)
        for col in range(8):
            pygame.draw.rect(
                surf, r_color,
                (mat_x + col * 13, mat_y + row * 13, 11, 11),
                border_radius=2,
            )
    led_label = font_small.render("LED Matrix", True, (200, 200, 200))
    surf.blit(led_label, (mat_x, mat_y - 20))

    surface.blit(surf, (0, 0))


def draw_right_panel(
    surface: pygame.Surface,
    buf: DataBuffer,
    frame: Optional[ParkingFrame],
) -> None:
    """Draw the graphs panel (right side)."""
    surf = pygame.Surface((RIGHT_W, WIN_H))
    surf.fill((30, 30, 35))

    font_med   = pygame.font.SysFont("monospace", 16)
    font_small = pygame.font.SysFont("monospace", 13)

    # ── Top 60%: distance graph ────────────────────────────────────────────────
    graph_h = int(WIN_H * 0.6)
    graph_rect = pygame.Rect(30, 20, RIGHT_W - 50, graph_h - 30)

    # Zone bands
    for lo, hi, color in GRAPH_BANDS:
        band_y_hi = graph_rect.top + int((1 - min(hi, 130) / 130) * graph_rect.height)
        band_y_lo = graph_rect.top + int((1 - lo / 130) * graph_rect.height)
        pygame.draw.rect(surf, color,
                         (graph_rect.left, band_y_hi,
                          graph_rect.width, band_y_lo - band_y_hi), 0)

    pygame.draw.rect(surf, (100, 100, 120), graph_rect, 1)
    title = font_med.render("Distance (10 s rolling)", True, (200, 200, 200))
    surf.blit(title, (30, 5))

    # Y-axis labels
    for cm_label in [0, 25, 50, 75, 100, 130]:
        y = graph_rect.top + int((1 - cm_label / 130) * graph_rect.height)
        pygame.draw.line(surf, (70, 70, 80), (graph_rect.left, y), (graph_rect.right, y), 1)
        lbl = font_small.render(f"{cm_label}", True, (140, 140, 140))
        surf.blit(lbl, (graph_rect.left - 28, y - 7))

    # Plot line
    data = list(buf.distance)
    if len(data) > 1:
        pts = []
        for i, d in enumerate(data):
            x = graph_rect.left + int(i / (BUFFER_SIZE - 1) * graph_rect.width)
            y = graph_rect.top + int((1 - min(d, 130) / 130) * graph_rect.height)
            pts.append((x, y))
        pygame.draw.lines(surf, (100, 180, 255), False, pts, 2)

    # ── Bottom 40%: PWM bar ────────────────────────────────────────────────────
    bar_y_start = graph_h + 10
    bar_h = WIN_H - bar_y_start - 20
    bar_title = font_med.render("Motor PWM", True, (200, 200, 200))
    surf.blit(bar_title, (30, bar_y_start - 2))

    pwm = frame.motor_pwm if frame else 0
    bar_max_w = RIGHT_W - 80
    bar_fill = int(pwm / 255 * bar_max_w)

    pwm_colors = {255: (60, 200, 60), 191: (160, 200, 60), 128: (200, 160, 40), 0: (200, 60, 60)}
    bar_color = pwm_colors.get(pwm, (100, 100, 200))

    bar_rect = pygame.Rect(40, bar_y_start + 20, bar_max_w, 40)
    pygame.draw.rect(surf, (50, 50, 60), bar_rect, border_radius=6)
    if bar_fill > 0:
        pygame.draw.rect(surf, bar_color,
                         (40, bar_y_start + 20, bar_fill, 40), border_radius=6)
    pygame.draw.rect(surf, (140, 140, 160), bar_rect, 1, border_radius=6)

    pwm_str = f"PWM = {pwm}/255"
    pt = font_med.render(pwm_str, True, (220, 220, 220))
    surf.blit(pt, (40, bar_y_start + 68))

    surface.blit(surf, (LEFT_W, 0))


# =============================================================================
# parse_args
# =============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Step 7 — Full system pygame demo."
    )
    add_args(parser)
    return parser.parse_args()


# =============================================================================
# main
# =============================================================================

def main() -> None:
    """Entry point for Step 7."""
    args = parse_args()

    reader = make_reader(port=args.port, simulate=args.simulate)
    reader.connect()

    buf = DataBuffer()
    stop_event = threading.Event()

    reader_thread = threading.Thread(
        target=serial_reader_thread,
        args=(reader, buf, stop_event),
        daemon=True,
    )
    reader_thread.start()

    # ── pygame init ───────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Lab 3 — Smart Parking Sensor Demo")
    clock = pygame.time.Clock()

    print("[Step 7] Demo running. Press Q or ESC to quit.")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

        t = time.time()
        frame = buf.latest()

        screen.fill((20, 20, 25))
        draw_left_panel(screen, frame, t)
        draw_right_panel(screen, buf, frame)

        # Divider line
        pygame.draw.line(screen, (80, 80, 100), (LEFT_W, 0), (LEFT_W, WIN_H), 2)

        pygame.display.flip()
        clock.tick(FPS)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    stop_event.set()
    pygame.quit()
    reader.close()
    print("[Step 7] Done.")


# =============================================================================

if __name__ == "__main__":
    main()
