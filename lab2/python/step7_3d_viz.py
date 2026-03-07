"""
step7_3d_viz.py
EECE 4520/5520 — Microprocessor II and Embedded System Design
Spring 2026, UMass Lowell

Step 7 (EECE 5520 only): 3D Orientation Visualiser + Marble Madness Game
──────────────────────────────────────────────────────────────────────────
A single pygame window with two panels:

  LEFT  panel — Real-time 3-D wireframe of the MPU-6050 board.
                Uses numpy rotation matrices + perspective projection.
                The board rotates to match the physical sensor orientation.

  RIGHT panel — "Marble Madness": tilt the board to roll a marble across
                a platform, collect glowing gems (+10 pts each), and avoid
                the dark holes.

Both panels share the same IMU data stream and the same complementary filter.

Controls:
  Tilt board  → roll the marble (and rotate the 3-D model)
  R           → restart game
  ESC         → quit

Run:
    python step7_3d_viz.py --simulate          # no hardware needed
    python step7_3d_viz.py                     # auto-detect port
    python step7_3d_viz.py --port /dev/ttyUSB0

Dependencies (already in requirements.txt):
    pip install pyserial numpy pygame
    (vpython is NOT required for this step)
"""

import argparse
import math
import random
import threading
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import pygame

from imu_reader import make_reader

# ─── Complementary filter ─────────────────────────────────────────────────────
ALPHA: float = 0.98

# ─── Shake detection ──────────────────────────────────────────────────────────
SHAKE_THRESHOLD: float = 0.45   # |accel_magnitude - 1 g| to count as a shake sample
SHAKE_CONFIRM: int     = 6      # consecutive samples above threshold → shake confirmed

# ─── Window layout ────────────────────────────────────────────────────────────
VIZ_W: int  = 560          # width of the 3-D panel (px)
GAME_W: int = 660          # width of the game panel (px)
WIN_H: int  = 720          # total window height (px)
WIN_W: int  = VIZ_W + GAME_W

HUD_H: int  = 46           # HUD strip at the top of the game panel
BOARD_M: int = 28          # margin inside the game board
FPS: int    = 60

# ─── 3-D wireframe constants ──────────────────────────────────────────────────
# MPU-6050 coordinate frame (Z-up convention):
#   X = one horizontal axis on the PCB surface
#   Y = other horizontal axis on the PCB surface
#   Z = surface normal, pointing UP from the chip (reads +1 g when flat)
#
# The board model therefore lies in the XY plane with Z pointing up.
#   PCB_W × PCB_L = the flat face;  PCB_T = thickness along Z
#
# Rotation order (standard ZYX Euler, Z-up):
#   roll  → Rx  (tips one horizontal edge up/down)
#   pitch → Ry  (tips the other horizontal edge up/down)
#   yaw   → Rz  (spins the board around the vertical Z axis)
PCB_W: float = 3.8      # width  along X (units)
PCB_L: float = 5.6      # length along Y (units)
PCB_T: float = 0.28     # thickness along Z (units)

FOV: float   = 260.0    # perspective field-of-view factor (px)
CAM_Z: float = 10.0     # camera distance (depth offset for perspective)

# Camera view angles — applied as a fixed pre-rotation so we see the board
# from above and slightly to the side, as if looking down at a PCB on a desk.
# VIEW_PITCH is the gaze angle below horizontal (negative = look down).
# VIEW_YAW rotates the scene left/right to add depth perspective.
VIEW_PITCH: float = -30.0   # 30° below horizontal — natural "looking down at a board" angle
VIEW_YAW:   float =  20.0   # slight rotation so depth/edges are visible

# Sign tuning — flip to -1 if a rotation appears visually inverted on your
# specific sensor mounting. Adjust ROLL_SIGN first, then PITCH_SIGN.
ROLL_SIGN:  int = 1    # roll  around X  (tips left/right horizontal edge)
PITCH_SIGN: int = 1    # pitch around Y  (tips front/back horizontal edge)
YAW_SIGN:   int = 1    # yaw   around Z  (spins around vertical axis)

# ─── Marble physics ───────────────────────────────────────────────────────────
TILT_ACCEL: float = 5.0    # px / s² per degree of tilt
FRICTION: float   = 0.91   # velocity multiplier per frame
BALL_R: int       = 14

# ─── Game objects ─────────────────────────────────────────────────────────────
N_GEMS:  int = 6
N_HOLES: int = 3
GEM_R:   int = 11
HOLE_R:  int = 22
LIVES:   int = 3

# ─── Colours ─────────────────────────────────────────────────────────────────
C_BG_VIZ    = (12,  18,  28)
C_BG_GAME   = (14,  20,  14)
C_DIVIDER   = (60,  70,  80)
C_PCB_EDGE  = ( 80, 200,  80)
C_PCB_FACE  = ( 30,  90,  40)
C_AXIS_X    = (220,  60,  60)
C_AXIS_Y    = ( 60, 210,  60)
C_AXIS_Z    = ( 60, 160, 230)
C_TEXT_VIZ  = (190, 210, 230)
C_BOARD     = ( 38,  62,  38)
C_GRID      = ( 48,  72,  48)
C_BORDER    = ( 75, 105,  75)
C_BALL      = (230, 230, 230)
C_BALL_HI   = (255, 255, 255)
C_GEM       = ( 55, 210, 240)
C_GEM_HI    = (155, 238, 255)
C_HOLE      = ( 10,  10,  10)
C_HOLE_RIM  = ( 38,  38,  38)
C_HUD_BG    = ( 18,  18,  18)
C_TEXT      = (228, 228, 208)
C_LIFE      = (200,  65,  65)
C_GEM_GOLD  = (255, 200,  30)    # gem colour when double-points is active
C_GEM_GOLD_HI = (255, 240, 120)
C_DOUBLE_TXT  = (255, 210,  40)  # "2× POINTS!" HUD text


# =============================================================================
# Shared state  (IMU thread → main loop)
# =============================================================================

@dataclass
class SharedState:
    """Thread-safe complementary-filter output."""
    lock:    threading.Lock = field(default_factory=threading.Lock)
    roll:          float = 0.0   # degrees; + = right-side-down
    pitch:         float = 0.0   # degrees; + = front-up
    yaw:           float = 0.0   # degrees; gyro-integrated (drifts)
    double_points: bool  = False # True after shake detected; cleared on first gem
    running:       bool  = True


# =============================================================================
# IMU worker thread
# =============================================================================

def imu_worker(reader, state: SharedState) -> None:
    """
    Background thread: reads packets, applies complementary filter, writes SharedState.

    Complementary filter (same as Step 5):
        roll  = α*(roll  + gx*dt) + (1-α)*accel_roll
        pitch = α*(pitch + gy*dt) + (1-α)*accel_pitch
        yaw  += gz*dt   (gyro-only — drifts without a magnetometer)

    Shake detection:
        Accel magnitude at rest ≈ 1 g (gravity only).  During shaking the
        magnitude spikes well above or below 1 g.  We count consecutive
        samples where |mag - 1| > SHAKE_THRESHOLD; once SHAKE_CONFIRM samples
        accumulate, we set state.double_points = True.  The flag stays set
        until the game clears it after the next gem is collected.
    """
    roll = pitch = yaw = 0.0
    prev_ts: Optional[float] = None
    shake_counter: int = 0          # consecutive above-threshold samples

    while state.running:
        pkt = reader.read_packet()
        if pkt is None:
            continue

        ts = pkt["ts"]
        ax, ay, az = pkt["ax"], pkt["ay"], pkt["az"]
        gx, gy, gz = pkt["gx"], pkt["gy"], pkt["gz"]

        a_roll  = math.degrees(math.atan2(ay, az))
        a_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        if prev_ts is None:
            roll, pitch = a_roll, a_pitch
        else:
            dt = max(0.001, min(ts - prev_ts, 0.1))
            roll  = ALPHA * (roll  + gx * dt) + (1.0 - ALPHA) * a_roll
            pitch = ALPHA * (pitch + gy * dt) + (1.0 - ALPHA) * a_pitch
            yaw  += gz * dt

        prev_ts = ts

        # ── Shake detection ───────────────────────────────────────────────
        mag = math.sqrt(ax**2 + ay**2 + az**2)
        if abs(mag - 1.0) > SHAKE_THRESHOLD:
            shake_counter += 1
        else:
            shake_counter = 0

        with state.lock:
            state.roll, state.pitch, state.yaw = roll, pitch, yaw
            if shake_counter >= SHAKE_CONFIRM and not state.double_points:
                state.double_points = True
                shake_counter = 0   # reset so it doesn't re-trigger immediately
                print("[Shake!] Double points activated — collect a gem for 2× score!")


# =============================================================================
# 3-D wireframe renderer
# =============================================================================

# PCB box: 8 vertices in world units — board lies in XY plane, Z = up
# Vertex order: bottom face (z = -PCB_T/2) then top face (z = +PCB_T/2)
_hx, _hy, _hz = PCB_W / 2, PCB_L / 2, PCB_T / 2
PCB_VERTS = np.array([
    [-_hx, -_hy, -_hz], [ _hx, -_hy, -_hz], [ _hx,  _hy, -_hz], [-_hx,  _hy, -_hz],  # bottom
    [-_hx, -_hy,  _hz], [ _hx, -_hy,  _hz], [ _hx,  _hy,  _hz], [-_hx,  _hy,  _hz],  # top
], dtype=float)

# 12 edges
PCB_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),   # bottom face
    (4, 5), (5, 6), (6, 7), (7, 4),   # top face
    (0, 4), (1, 5), (2, 6), (3, 7),   # vertical edges
]

# 6 faces for depth-sorted filled quads (painter's algorithm)
PCB_FACES = [
    ([0, 1, 2, 3], C_PCB_FACE),   # bottom  (-Z)
    ([4, 5, 6, 7], C_PCB_FACE),   # top     (+Z)  ← PCB surface facing up
    ([0, 1, 5, 4], C_PCB_FACE),   # front   (-Y)
    ([2, 3, 7, 6], C_PCB_FACE),   # back    (+Y)
    ([0, 3, 7, 4], C_PCB_FACE),   # left    (-X)
    ([1, 2, 6, 5], C_PCB_FACE),   # right   (+X)
]

# Body-frame axis arrows: X=roll axis, Y=pitch axis, Z=yaw/up axis
AXES = [
    (np.array([0, 0, 0]), np.array([3.2, 0, 0]), C_AXIS_X, "X (roll)"),
    (np.array([0, 0, 0]), np.array([0, 3.2, 0]), C_AXIS_Y, "Y (pitch)"),
    (np.array([0, 0, 0]), np.array([0, 0, 3.2]), C_AXIS_Z, "Z (yaw/up)"),
]

# Camera view matrix — rotates the whole scene so we see the board
# from above-and-to-the-side.  Rebuilt each frame when the user drags.
def _make_view_matrix(pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Build the camera view matrix from pitch and yaw angles.

    Args:
        pitch_deg: Gaze angle below horizontal (negative = look down).
                   Clamped to [-85°, -5°] to prevent flipping.
        yaw_deg:   Left/right orbit angle (unclamped, wraps naturally).

    Returns:
        3×3 rotation matrix applied to all world-frame points before projection.
    """
    vp = math.radians(pitch_deg)
    vy = math.radians(yaw_deg)
    Rx = np.array([[1, 0,            0            ],
                   [0, math.cos(vp), -math.sin(vp)],
                   [0, math.sin(vp),  math.cos(vp)]])
    Ry = np.array([[ math.cos(vy), 0, math.sin(vy)],
                   [0,             1, 0            ],
                   [-math.sin(vy), 0, math.cos(vy)]])
    return Ry @ Rx


def _rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Build a 3×3 rotation matrix from Euler angles (degrees).

    The PCB model lies flat in the XY plane (PCB_W along X, PCB_L along Y,
    PCB_T along Z — so Z is the board normal pointing up when the board is flat).
    This matches the MPU-6050 body frame (Z = +1 g when the chip faces up).

    Rotation convention (ZYX Euler, applied innermost-first):
        roll  → Rx  (rotate around X-axis → one horizontal edge tips up/down)
        pitch → Ry  (rotate around Y-axis → the other horizontal edge tips up/down)
        yaw   → Rz  (rotate around Z-axis → board spins around vertical axis)

    Applied order: R = Rz(yaw) · Ry(pitch) · Rx(roll)
    (roll is applied first in body frame; yaw applied last in world frame)

    Sign constants ROLL_SIGN / PITCH_SIGN / YAW_SIGN at the top of the file
    let you flip individual channels if the visual appears inverted on your
    specific sensor mounting.
    """
    r = math.radians(ROLL_SIGN  * roll_deg)
    p = math.radians(PITCH_SIGN * pitch_deg)
    y = math.radians(YAW_SIGN   * yaw_deg)

    # Rx: roll  — tips board left/right (around X, one horizontal edge goes up)
    Rx = np.array([[1,            0,             0          ],
                   [0,  math.cos(r), -math.sin(r)],
                   [0,  math.sin(r),  math.cos(r)]])

    # Ry: pitch — tips board forward/back (around Y, the other horizontal edge)
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])

    # Rz: yaw   — spins board around vertical Z axis (compass heading)
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])

    # ZYX order: yaw applied last (in world frame), roll applied first (body frame)
    return Rz @ Ry @ Rx


def _project(points: np.ndarray, cx: float, cy: float) -> np.ndarray:
    """
    Perspective-project 3-D world points onto the screen.

    Uses the pinhole camera model:
        x_screen = (x_world / (z_world + cam_z)) * FOV + cx
        y_screen = -(y_world / (z_world + cam_z)) * FOV + cy   (Y flipped)

    Args:
        points: (N, 3) array of world-frame coordinates.
        cx, cy: pixel coordinates of the projection centre.

    Returns:
        (N, 2) array of screen coordinates (float).
    """
    z = points[:, 2] + CAM_Z
    z = np.where(z < 0.5, 0.5, z)           # prevent division by near-zero
    sx = (points[:, 0] / z) * FOV + cx
    sy = -(points[:, 1] / z) * FOV + cy     # flip Y: world +Y → screen up
    return np.stack([sx, sy], axis=1)


def _draw_reference_planes(surface: pygame.Surface, cx: float, cy: float,
                           font_sm: pygame.font.Font,
                           view_matrix: np.ndarray) -> None:
    """
    Draw three fixed world-frame coordinate planes as wireframe grids.

    The planes do NOT rotate — they represent the static world frame, while
    the PCB model rotates relative to them.

    Planes drawn:
        XZ (y=0)  — horizontal "floor" plane, grey
        XY (z=0)  — frontal plane, dark blue
        YZ (x=0)  — sagittal plane, dark green

    World-frame axis arrows (thin, fixed) are also drawn at the origin so
    students can distinguish the static world axes from the rotating body axes.
    """
    extent = 3.2
    steps  = np.linspace(-extent, extent, 9)  # 8 grid cells per side

    # All world-frame geometry is transformed by view_matrix only (not R)
    def _vproject(p3d):
        """Project a world-frame point through view_matrix then to screen."""
        vp = (view_matrix @ np.array(p3d)).reshape(1, 3)
        return _project(vp, cx, cy)[0].astype(int)

    def _line(p1_3d, p2_3d, colour, width=1):
        pygame.draw.line(surface, colour, tuple(_vproject(p1_3d)),
                         tuple(_vproject(p2_3d)), width)

    # ── XY plane (z = 0): the horizontal "floor" — board lies here when flat
    for v in steps:
        _line([ v, -extent, 0], [ v,  extent, 0], (55, 55, 55))  # lines || Y
        _line([-extent,  v, 0], [ extent, v,  0], (55, 55, 55))  # lines || X
    for a, b in [([-extent,-extent,0],[extent,-extent,0]),
                 ([extent,-extent,0],[extent,extent,0]),
                 ([extent,extent,0],[-extent,extent,0]),
                 ([-extent,extent,0],[-extent,-extent,0])]:
        _line(a, b, (90, 90, 90), 1)

    # ── XZ plane (y = 0): vertical plane containing X and Z (up) ── blue
    for v in steps:
        _line([ v, 0, -extent/2], [ v, 0,  extent], (25, 35, 65))  # lines || Z
        _line([-extent,  0, v/2], [ extent, 0, v/2], (25, 35, 65)) # lines || X

    # ── YZ plane (x = 0): vertical plane containing Y and Z (up) ── green
    for v in steps:
        _line([0,  v, -extent/2], [0,  v,  extent], (25, 55, 35))  # lines || Z
        _line([0, -extent, v/2],  [0,  extent, v/2], (25, 55, 35)) # lines || Y

    # ── World-frame axis arrows (fixed, thin) ────────────────────────────
    for tip_3d, colour, label in [
        ([extent, 0, 0], C_AXIS_X, "Xw"),
        ([0, extent, 0], C_AXIS_Y, "Yw"),
        ([0, 0, extent], C_AXIS_Z, "Zw↑"),
    ]:
        o2d = _vproject([0, 0, 0])
        t2d = _vproject(tip_3d)
        pygame.draw.line(surface, colour, tuple(o2d), tuple(t2d), 1)
        lbl = font_sm.render(label, True, colour)
        surface.blit(lbl, (t2d[0] + 3, t2d[1] - 7))


def draw_3d_panel(
    surface: pygame.Surface,
    font_sm: pygame.font.Font,
    roll: float, pitch: float, yaw: float,
    view_matrix: np.ndarray,
) -> None:
    """
    Draw the 3-D PCB wireframe and axis arrows onto *surface*.

    The PCB box is rotated by the current Euler angles, then projected
    with perspective onto the surface.  Faces are depth-sorted (painter's
    algorithm) so back faces render first.

    Args:
        surface:  pygame Surface for this panel (width = VIZ_W).
        font_sm:  Small font for labels and angle readout.
        roll, pitch, yaw: Current Euler angles in degrees.
    """
    surface.fill(C_BG_VIZ)

    cx, cy = VIZ_W / 2, WIN_H / 2 - 20      # projection centre

    # ── Draw world-frame reference planes first (behind everything) ──────
    _draw_reference_planes(surface, cx, cy, font_sm, view_matrix)

    R = _rotation_matrix(roll, pitch, yaw)

    # ── Rotate PCB vertices, then apply camera view matrix ───────────────
    # 1. R           — object rotation (IMU angles)
    # 2. view_matrix — camera orientation (updated by mouse drag)
    verts_r = (view_matrix @ R @ PCB_VERTS.T).T   # (8, 3)
    proj    = _project(verts_r, cx, cy)            # (8, 2) screen coords

    # ── Painter's algorithm: sort faces by average Z (furthest first) ───
    face_depths = []
    for indices, colour in PCB_FACES:
        avg_z = np.mean(verts_r[indices, 2])
        face_depths.append((avg_z, indices, colour))
    face_depths.sort(key=lambda t: t[0])      # furthest Z first

    for _, indices, colour in face_depths:
        pts = [tuple(proj[i].astype(int)) for i in indices]
        pygame.draw.polygon(surface, colour, pts)

    # ── PCB edges ────────────────────────────────────────────────────────
    for i, j in PCB_EDGES:
        p1 = tuple(proj[i].astype(int))
        p2 = tuple(proj[j].astype(int))
        pygame.draw.line(surface, C_PCB_EDGE, p1, p2, 2)

    # ── World-frame axis arrows ──────────────────────────────────────────
    for origin, direction, colour, label in AXES:
        tip   = view_matrix @ (R @ direction)          # rotate body axis, then view
        o_pts = _project((view_matrix @ origin).reshape(1, 3), cx, cy)[0]
        t_pts = _project(tip.reshape(1, 3),            cx, cy)[0]
        o2 = tuple(o_pts.astype(int))
        t2 = tuple(t_pts.astype(int))
        pygame.draw.line(surface, colour, o2, t2, 2)
        # Arrowhead (small filled triangle at tip)
        dx, dy = t2[0] - o2[0], t2[1] - o2[1]
        length = max(math.hypot(dx, dy), 1)
        ux, uy = dx / length, dy / length
        px, py = -uy * 5, ux * 5
        head = [t2,
                (int(t2[0] - ux * 12 + px), int(t2[1] - uy * 12 + py)),
                (int(t2[0] - ux * 12 - px), int(t2[1] - uy * 12 - py))]
        pygame.draw.polygon(surface, colour, head)
        lbl = font_sm.render(label, True, colour)
        surface.blit(lbl, (t2[0] + 4, t2[1] - 8))

    # ── Angle readout ────────────────────────────────────────────────────
    lines = [
        (f"Roll:  {roll:+7.1f}°  (around X)", C_AXIS_X),  # roll  → X axis (red)
        (f"Pitch: {pitch:+7.1f}°  (around Y)", C_AXIS_Y), # pitch → Y axis (green)
        (f"Yaw:   {yaw:+7.1f}°  (gyro-only)", C_AXIS_Z),  # yaw   → Z axis (blue)
    ]
    y0 = WIN_H - 80
    for i, (text, col) in enumerate(lines):
        lbl = font_sm.render(text, True, col)
        surface.blit(lbl, (12, y0 + i * 22))

    # ── Panel title + drag hint ──────────────────────────────────────────
    title = font_sm.render("3-D Orientation  (ZYX Euler)", True, C_TEXT_VIZ)
    surface.blit(title, (VIZ_W // 2 - title.get_width() // 2, 10))
    hint = font_sm.render("🖱 Drag to orbit   V = reset view", True, (100, 120, 140))
    surface.blit(hint, (VIZ_W // 2 - hint.get_width() // 2, WIN_H - 22))


# =============================================================================
# Marble Madness game
# =============================================================================

class _Vec2:
    __slots__ = ("x", "y")

    def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
        self.x = x
        self.y = y


class _Gem:
    __slots__ = ("x", "y", "collected")

    def __init__(self, x: float, y: float) -> None:
        self.x, self.y, self.collected = x, y, False


class _Hole:
    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float) -> None:
        self.x, self.y = x, y


class MarbleGame:
    """
    Top-down physics game: tilt the board to roll the marble.

    Physics
    -------
    Each frame the marble receives an acceleration proportional to tilt:
        vel.x = (vel.x + TILT_ACCEL * roll  * dt) * FRICTION
        vel.y = (vel.y + TILT_ACCEL * pitch * dt) * FRICTION

    This mirrors the real mechanics of a ball on a tilting platform — the
    same physical quantity that the accelerometer measures.

    Gameplay
    --------
    * Collect all gems (+10 pts each) → advance to next level.
    * Fall into a hole → lose a life; marble respawns at centre.
    * 3 lives; game over when all are lost.  Press R to restart.
    """

    # Board geometry (inside the game panel, below the HUD)
    _BX = BOARD_M
    _BY = HUD_H + BOARD_M
    _BW = GAME_W - 2 * BOARD_M
    _BH = WIN_H  - HUD_H - 2 * BOARD_M

    def __init__(self) -> None:
        self._rng = random.Random()
        self.reset()

    def reset(self) -> None:
        self.lives = LIVES
        self.score = 0
        self.level = 1
        self.over  = False
        self._respawn()

    # ── Object placement ───────────────────────────────────────────────────

    def _centre(self):
        return self._BX + self._BW / 2, self._BY + self._BH / 2

    def _respawn(self) -> None:
        cx, cy = self._centre()
        self.ball = _Vec2(cx, cy)
        self.vel  = _Vec2()
        self.gems  = self._place_gems()
        self.holes = self._place_holes()

    def _rand_pos(self, clear: float = 80):
        cx, cy = self._centre()
        for _ in range(600):
            x = self._rng.uniform(self._BX + 50, self._BX + self._BW - 50)
            y = self._rng.uniform(self._BY + 50, self._BY + self._BH - 50)
            if math.hypot(x - cx, y - cy) >= clear:
                return x, y
        return cx + clear, cy

    @staticmethod
    def _no_overlap(taken: list, x: float, y: float, min_d: float = 60) -> bool:
        return all(math.hypot(x - tx, y - ty) >= min_d for tx, ty in taken)

    def _place_gems(self) -> List[_Gem]:
        taken, gems = [], []
        for _ in range(N_GEMS):
            for _ in range(300):
                x, y = self._rand_pos()
                if self._no_overlap(taken, x, y):
                    taken.append((x, y))
                    gems.append(_Gem(x, y))
                    break
        return gems

    def _place_holes(self) -> List[_Hole]:
        taken = [(g.x, g.y) for g in self.gems]
        holes = []
        for _ in range(N_HOLES):
            for _ in range(300):
                x, y = self._rand_pos(clear=100)
                if self._no_overlap(taken, x, y, min_d=70):
                    taken.append((x, y))
                    holes.append(_Hole(x, y))
                    break
        return holes

    # ── Per-frame update ───────────────────────────────────────────────────

    def update(self, roll_deg: float, pitch_deg: float, dt: float,
               double_points: bool = False) -> bool:
        """
        Update marble physics and game state for one frame.

        Args:
            roll_deg:      Current roll from complementary filter (degrees).
            pitch_deg:     Current pitch from complementary filter (degrees).
            dt:            Frame time in seconds.
            double_points: True when a shake was detected; first gem collected
                           earns 2× points, then this resets to False.

        Returns:
            True if double_points was consumed (gem collected), False otherwise.
            The caller uses this to clear state.double_points.
        """
        if self.over:
            return False

        self.vel.x = (self.vel.x + TILT_ACCEL * roll_deg  * dt) * FRICTION
        self.vel.y = (self.vel.y + TILT_ACCEL * pitch_deg * dt) * FRICTION
        self.ball.x += self.vel.x
        self.ball.y += self.vel.y

        r = BALL_R
        if self.ball.x - r < self._BX:
            self.ball.x = self._BX + r;               self.vel.x =  abs(self.vel.x) * 0.5
        elif self.ball.x + r > self._BX + self._BW:
            self.ball.x = self._BX + self._BW - r;    self.vel.x = -abs(self.vel.x) * 0.5
        if self.ball.y - r < self._BY:
            self.ball.y = self._BY + r;                self.vel.y =  abs(self.vel.y) * 0.5
        elif self.ball.y + r > self._BY + self._BH:
            self.ball.y = self._BY + self._BH - r;     self.vel.y = -abs(self.vel.y) * 0.5

        consumed = False
        for gem in self.gems:
            if not gem.collected:
                if math.hypot(self.ball.x - gem.x, self.ball.y - gem.y) < r + GEM_R:
                    gem.collected = True
                    pts = 20 if double_points else 10
                    self.score += pts
                    consumed = double_points   # signal to clear the flag
                    break                      # one gem per frame

        for hole in self.holes:
            if math.hypot(self.ball.x - hole.x, self.ball.y - hole.y) < HOLE_R - r // 2:
                self.lives -= 1
                if self.lives <= 0:
                    self.over = True
                    return False
                cx, cy = self._centre()
                self.ball = _Vec2(cx, cy)
                self.vel  = _Vec2()
                return False

        if all(g.collected for g in self.gems):
            self.level += 1
            self._respawn()

        return consumed

    # ── Rendering ─────────────────────────────────────────────────────────

    def draw(self, surface: pygame.Surface,
             font: pygame.font.Font, font_sm: pygame.font.Font,
             double_points: bool = False, frame: int = 0) -> None:
        """
        Render the game panel.

        Args:
            surface:       Target pygame Surface (GAME_W wide).
            font:          Main HUD font.
            font_sm:       Small font for labels.
            double_points: When True, gems are drawn in gold and the HUD
                           shows a blinking "2× POINTS!" indicator.
            frame:         Current frame count (used for blink timing).
        """
        surface.fill(C_BG_GAME)

        pygame.draw.rect(surface, C_BOARD, (self._BX, self._BY, self._BW, self._BH))
        for xi in range(0, self._BW + 1, 76):
            pygame.draw.line(surface, C_GRID,
                             (self._BX + xi, self._BY), (self._BX + xi, self._BY + self._BH))
        for yi in range(0, self._BH + 1, 76):
            pygame.draw.line(surface, C_GRID,
                             (self._BX, self._BY + yi), (self._BX + self._BW, self._BY + yi))
        pygame.draw.rect(surface, C_BORDER, (self._BX, self._BY, self._BW, self._BH), 3)

        for hole in self.holes:
            ix, iy = int(hole.x), int(hole.y)
            pygame.draw.circle(surface, C_HOLE,    (ix, iy), HOLE_R)
            pygame.draw.circle(surface, C_HOLE_RIM, (ix, iy), HOLE_R, 2)

        # Gems: cyan normally, gold when double-points is active
        gem_col    = C_GEM_GOLD    if double_points else C_GEM
        gem_hi_col = C_GEM_GOLD_HI if double_points else C_GEM_HI
        for gem in self.gems:
            if not gem.collected:
                ix, iy = int(gem.x), int(gem.y)
                r_draw = GEM_R + 2 if double_points else GEM_R   # slightly bigger when gold
                pygame.draw.circle(surface, gem_col,    (ix, iy), r_draw)
                pygame.draw.circle(surface, gem_hi_col, (ix - 3, iy - 3), r_draw // 3)

        bx, by = int(self.ball.x), int(self.ball.y)
        pygame.draw.circle(surface, C_BALL,       (bx, by), BALL_R)
        pygame.draw.circle(surface, C_BALL_HI,    (bx - 4, by - 4), BALL_R // 3)
        pygame.draw.circle(surface, (80, 80, 80),  (bx, by), BALL_R, 2)  # dark outline

        # HUD background
        pygame.draw.rect(surface, C_HUD_BG, (0, 0, GAME_W, HUD_H))
        surface.blit(font.render(f"Score: {self.score}", True, C_TEXT), (12, 12))
        lev = font.render(f"Level {self.level}", True, C_TEXT)
        surface.blit(lev, (GAME_W // 2 - lev.get_width() // 2, 12))
        for i in range(self.lives):
            pygame.draw.circle(surface, C_LIFE, (GAME_W - 20 - i * 28, HUD_H // 2), 10)

        # "2× POINTS!" blink (visible every other 20-frame block)
        if double_points and (frame // 20) % 2 == 0:
            dbl = font.render("2×  POINTS!", True, C_DOUBLE_TXT)
            # Draw with a dark drop-shadow for readability
            shadow = font.render("2×  POINTS!", True, (60, 50, 0))
            cx = GAME_W // 2
            surface.blit(shadow, (cx - dbl.get_width() // 2 + 2, WIN_H // 2 - 30 + 2))
            surface.blit(dbl,    (cx - dbl.get_width() // 2,     WIN_H // 2 - 30))

        # Shake hint in simulator mode (bottom of panel)
        title = font_sm.render("Marble Madness  ·  Shake=2×pts  R=restart  ESC=quit",
                                True, C_TEXT)
        surface.blit(title, (GAME_W // 2 - title.get_width() // 2, WIN_H - 22))

        if self.over:
            overlay = pygame.Surface((GAME_W, WIN_H), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 165))
            surface.blit(overlay, (0, 0))
            go = font.render("GAME OVER", True, (240, 80, 80))
            sc = font.render(f"Final Score: {self.score}", True, C_TEXT)
            rs = font_sm.render("Press  R  to restart   |   ESC to quit", True, C_TEXT)
            mid = WIN_H // 2
            surface.blit(go, (GAME_W // 2 - go.get_width() // 2, mid - 60))
            surface.blit(sc, (GAME_W // 2 - sc.get_width() // 2, mid))
            surface.blit(rs, (GAME_W // 2 - rs.get_width() // 2, mid + 55))


# =============================================================================
# Main loop
# =============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Step 7 — 3-D Orientation Visualiser + Marble Madness game."
    )
    parser.add_argument("--port",     default=None)
    parser.add_argument("--baud",     type=int, default=115200)
    parser.add_argument("--simulate", action="store_true",
                        help="Use synthetic IMU data — no hardware required.")
    args = parser.parse_args()

    reader = make_reader(port=args.port, simulate=args.simulate)
    reader.connect()

    state = SharedState()
    threading.Thread(
        target=imu_worker, args=(reader, state),
        daemon=True, name="imu",
    ).start()

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("EECE 5520 Step 7  ·  3-D Orientation + Marble Madness")
    clock = pygame.time.Clock()

    font    = pygame.font.SysFont("Arial", 22, bold=True)
    font_sm = pygame.font.SysFont("Arial", 16)

    # Two off-screen surfaces, blitted side-by-side each frame
    viz_surface  = pygame.Surface((VIZ_W, WIN_H))
    game_surface = pygame.Surface((GAME_W, WIN_H))

    game = MarbleGame()
    frame = 0

    # ── Camera orbit state (mouse drag on the viz panel) ──────────────────
    cam_pitch: float = VIEW_PITCH   # degrees below horizontal; clamped [-85, -5]
    cam_yaw:   float = VIEW_YAW     # left/right orbit angle; unclamped
    drag_active: bool = False
    drag_last_x: int  = 0
    drag_last_y: int  = 0
    DRAG_SENSITIVITY: float = 0.35  # degrees per pixel
    PITCH_MIN: float = -85.0
    PITCH_MAX: float = -5.0

    print("[Step 7] Window open.")
    print("[Step 7] Tilt your board to roll the marble and rotate the 3-D model.")
    print("[Step 7]   Cyan gems  = +10 pts       Dark holes = lose a life")
    print("[Step 7]   SHAKE      = gems turn gold, next one = +20 pts")
    print("[Step 7]   R = restart   |   V = reset view   |   ESC = quit")
    print("[Step 7]   Left-drag on the 3-D panel to orbit the viewpoint.")
    if hasattr(reader, '_next_shake'):
        print(f"[Step 7] (Simulator: shake event every {15:.0f} s of sim time)")

    while True:
        dt = clock.tick(FPS) / 1000.0
        frame += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                state.running = False
                pygame.quit()
                reader.close()
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    game.reset()
                elif event.key == pygame.K_v:
                    # Reset camera to default viewpoint
                    cam_pitch = VIEW_PITCH
                    cam_yaw   = VIEW_YAW
                elif event.key == pygame.K_ESCAPE:
                    state.running = False
                    pygame.quit()
                    reader.close()
                    return

            # ── Mouse drag to orbit the 3-D panel ─────────────────────────
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1 and event.pos[0] < VIZ_W:
                    drag_active = True
                    drag_last_x, drag_last_y = event.pos
                    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_SIZEALL)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1 and drag_active:
                    drag_active = False
                    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
            elif event.type == pygame.MOUSEMOTION:
                if drag_active:
                    dx = event.pos[0] - drag_last_x
                    dy = event.pos[1] - drag_last_y
                    drag_last_x, drag_last_y = event.pos
                    cam_yaw   += dx * DRAG_SENSITIVITY
                    cam_pitch += dy * DRAG_SENSITIVITY   # drag down → shallower angle
                    cam_pitch  = max(PITCH_MIN, min(PITCH_MAX, cam_pitch))

        with state.lock:
            roll, pitch, yaw = state.roll, state.pitch, state.yaw
            double_points = state.double_points

        # Update game; if a gem was collected with double_points, clear the flag
        consumed = game.update(roll, pitch, dt, double_points=double_points)
        if consumed:
            with state.lock:
                state.double_points = False

        # Render 3-D panel (left) — rebuild view matrix each frame so drag updates instantly
        view_matrix = _make_view_matrix(cam_pitch, cam_yaw)
        draw_3d_panel(viz_surface, font_sm, roll, pitch, yaw, view_matrix)

        # Render game panel (right)
        game.draw(game_surface, font, font_sm,
                  double_points=double_points, frame=frame)

        # Compose: blit both panels + divider
        screen.blit(viz_surface,  (0,     0))
        screen.blit(game_surface, (VIZ_W, 0))
        pygame.draw.line(screen, C_DIVIDER, (VIZ_W, 0), (VIZ_W, WIN_H), 2)

        pygame.display.flip()

    state.running = False


if __name__ == "__main__":
    main()
