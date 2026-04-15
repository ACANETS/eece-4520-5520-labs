# EECE 4520/5520 — Microprocessor II and Embedded System Design
# Lab: IMU Motion Sensing & Orientation Tracking
# Semester: Spring 2026 | University of Massachusetts Lowell

---

## General Information

| | |
|---|---|
| **Course** | EECE 4520 (UG) / EECE 5520 (Graduate) |
| **Semester** | Spring 2026 |
| **Lab Title** | IMU Motion Sensing & Orientation Tracking |
| **Submission** | Submit on Blackboard: PDF report including GitHub repo link  |
| **Due Date** | Check Blackboard |

> **EECE 5520 Graduate Requirement**
> Graduate students must complete all steps (1–5) plus **Step 6: Activity Detection**,
> which is a graduate-only task.  Step 7 (3D Orientation + Marble Madness) is a
> **provided demonstration** — run it, explore it, but no coding or report questions
> are required for it.
> The graduate report must include additional analysis sections marked **[5520]**.

---

## 1. Lab Description

In this lab you will build a complete pipeline for acquiring, processing, and
visualizing inertial measurement unit (IMU) data.  You will work with the
**MPU-6050**, a low-cost 6-axis IMU that combines a 3-axis MEMS accelerometer
and a 3-axis MEMS gyroscope on a single chip.

Starting from raw register-level I2C communication on the Arduino, you will:

1. Stream calibrated sensor data to a Python host over USB serial
2. Visualize all six signal channels in real time
3. Compute roll and pitch from accelerometer math
4. Estimate orientation by integrating gyroscope readings over time
5. Fuse both sensors using a **complementary filter** — the workhorse of low-cost
   orientation estimation
6. *(5520 only)* Detect and classify motion activity 
7. *(provided demo)* Explore a real-time 3D orientation viewer and tilt-controlled
   game built on the same IMU pipeline

By the end you will understand the fundamental trade-offs between accelerometers
and gyroscopes, and you will have implemented the complementary filter from
scratch — the foundation upon which more sophisticated algorithms (Kalman filter,
Mahony filter, Madgwick filter) are built.

---

## 2. Background

### 2.1 The MPU-6050 IMU

The MPU-6050 (InvenSense / TDK) is a 6-axis inertial measurement unit:

- **Accelerometer**: 3-axis (X, Y, Z), measures specific force in units of *g*
  (1 g = 9.81 m/s²).  Default full-scale range: ±2 g, sensitivity: 16 384 LSB/g.
- **Gyroscope**: 3-axis (X, Y, Z), measures angular velocity in °/s.
  Default full-scale range: ±250 °/s, sensitivity: 131 LSB/(°/s).
- **Communication**: I2C (up to 400 kHz).  I2C address: 0x68 (AD0 = LOW).
- **Supply**: 3.3 V (most breakout boards include a 5 V → 3.3 V regulator).

### 2.2 I2C Communication

I2C is a two-wire serial protocol:
- **SDA** (Serial Data): bidirectional data line
- **SCL** (Serial Clock): master-generated clock

On the Arduino Mega 2560: SDA → pin 20, SCL → pin 21.

A master write transaction:
```
[START] [addr+W] [ACK] [reg] [ACK] [data] [ACK] [STOP]
```
A master read (register read via repeated start):
```
[START] [addr+W] [ACK] [reg] [ACK]
[START] [addr+R] [ACK] [data] [NACK] [STOP]
```

The MPU-6050 supports **burst reads**: after selecting the starting register,
the device auto-increments the register pointer with each clock, allowing all
14 bytes (accel X/Y/Z + temp + gyro X/Y/Z) to be read in one transaction.

### 2.3 Coordinate Frame

The MPU-6050 uses a right-handed coordinate system with the chip face-up:

```
          +Y (forward / pitch nose-up)
           ^
           |
   +Z up   +------> +X (right / roll right)
```

- **Roll**: rotation about the X-axis (board tilts left/right)
- **Pitch**: rotation about the Y-axis (board tilts nose up/down)
- **Yaw**: rotation about the Z-axis (board rotates in the horizontal plane)

When the board lies flat (face up):
- `az ≈ +1.0 g`   (gravity points down = −Z in world = +Z in sensor)
- `ax ≈ ay ≈ 0 g`

### 2.4 Gravity-Based Roll and Pitch

When stationary, the accelerometer measures Earth's gravity vector.  By
decomposing gravity into the sensor's body axes, we can infer orientation:

```
roll  = atan2(ay, az)
pitch = atan2(−ax, sqrt(ay² + az²))
```

**Why does this work?**
- If the board rolls right, the Y component of gravity increases while Z decreases.
- `atan2(ay, az)` gives the angle between the gravity vector projection and the Z axis.
- The pitch formula uses the magnitude of the Y-Z plane to remain stable under roll.

**Limitation**: This only works when gravity dominates.  During linear acceleration
the measurement is corrupted.  Yaw cannot be determined this way (gravity is
symmetric about the vertical axis).

### 2.5 Gyroscope Integration

The gyroscope measures angular velocity ω (°/s).  Integrating over time gives angle:

```
θ(t) = θ(t-1) + ω · Δt
```

**Strength**: Smooth, unaffected by short-duration accelerations.
**Weakness**: Even a tiny constant bias ε in the gyro reading causes the
integrated angle to grow linearly:  θ_error(t) = ε · t.  This is called **gyro
drift**.

A typical low-cost gyro may have a bias of 0.1–1 °/s at room temperature,
producing 6–60° of drift per minute.

### 2.6 Complementary Filter

The complementary filter combines the long-term accuracy of the accelerometer
with the short-term smoothness of the gyroscope:

```
θ_fused = α · (θ_fused + ω · Δt) + (1−α) · θ_accel
```

- `α → 1.0`: trust the gyro more (smooth but drifts)
- `α → 0.0`: trust the accelerometer more (accurate long-term but noisy)
- `α = 0.98` at 50 Hz applies a time constant of ~1 second

The filter is **complementary** because it applies a high-pass filter to the
gyro (passes fast changes, blocks DC drift) and a low-pass filter to the
accelerometer (passes slow gravity signal, rejects high-frequency noise).
The two filters sum to unity, hence "complementary."

---

## 3. Lab Materials

### Hardware
- Arduino Mega 2560 (same as Lab 2)
- MPU-6050 breakout board
- USB cable (Arduino ↔ host computer)
- Breadboard and jumper wires

**No additional parts are required.**

### Wiring

| MPU-6050 Pin | Arduino Mega Pin |
|:---:|:---:|
| VCC | 5 V (or 3.3 V — check your breakout) |
| GND | GND |
| SDA | 20 (SDA) |
| SCL | 21 (SCL) |
| AD0 | GND (sets I2C address to 0x68) |
| INT | Not connected |

### Software

| Tool | Version | Notes |
|------|---------|-------|
| Arduino IDE | ≥ 2.0 | |
| Python | ≥ 3.9 | |
| pyserial | ≥ 3.5 | `pip install pyserial` |
| matplotlib | ≥ 3.5 | `pip install matplotlib` |
| numpy | ≥ 1.21 | `pip install numpy` |
| pygame | ≥ 2.1 | Step 7 demo only: `pip install pygame` |

**Set up a virtual environment first** (recommended — keeps lab packages isolated):

macOS / Linux:
```bash
cd python
python3 -m venv venv
source venv/bin/activate
```

Windows (Command Prompt):
```bat
cd python
python -m venv venv
venv\Scripts\activate.bat
```

Windows (PowerShell):
```powershell
cd python
python -m venv venv
venv\Scripts\Activate.ps1
```

Your prompt will show `(venv)` when the environment is active.
To deactivate when done: `deactivate`

> **Note:** Re-activate the virtual environment each time you open a new terminal
> before running any lab scripts.

Install all Python dependencies at once (with venv active):
```bash
pip install -r requirements.txt
```

---

## 4. Pre-Lab Reading

Before arriving at the lab session, read:
- This document in full
- The MPU-6050 datasheet register map (Section 4, "Register Descriptions")
  — specifically PWR_MGMT_1, ACCEL_XOUT_H, GYRO_XOUT_H
- The `imu_reader.py` source code (provided) — understand the `IMUReader` class
  and the `IMUSimulator` class

**Simulator mode**: All Python scripts accept `--simulate`.  This generates
synthetic IMU data without hardware.  Use it to understand the code before
your lab session.

---

## 5. Lab Instructions

All students complete Steps 1–5.  Graduate students (5520) also complete **Step 6**.
Step 7 is a **provided demonstration** — all students are encouraged to run it,
but no deliverables are required.

Each step builds on the previous.  The reference scripts are self-contained and
runnable.  You will observe behaviors, answer questions, and capture screenshots
for your report.

### 5.1 Step 1 — Raw Data Streaming & Logging

**Objective**: Connect to the Arduino, confirm sensor data is streaming correctly,
and log a data file.

**Setup**:
1. Flash `arduino/imu_stream/imu_stream.ino` to the Arduino Mega.
2. Open a terminal and `cd` to the `python/` directory.

**Run**:
```
python step1_stream.py
```
Or with the simulator (no hardware):
```
python step1_stream.py --simulate
```

**Expected output**:
```
[  12.345s]  ax=  0.0023g  ay= -0.0105g  az=  0.9987g  |  gx=  0.21°/s  gy= -0.10°/s  gz=  0.09°/s
```

**What to observe**:
- With the board flat, `az` should be close to +1.0 g.  `ax` and `ay` should be near 0.
- Tilt the board around each axis and observe which accelerometer value changes.
- The gyroscope reads near zero when the board is stationary.  Small non-zero
  values are normal (gyro noise + bias).
- A CSV file `imu_log.csv` is created in the current directory.

**Questions for report (Step 1)**:
1. When you tilt the board 90° to the right (roll), which accelerometer axis
   changes and in which direction?  Why?
2. What is the approximate noise level (standard deviation) of `ax` when the
   board is stationary?  Compute this from your logged CSV data.
3. Why is it necessary for Python to send `'s'` before the Arduino begins
   streaming?  What problem does this handshake solve?

---

### 5.2 Step 2 — Real-Time 6-Axis Plot

**Objective**: Visualize all six IMU channels simultaneously in a live scrolling plot.

**Run**:
```
python step2_live_plot.py --simulate
```

**What to observe**:
- The top subplot shows accelerometer X, Y, Z (range ±2 g).
- The bottom subplot shows gyroscope X, Y, Z (range ±250 °/s).
- At rest: `az ≈ 1 g`, gyros ≈ 0 °/s.
- During motion: gyro spikes instantly; accelerometer shows both gravity + linear acceleration.
- After 30+ seconds at rest: the gyro values do not drift (they stay near 0) — but
  integrated gyro *angle* would drift (see Step 4).

**Questions for report (Step 2)**:
1. Describe what you observe in the plot when you hold the board still for 10 seconds,
   then move it rapidly, then hold it still again.
2. Why is data acquisition done in a separate thread from the plotting?  What would
   happen if both were in the same thread?
3. What is the purpose of the `deque(maxlen=200)` circular buffer?

---

### 5.3 Step 3 — Accelerometer-Based Roll and Pitch

**Objective**: Compute roll and pitch from the accelerometer gravity projection
and verify the math against physical orientation.

**Run**:
```
python step3_accel_angles.py --simulate
```

**What to observe**:
- Roll and pitch angles update as you tilt the board.
- When flat: Roll ≈ 0°, Pitch ≈ 0°.
- When tilted 45° to the right (roll): Roll ≈ 45°, Pitch ≈ 0°.
- Shake the board rapidly — the angle estimate becomes very noisy during motion.
  This is the key weakness of accelerometer-only orientation.

**Questions for report (Step 3)**:
1. Derive the roll formula `atan2(ay, az)` from first principles.  Draw a diagram
   showing the gravity vector decomposition.
2. Why can't you estimate yaw from the accelerometer alone?
3. Shake the board and describe what happens to the roll/pitch estimate.  Why does
   this happen?

---

### 5.4 Step 4 — Gyroscope Integration

**Objective**: Compare accelerometer angles and gyro-integrated angles side by side,
and observe gyro drift.

**Run**:
```
python step4_gyro_integrate.py --simulate
```

**What to observe**:
- Initially both panels show the same roll and pitch (gyro is initialised from accel).
- During motion, the gyro panel is smoother than the accel panel.
- After 60+ seconds at rest, the gyro-integrated angle slowly drifts away from
  the accelerometer angle.  The simulator includes a small simulated gyro bias drift
  to make this observable.

**Questions for report (Step 4)**:
1. Measure the gyro drift rate (degrees per minute) from your plot.  Is it consistent
   with the MPU-6050 datasheet specifications?
2. Explain why the gyro panel is smoother than the accel panel during motion.
3. What is the fundamental physical reason that gyroscopes drift?  (Hint: research
   "gyro bias" and "temperature coefficient".)

---

### 5.5 Step 5 — Complementary Filter

**Objective**: Fuse accelerometer and gyroscope data to produce a stable,
drift-free angle estimate.

**Run**:
```
python step5_comp_filter.py --simulate
```

**What to observe**:
- The fused panel (rightmost) should be both smooth (like the gyro) and
  drift-free (like the accel).
- Experiment by changing the `ALPHA` constant at the top of the file:
  - `ALPHA = 0.80`: Very drift-resistant but can feel sluggish
  - `ALPHA = 0.95`: Balanced
  - `ALPHA = 0.98`: Default — good for most applications at 50 Hz
  - `ALPHA = 0.999`: Very smooth but takes a long time to correct drift

**Questions for report (Step 5)**:
1. For each ALPHA value you tested, describe the qualitative behavior during
   (a) stationary hold, (b) slow tilt, (c) fast shake.
2. Derive the "time constant" of the complementary filter as a function of
   ALPHA and sample rate fs.  What is the time constant for ALPHA=0.98, fs=50 Hz?
3. A Kalman filter can also fuse these sensors.  What advantage does it offer over
   the complementary filter?  What disadvantage?

---

### 5.6 Step 6 — Activity Detection *(EECE 5520 only)*

**Objective**: Implement and tune a state machine that classifies motion as
STATIONARY, MOVING, or SHAKING using the accelerometer magnitude.

**Run**:
```
python step6_activity.py --simulate
```

**What to observe**:
- The live plot shows the accelerometer magnitude (should be near 1.0 g at rest).
- The background color changes: green (stationary), yellow (moving), red (shaking).
- State transitions are printed to the terminal with timestamps.
- Try walking with the board, then shaking it vigorously.

**Tuning exercise**:
Adjust the three threshold constants in the file:
- `THRESH_MOVING` (default 0.05 g)
- `THRESH_SHAKING` (default 0.30 g)
- `STAT_WINDOW` (default 10 samples)

**Questions for report (Step 6)**:
1. What thresholds gave the best classification on your data?  Justify your choices.
2. What are the limitations of using a single scalar feature (magnitude) for activity
   detection?  Give an example of a motion that would be misclassified.
3. Propose one additional feature (e.g., from the gyroscope) that could improve
   classification accuracy.

---

### 5.7 Step 7 — 3D Orientation + Marble Madness *(Provided Demo — all students)*

**Overview**: `step7_3d_viz.py` is a fully implemented demonstration that brings
together everything from Steps 1–6 into a single pygame window:

- **Left panel**: Real-time 3D wireframe of the MPU-6050 board, rotating to match
  the physical sensor orientation.  Uses numpy rotation matrices and perspective
  projection.  *Drag the panel with your mouse to orbit the viewpoint.*
- **Right panel**: "Marble Madness" — tilt the board to roll a marble, collect gems
  (+10 pts each), avoid holes.  Shake the board to activate double-points mode.

This script is **complete and requires no modifications**.  There are no TODOs.

**Run with simulator** (no hardware needed):
```
python step7_3d_viz.py --simulate
```

**Run with hardware**:
```
python step7_3d_viz.py
```

**Controls**:

| Input | Action |
|-------|--------|
| Tilt board | Rotate 3D model + roll marble |
| Left-drag (3D panel) | Orbit camera viewpoint |
| **V** | Reset camera to default view |
| Shake board | Gems turn gold — next collected = 2× points |
| **R** | Restart game |
| **ESC** | Quit |

> **What to notice**: The 3D panel uses the same complementary filter from Step 5.
> The marble game uses the same `roll` and `pitch` values that drive the 3D model —
> roll tilts the marble left/right; pitch tilts it forward/back.  Yaw is visualized
> in the 3D panel but does not affect the game (it would require a magnetometer for
> a drift-free heading).

*No report questions or deliverables for Step 7.*

---

## 6. Deliverables

**PDF report** — Follow the lab report guideline on Blackboard, and must include a URL to your private (instructor-accessible) repo

### Repository Structure (all students)

Your repo must contain at least:
```
arduino/imu_stream/imu_stream.ino
python/
├── imu_reader.py            (provided — include unchanged)
├── step1_stream.py
├── step2_live_plot.py
├── step3_accel_angles.py
├── step4_gyro_integrate.py
├── step5_comp_filter.py     (ALPHA value you settled on noted in a comment)
└── imu_log.csv              (at least 60 seconds of logged data from Step 1)
```

### Repository Structure (5520 only, add to repo)
```
python/
└── step6_activity.py        (final threshold values noted in comments)
```

> **Note**: `step7_3d_viz.py` is provided — include it unchanged in your repo,
> but no modifications are expected.

### Report (all students)
Follow the lab report guideline on Blackboard, **and** includes:
- Screenshots from Steps 1 through 5
- All questions from Steps 1–5 answered, with plots/screenshots

### Report (5520 only, add sections)
- Step 6 questions answered (with activity plots and threshold justification)

---

## Appendix A: Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| No port found | Arduino not connected or driver missing | Check Device Manager (Windows) or `ls /dev/tty.*` (macOS/Linux) |
| `IMU_READY` not received | Wrong baud rate or wrong sketch flashed | Verify 115200 baud; re-flash sketch |
| `az` reads −1.0 g (negative) | Board is face-down | Flip the board over |
| Plot does not open | Wrong matplotlib backend | Try `--simulate`; change `matplotlib.use()` |
| Step 7 pygame window doesn't appear | pygame not installed | `pip install pygame` |
| Gyro drifts very fast | High temperature or power supply noise | Let the board warm up 1–2 minutes |

---

## Appendix B: Key Register Map (MPU-6050)

| Register | Hex | Description |
|----------|-----|-------------|
| `PWR_MGMT_1` | 0x6B | Write 0x00 to wake device |
| `SMPLRT_DIV` | 0x19 | Sample rate divider |
| `CONFIG` | 0x1A | DLPF config (0x03 = 44 Hz BW) |
| `GYRO_CONFIG` | 0x1B | Gyro full-scale (0x00 = ±250 °/s) |
| `ACCEL_CONFIG` | 0x1C | Accel full-scale (0x00 = ±2 g) |
| `ACCEL_XOUT_H` | 0x3B | Start of 14-byte data block |
| `GYRO_XOUT_H` | 0x43 | Gyro X high byte |

Raw-to-physical conversion:
```
ax_g   = raw_ax / 16384.0    (±2g range)
gx_dps = raw_gx / 131.0      (±250°/s range)
```

---

*Lab writeup prepared for EECE 4520/5520 Spring 2026, University of Massachusetts Lowell.*
*Questions? Post on Blackboard/Canvas or visit office hours.*
