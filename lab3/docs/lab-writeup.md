# EECE 4520/5520 — Microprocessor II and Embedded System Design
# Lab 3: Smart Parking Sensor — Ultrasonic Sensing, Motor Control & SPI Display
# Semester: Spring 2026 | University of Massachusetts Lowell

---

## General Information

| | |
|---|---|
| **Course** | EECE 4520 (UG) / EECE 5520 (Graduate) |
| **Semester** | Spring 2026 |
| **Lab Title** | Smart Parking Sensor — Ultrasonic Sensing, Motor Control & SPI Display |
| **Submission** | Submit on Blackboard: GitHub repo link + PDF report |
| **Late Policy** | 10% per day, no credit after 5 days |

> **EECE 5520 Graduate Requirement**
> Graduate students must complete all steps (1–7) plus **Steps 5–6: Serial Protocol
> Framing & Python Dashboard**, which are graduate-only coding tasks.
> The graduate report must include additional analysis sections marked **[5520]**.

---

## 1. Lab Description

In this lab you will build a **smart parking assistant** — an embedded system that
measures the distance from a car to a wall using ultrasound, and responds in
multiple ways: slowing a DC motor, activating a buzzer alarm, displaying distance
on an LCD, visualizing proximity on an LED matrix, and streaming sensor data to a
Python host.

Starting from raw hardware control on the Arduino, you will:

1. Measure distance with the HC-SR04 ultrasonic sensor using pulse timing
2. Drive a DC motor at proportional speed via the L293D Half-H driver and PWM
3. Display real-time status on a 16×2 parallel LCD and a MAX7219 SPI LED matrix
4. Sound a passive buzzer alarm with frequency proportional to proximity
5. Use **Timer1 hardware interrupts** to update the display and motor every 100 ms
   without blocking the main measurement loop
6. *(5520 only)* Design and implement a **binary serial framing protocol** with
   start bytes and checksums
7. *(5520 only)* Build a multi-panel **Python dashboard** that decodes and
   visualizes your binary protocol
8. *(provided demo)* Explore a real-time **pygame visualization** integrating all
   system components

By the end you will have experience with ultrasonic ranging, H-bridge motor
control, SPI bus communication, hardware timer interrupts, and the principles of
robust serial framing — skills directly applicable to robotics, automotive
systems, and IoT products.

---

## 2. Background

### 2.1 HC-SR04 Ultrasonic Sensing

The HC-SR04 is a low-cost ultrasonic ranging module that measures distance by
timing a sound-wave round trip.

**How it works:**
1. Pull TRIG HIGH for ≥10 µs → the sensor fires **8 ultrasonic pulses** at 40 kHz
2. ECHO pin goes HIGH immediately after the pulses are transmitted
3. ECHO goes LOW when the reflected echo returns to the sensor
4. The time ECHO is HIGH equals the round-trip travel time of sound

**Distance formula:**

```
echo_time (µs) = 2 × distance_cm / speed_of_sound_cm_per_µs
distance_cm = echo_time × 0.034 / 2
```

Speed of sound ≈ 340 m/s = 0.034 cm/µs at room temperature.  The factor of 2
accounts for the round trip (out and back).

**Arduino implementation:**

```cpp
// 1. Ensure TRIG is LOW, settle 2 µs
digitalWrite(TRIG_PIN, LOW);
delayMicroseconds(2);

// 2. Fire 10 µs pulse
digitalWrite(TRIG_PIN, HIGH);
delayMicroseconds(10);
digitalWrite(TRIG_PIN, LOW);

// 3. Measure ECHO pulse width (timeout 30 ms ≈ 5 m range)
unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

// 4. Convert to cm
float dist_cm = (float)duration * 0.034f / 2.0f;
```

**Range:** Effective range is 2–400 cm.  `pulseIn()` returns 0 on timeout
(no echo detected), which we treat as "out of range."

### 2.2 DC Motor Control with L293D Half-H Driver

The L293D is a Half-H driver integrated circuit that allows a microcontroller to
control a DC motor's **speed and direction** using low-current logic signals.

**Why an H-bridge?**
A DC motor requires more current than an Arduino pin can supply (~40 mA per pin
vs. hundreds of mA for a motor).  The H-bridge acts as a power amplifier:
the Arduino logic signal controls internal transistors that switch the full
motor supply voltage.

**H-bridge direction control (IN1/IN2):**

| IN1 | IN2 | Motor |
|-----|-----|-------|
| HIGH | LOW | Forward |
| LOW | HIGH | Reverse |
| LOW | LOW | Brake (coasting) |
| HIGH | HIGH | Brake (short) |

**Speed control (ENA pin):**
The ENA pin controls motor voltage via PWM:
- `analogWrite(ENA_PIN, 255)` → full voltage → full speed
- `analogWrite(ENA_PIN, 128)` → 50% duty cycle → ~half speed
- `analogWrite(ENA_PIN, 0)` → motor off

**L293D power supply pins:**
- **VCC1** → 5V (logic supply for the IC's internal gates)
- **VCC2** → motor supply voltage (4.5–36V; use 5V from Arduino if your motor is 5V-rated, or an external supply for higher-voltage motors)

> ℹ️ Unlike some H-bridge modules (e.g. L298N), the L293D has **no ENA jumper** — the ENA pin is directly accessible and works with PWM from the start.

**Zone-based proportional control in this lab:**

| Zone | Distance | Motor PWM | Speed Level |
|------|----------|-----------|-------------|
| 0 | > 75 cm | 255 | Full |
| 1 | 50–75 cm | 191 | 3/4 |
| 2 | 25–50 cm | 128 | Half |
| 3 | 0–25 cm | 0 | Stop |

### 2.3 SPI Bus and MAX7219 LED Matrix Driver

**SPI (Serial Peripheral Interface)** is a synchronous serial protocol using
four wires:
- **MOSI** (Master Out Slave In): data from Arduino to MAX7219 → Pin 51
- **MISO** (Master In Slave Out): data from MAX7219 to Arduino (not used here)
- **SCK** (Serial Clock): clock generated by Arduino → Pin 52
- **SS/CS** (Chip Select): active-LOW select → Pin 53

SPI is faster than I2C (up to tens of MHz) and uses a dedicated CS line per
device, making it well-suited for high-speed peripherals like LED matrix drivers,
DACs, and display controllers.

**MAX7219 overview:**
The MAX7219 drives up to 64 LEDs (8 rows × 8 columns) using just 3 wires from
the Arduino (DIN, CLK, CS).  The `LedControl` library abstracts the SPI register
writes, providing `lc.setRow(device, row, byte)` to turn on/off any row.

**LED matrix visualization in this lab:**
- Distance 0 cm → 8 rows lit (all LEDs on — danger)
- Distance 100+ cm → 0 rows lit (all LEDs off — safe)
- Formula: `rows_lit = map(constrain(dist, 0, 100), 0, 100, 8, 0)`

```
Closer → more rows lit → danger indicator
Top row = most danger / closest
```

### 2.4 Parallel LCD (16×2 HD44780)

The 16×2 LCD in your kit uses a standard **HD44780-compatible controller** with
a 16-pin header, wired directly to Arduino digital pins in **4-bit mode**.  The
built-in `LiquidCrystal` library (no installation required) handles the timing.

In 4-bit mode only pins D4–D7 carry data; D0–D3 are left unconnected.  You also
need a **10 kΩ potentiometer** on the V0 (contrast) pin — without it, characters
may be invisible.  A common first-run issue: turn the pot slowly until a row of
blocks appears on line 1 (power-on test), then text will show once the sketch
runs.

No I2C, no address scanning, no extra library needed.  Key difference from an
I2C LCD: `lcd.begin(16, 2)` replaces `lcd.init() + lcd.backlight()`.  The
`LiquidCrystal` library uses only bit-banging (GPIO + `delayMicroseconds`), so
it is safe to call from a Timer1 ISR — unlike `LiquidCrystal_I2C` which uses
the hardware I2C interrupt and would deadlock inside an ISR.

**LCD line format:**
```
Line 0: "Dist:  47.3 cm  "
Line 1: "Speed: Half     "
```

### 2.5 Passive Buzzer and tone()

A passive buzzer contains only a piezoelectric element — it requires an external
oscillating voltage to produce sound.  The Arduino's `tone(pin, frequency)` function
generates a square wave at the specified frequency on the output pin.

**Alarm states:**
- `dist ≥ 60 cm`: `noTone()` — silent
- `20 ≤ dist < 60 cm`: `tone(BUZZER_PIN, map(dist, 20, 60, 2000, 500))`
  — beeping, frequency inversely proportional to distance (closer = higher pitch)
- `dist < 20 cm`: `tone(BUZZER_PIN, 2000)` — continuous 2 kHz alarm

### 2.6 Timer Interrupts (Timer1)

**What is a timer interrupt?**
A hardware timer counts clock cycles independently of the CPU.  When the count
reaches a configured threshold, the timer fires an **interrupt** — the CPU
immediately stops whatever it was doing, saves its state, executes an
**Interrupt Service Routine (ISR)**, then resumes the interrupted code.

**Why use Timer1 in this lab?**
- The main loop measures distance (blocking: ~30 ms for `pulseIn()` timeout)
- The display and motor updates should happen at a fixed 100 ms interval
- Timer1 decouples these: `loop()` measures as fast as possible; Timer1 ISR
  updates the display/motor exactly every 100 ms regardless of loop timing

```cpp
// Configure Timer1: fire ISR every 100 000 µs = 100 ms
Timer1.initialize(100000);
Timer1.attachInterrupt(isrCallback);
```

**The `volatile` keyword:**
`g_distance` is written by the main loop and read by the Timer1 ISR.  Without
`volatile`, the compiler may cache the variable in a CPU register and the ISR
will read a stale (old) value.  `volatile` tells the compiler: *every read/write
must go to RAM — never cache this variable*.

```cpp
volatile float g_distance = 0.0f;  // shared between loop() and ISR
```

**ISR constraints — never call inside an ISR:**
- `Serial.print()` — uses its own interrupt mechanism; calling from ISR → deadlock
- `delay()` — relies on Timer0 overflow interrupt; will never return from ISR
- Any long-running or blocking function — ISRs must be fast (< a few µs ideally)

The ISR in this lab only reads `g_distance`, calls `updateDisplay()` and
`updateMotor()` — no serial I/O, no delays.

### 2.7 Binary Serial Framing Protocol (5520 only)

**Why not CSV?**
CSV text (`"47.3,128,1,2\n"`) is human-readable but fragile:
- A single corrupted byte can misalign all subsequent fields
- A newline embedded in the data would be misinterpreted as a packet boundary
- No mechanism to detect partial or corrupted frames

**Binary framing goals:**
1. **Start byte** — receiver can find frame boundaries even after corruption
2. **Fixed length** — know exactly where each frame ends
3. **Checksum** — detect and reject corrupted frames

**Frame layout (7 bytes):**
```
[0xAA] [d_hi] [d_lo] [pwm] [alarm] [zone] [checksum]
  ↑      ↑       ↑     ↑      ↑       ↑        ↑
start   dist×10 big-  motor  alarm   zone    XOR of
byte    endian  uint16 PWM   state    id     bytes 1-5
```

**Checksum:** XOR of bytes 1–5 (the payload, excluding start byte and checksum
itself).  XOR is simple, fast, and detects any single-byte error.

**Framing robustness:** If the receiver loses sync (e.g., misses a start byte),
it simply scans forward until it sees 0xAA, then re-aligns.  The `FrameParser`
class in `sensor_reader.py` implements this state machine.

---

## 3. Lab Materials

### Hardware

| Component | Quantity | Purpose |
|-----------|----------|---------|
| Arduino Mega 2560 | 1 | Main controller |
| HC-SR04 ultrasonic sensor | 1 | Distance measurement |
| DC motor + L293D Half-H driver IC | 1 | Proportional speed control |
| 16×2 parallel LCD (HD44780) | 1 | Distance + speed display |
| Passive buzzer | 1 | Proximity alarm |
| MAX7219 8×8 LED matrix module | 1 | LED bar graph |
| USB cable | 1 | Programming + serial |
| Breadboard + jumper wires | — | Wiring |

### Wiring Summary

| Component | Signal | Arduino Mega Pin |
|-----------|--------|-----------------|
| HC-SR04 | TRIG | 9 |
| HC-SR04 | ECHO | 10 |
| L293D | ENA | 6 (PWM) |
| L293D | IN1 | 7 |
| L293D | IN2 | 8 |
| L293D | VCC1 | 5V |
| L293D | VCC2 | motor supply (5V or external) |
| Passive buzzer | + | 5 |
| LCD | RS | 22 |
| LCD | E | 23 |
| LCD | D4 | 24 |
| LCD | D5 | 25 |
| LCD | D6 | 26 |
| LCD | D7 | 27 |
| LCD | RW | GND (not an Arduino pin) |
| LCD | V0 | Middle pin of 10 kΩ pot (pot ends to 5V / GND) |
| MAX7219 | DIN | 51 (SPI MOSI) |
| MAX7219 | CLK | 52 (SPI SCK) |
| MAX7219 | CS | 53 (SPI SS) |

All sensor VCC → 5V, all GND → GND (common ground between Arduino and L293D).
L293D: VCC1 → 5V (logic), VCC2 → motor supply (5V–12V depending on motor).

### Software

| Tool | Version | Notes |
|------|---------|-------|
| Arduino IDE | ≥ 2.0 | |
| LiquidCrystal | built-in | Pre-installed with Arduino IDE — no download needed |
| LedControl | latest | Library Manager: "LedControl by Eberhard Fahle" |
| TimerOne | latest | Library Manager: "TimerOne" |
| Python | ≥ 3.9 | |
| pyserial | ≥ 3.5 | |
| matplotlib | ≥ 3.5 | |
| numpy | ≥ 1.21 | |
| pygame | ≥ 2.1 | Step 7 demo only |

**Install Arduino libraries** via Arduino IDE → Tools → Manage Libraries.
Search for each library by name and install.

**Set up Python virtual environment:**

macOS / Linux:
```bash
cd python
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Windows (Command Prompt):
```bat
cd python
python -m venv venv
venv\Scripts\activate.bat
pip install -r requirements.txt
```

---

## 4. Pre-Lab Reading

Before the lab session, read:
- This document in full
- `arduino/parking_sensor/parking_sensor.ino` — study the provided code
  (especially `updateDisplay()`, `sendFrame()`, and the ISR)
- `python/sensor_reader.py` — understand `ParkingFrame`, `FrameParser`,
  `SensorReader`, and `SensorSimulator`

**Simulator mode:** All Python scripts accept `--simulate`.  Use this to
understand the code before your lab session without hardware.

---

## 5. Lab Instructions

All students complete Steps 1–4 and Step 7.
Graduate students (5520) also complete Steps 5–6.

### 5.1 Step 1 — Distance Sensing

**Background:**
HC-SR04 converts time-of-flight to distance using the speed of sound
(0.034 cm/µs).  The Arduino `pulseIn()` function measures the ECHO pulse width
in microseconds.  Review Section 2.1 before proceeding.

**Setup:**
1. Flash `arduino/parking_sensor/parking_sensor.ino` to the Arduino Mega.
2. Wire the HC-SR04 (TRIG→pin 9, ECHO→pin 10, VCC, GND).
3. Open a terminal and `cd` to the `python/` directory.

**Run (hardware):**
```
python step1_distance.py
```
**Run (simulator):**
```
python step1_distance.py --simulate
```

**Expected output:**
```
Distance:   47.3 cm  Zone: Half Speed    Motor:  128/255  Alarm: Silent
Distance:   22.1 cm  Zone: Half Speed    Motor:  128/255  Alarm: Beeping
Distance:   11.6 cm  Zone: Stop          Motor:    0/255  Alarm: Continuous
```

**What to observe:**
- Move your hand toward and away from the HC-SR04 and watch the distance update.
- Verify the zone, motor, and alarm fields change at the correct thresholds.
- A reading of `0.0 cm` means no echo was received (object too far or not connected).

**Questions for report (Step 1):**
1. What is the maximum range of the HC-SR04, and why does `pulseIn()` have a
   30 000 µs timeout?  Calculate the corresponding maximum distance.
2. Why must the TRIG pulse be at least 10 µs wide?  What happens if it is shorter?
3. Why does the distance formula divide by 2?  What does this represent physically?

---

### 5.2 Step 2 — Motor Proportional Control

**Background:**
Proportional control maps the input (distance) to the output (motor speed) using
a fixed relationship — closer object → lower speed.  The L293D Half-H driver converts
the Arduino's PWM signal to motor voltage.  Review Sections 2.2 before proceeding.

**Setup:**
Wire the L293D (ENA→pin 6, IN1→pin 7, IN2→pin 8, VCC1→5V, VCC2→motor supply).

**Run:**
```
python step2_live_plot.py --simulate
```

**What to observe:**
- The top panel shows distance with zone color bands (red/orange/yellow/green).
- The dotted red line shows motor PWM dropping as distance decreases.
- The step function is characteristic of zone-based (not continuous) proportional
  control — this is intentional for this lab.

**Questions for report (Step 2):**
1. Sketch the expected PWM vs. distance curve.  Is this linear?  Why or why not?
2. If VCC2 is powered from 5V but the motor needs 9V, what would you expect to observe?  How would you fix it?
3. Why is data acquisition done in a background thread (separated from plotting)?

---

### 5.3 Step 3 — Motor Speed Mapping

**Objective:** Implement `distance_to_pwm()` in `step3_motor_map.py` to replicate
the Arduino zone logic in Python.

**Your task:** Open `step3_motor_map.py` and implement:

```python
def distance_to_pwm(distance_cm: float) -> int:
    # TODO: map distance_cm to motor PWM using the four-zone table
    raise NotImplementedError(...)
```

Zone table (same as Arduino firmware):

| Condition | PWM |
|-----------|-----|
| distance > 75 cm | 255 |
| 50 < distance ≤ 75 cm | 191 |
| 25 < distance ≤ 50 cm | 128 |
| distance ≤ 25 cm | 0 |

**Verify:**
```
python step3_motor_map.py --simulate
```
The step function plot should show the exact four-level staircase.

**With hardware:**
```
python step3_motor_map.py
```
The live comparison table should show ✓ (match) for every frame.

**Questions for report (Step 3):**
1. At exactly 50 cm, which zone does the Arduino assign?  Does your Python
   implementation agree?  Show the relevant code and explain.
2. What would change in both the Arduino and Python code to add a fifth zone
   (e.g., 75–100 cm at 75% speed)?
3. Why is zone-based control simpler to implement than continuous proportional
   control?  What are the trade-offs?

---

### 5.4 Step 4 — Buzzer Alarm Logic

**Objective:** Implement `classify_alarm()` in `step4_buzzer_alarm.py` to
replicate the Arduino buzzer alarm state logic in Python.

**Your task:** Open `step4_buzzer_alarm.py` and implement:

```python
def classify_alarm(distance_cm: float) -> int:
    # TODO: return 0 (silent), 1 (beeping), or 2 (continuous)
    raise NotImplementedError(...)
```

Alarm state table:

| Condition | State | Buzzer |
|-----------|-------|--------|
| distance ≥ 60 cm | 0 — Silent | Off |
| 20 ≤ distance < 60 cm | 1 — Beeping | Freq = map(dist, 20, 60, 2000, 500) |
| distance < 20 cm | 2 — Continuous | 2000 Hz constant |

**Verify:**
```
python step4_buzzer_alarm.py --simulate
```

**Questions for report (Step 4):**
1. At 60 cm exactly, what alarm state does your function return?  What about 19.9 cm?
2. Why does the beeping frequency *increase* as the object gets *closer*?  How
   does this affect the user experience?
3. Describe a scenario where a fixed-frequency beep (no frequency mapping) would
   be sufficient.  When is variable frequency important?

---

### 5.5 Step 5 — Serial Protocol Framing *(EECE 5520 only)*

**Objective:** Design and implement a binary framing protocol by completing
`encode_frame()` and `decode_frame()` in `step5_protocol.py`.

**Background:** Review Section 2.7 on binary framing, start bytes, and checksums.

**Your task:** Open `step5_protocol.py` and implement:
- `encode_frame(distance_cm, motor_pwm, alarm_state, zone_id) -> bytes`
- `decode_frame(data: bytes) -> Optional[ParkingFrame]`

Your protocol must produce exactly 7 bytes.  Document your design in your report:
- What start byte did you choose and why?
- What checksum algorithm? (XOR? Sum mod 256? CRC-8?)
- What happens if the start byte value appears in the payload?
- How would you make the protocol more robust?

**Test:**
```
python step5_protocol.py
```

Expected output:
```
[Step 5] Roundtrip ✓ PASS: 1000/1000
[Step 5] Corruption ✓ PASS: 999/1000 rejected (expected: ~100%)
```

**Questions for report (Step 5):**
1. Explain why a start byte alone is not sufficient for reliable framing.
   What additional mechanism makes the parser robust?
2. If the start byte `0xAA` appears as the high byte of the distance field,
   what would a naive parser do?  How would you fix this?
3. Compare XOR and sum-mod-256 checksums.  Which detects more error patterns?
   Which is simpler to implement?

---

### 5.6 Step 6 — Python Dashboard *(EECE 5520 only)*

**Objective:** Run the provided 4-panel matplotlib dashboard and observe real-time
system performance including protocol statistics.

**Prerequisite:** Step 5 must be complete.

**Run:**
```
python step6_dashboard.py --simulate
python step6_dashboard.py       # with hardware
```

**Panels:**
- **Top-left:** Rolling 30s distance with zone color bands
- **Top-right:** Motor PWM level over time (step function)
- **Bottom-left:** Alarm state timeline (color-coded: green/orange/red)
- **Bottom-right:** Protocol stats (frames OK, bad frames, frame rate, error rate)

**What to observe on hardware:**
- The protocol stats panel should show near-zero bad frames on a clean USB connection.
- Deliberately introduce noise (wiggle the USB cable, run other USB devices) and
  observe if bad frame counts increase.

**Questions for report (Step 6):**
1. What frame rate does the stats panel report?  Does it match the expected 10 Hz?
2. On real hardware, what causes checksum errors?  Did you observe any?
3. If the bad frame rate increased significantly, how would you modify the protocol
   to improve reliability?  Consider: longer preamble, CRC instead of XOR, frame
   sequence numbers.

---

### 5.7 Step 7 — Full System Demo *(provided, all students)*

**Overview:** `step7_demo.py` is a complete pygame visualization that integrates
all components:

- **Left panel:** Top-down parking view — car approaches wall, zone-colored
  background, motor speed indicator, alarm icon (flashing), LED matrix bar graph
- **Right panel:** Rolling distance plot + motor PWM bar

**Run (simulator):**
```
python step7_demo.py --simulate
```

**Run (hardware):**
```
python step7_demo.py
```

**Controls:**

| Key | Action |
|-----|--------|
| Q or ESC | Quit |

> **What to notice:** Watch the car rectangle move toward the wall as you approach
> with your hand.  The alarm icon flashes red in alarm zone 2 (< 20 cm).
> The LED matrix bars mirror what you would see on the MAX7219 hardware.

*No report questions required for Step 7.*

---

## 6. Deliverables

Submit **two items** on Blackboard:

1. **GitHub repository link** — a URL to your public (or instructor-accessible) repo
2. **PDF report** — see report requirements below

### Repository Structure (all students)

```
arduino/parking_sensor/parking_sensor.ino    (with your 4 functions implemented)
python/
├── sensor_reader.py          (provided — include unchanged)
├── step1_distance.py
├── step2_live_plot.py
├── step3_motor_map.py        (distance_to_pwm() implemented)
├── step4_buzzer_alarm.py     (classify_alarm() implemented)
├── step7_demo.py
└── requirements.txt
docs/
└── lab-writeup.md
```

### Repository Structure (5520 only, add to repo)

```
python/
├── step5_protocol.py         (encode_frame() + decode_frame() implemented)
└── step6_dashboard.py        (provided — include unchanged)
```

### Report (all students)
A single PDF containing:
- Cover page: name, student ID, course, date
- All questions from Steps 1–4 answered, with screenshots/plots
- Conclusion section

### Report (5520 only, add sections)
- Step 5–6 questions answered (protocol design + dashboard observations)
- Graduate Supplement (see Section 7)

---

## 7. Report Guidelines

### Format
- 11 pt font, single-spaced, 1-inch margins
- Figures must have captions and axis labels
- Code snippets may be included for critical sections only — do not copy entire files

### Required Plots
1. **Step 1:** Terminal screenshot showing distance, zone, motor, alarm values
2. **Step 2:** Screenshot of live distance + PWM plot during approach/recede
3. **Step 3:** PWM step function plot from `step3_motor_map.py`
4. **Step 4:** Alarm state bar chart from `step4_buzzer_alarm.py`
5. **Step 5** *(5520)*: Test harness output (roundtrip + corruption test results)
6. **Step 6** *(5520)*: Dashboard screenshot showing all 4 panels

### Conclusion Section
Address in 2–3 paragraphs:
- What you learned about ultrasonic sensing and its limitations
- How Timer1 interrupts improved the design over polling
- How the system could be extended (e.g., continuous proportional control,
  wireless streaming, camera-based parking assist)

### Graduate Supplement (5520 only)
Additional section "Graduate Analysis":
- Protocol design rationale: start byte choice, checksum algorithm, trade-offs
- Analysis of what happens when start byte appears in payload
- Comparison of XOR vs. CRC checksums for this application
- Proposed improvements to robustness (escape sequences, sequence numbers, etc.)

---

## Appendix A: Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| No port found | Arduino not connected or driver missing | Check Device Manager (Windows) or `ls /dev/tty.*` (macOS/Linux) |
| Handshake timeout | Wrong sketch or baud rate | Verify 115200; re-flash `parking_sensor.ino` |
| Distance always 0.0 | HC-SR04 not wired or wrong pins | Verify TRIG→9, ECHO→10, VCC, GND |
| Motor doesn't respond | VCC2 not powered | Ensure VCC2 is connected to motor supply voltage |
| Motor always full speed | ENA pin not connected or PWM not reaching L293D | Check pin 6 wiring to ENA |
| LCD shows nothing | Contrast pot at wrong extreme | Turn the 10 kΩ pot slowly — at one end chars disappear, at the other you see filled blocks. Adjust until text is visible. |
| LCD shows blocks but no text | Sketch not running or wrong pins | Verify pin defines in sketch match your wiring (RS=22, E=23, D4–D7=24–27). |
| LED matrix dark | SPI wiring wrong | Verify DIN→51, CLK→52, CS→53 |
| step6_dashboard.py fails | Step 5 not complete | Implement encode_frame/decode_frame first |
| pygame window doesn't appear | pygame not installed | `pip install pygame` |

---

## Appendix B: Key Arduino API Reference

| Function | Description |
|----------|-------------|
| `pulseIn(pin, HIGH, timeout)` | Measure HIGH pulse width in µs; returns 0 on timeout |
| `tone(pin, freq)` | Generate square wave on pin at freq Hz |
| `noTone(pin)` | Stop tone generation |
| `analogWrite(pin, value)` | Write PWM duty cycle 0–255 |
| `Timer1.initialize(µs)` | Set Timer1 period in microseconds |
| `Timer1.attachInterrupt(fn)` | Register ISR callback function |
| `lc.setRow(device, row, byte)` | Set one row of MAX7219 (0xFF = all on) |
| `lc.clearDisplay(device)` | Clear all MAX7219 LEDs |
| `lcd.begin(16, 2)` | Initialize 16×2 LCD (call once in setup) |
| `lcd.setCursor(col, row)` | Position LCD cursor |
| `lcd.print(value)` | Print to LCD at current cursor |
| `lcd.clear()` | Clear display (slow — avoid in ISR) |

---

*Lab writeup prepared for EECE 4520/5520 Spring 2026, University of Massachusetts Lowell.*
*Questions? Post on Piazza or visit office hours.*
