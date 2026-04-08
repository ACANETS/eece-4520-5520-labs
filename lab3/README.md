# Lab 3: Smart Parking Sensor — Student Release
## EECE 4520/5520 — Microprocessor II and Embedded System Design
### Spring 2026 | University of Massachusetts Lowell

---

## Overview

In this lab you will build a smart parking assistant that:
- Measures distance with an **HC-SR04** ultrasonic sensor
- Controls a **DC motor** speed proportionally (via L293D Half-H driver)
- Displays distance + speed level on a **16×2 I2C LCD**
- Shows proximity on a **MAX7219 8×8 LED matrix** (SPI)
- Sounds a **passive buzzer** with frequency proportional to proximity
- Uses **Timer1 hardware interrupts** for periodic 100 ms display updates
- (5520 only) Streams data via a **binary serial protocol** to a Python dashboard

Full instructions are in [`docs/lab-writeup.md`](docs/lab-writeup.md).

---

## Step Table

| Step | Title | Who | Student Task |
|------|-------|-----|-------------|
| 1 | Distance Sensing | All | — (run provided) |
| 2 | Motor Proportional Control | All | — (observe) |
| 3 | Motor Speed Mapping | All | `distance_to_pwm()` in `step3_motor_map.py` |
| 4 | Buzzer Alarm Logic | All | `classify_alarm()` in `step4_buzzer_alarm.py` |
| 5 | Serial Protocol Framing | **5520 only** | `encode_frame()` + `decode_frame()` in `step5_protocol.py` |
| 6 | Python Dashboard | **5520 only** | — (run with your Step 5 protocol) |
| 7 | Full System Demo | All | — (provided demo) |

---

## What You Need to Implement

### Arduino: 4 functions in `arduino/parking_sensor/parking_sensor.ino`
- `measureDistance()` — HC-SR04 pulse timing and distance formula
- `updateMotor(float dist)` — L293D PWM speed control
- `updateBuzzer(float dist)` — tone/noTone alarm logic
- `initPeripherals()` — all GPIO, LCD, LED matrix initialization

### Python (all students):
- `distance_to_pwm(distance_cm)` in `step3_motor_map.py`
- `classify_alarm(distance_cm)` in `step4_buzzer_alarm.py`

### Python (5520 only):
- `encode_frame(...)` in `step5_protocol.py`
- `decode_frame(data)` in `step5_protocol.py`

---

## Quick Start

### 1. Install Arduino Libraries

In Arduino IDE → Tools → Manage Libraries, install:
- `LiquidCrystal I2C` (by Frank de Brabander)
- `LedControl` (by Eberhard Fahle)
- `TimerOne`

### 2. Flash the Arduino Sketch

Open `arduino/parking_sensor/parking_sensor.ino`, select **Arduino Mega 2560**,
and flash.

### 3. Wire Hardware

| Component | Signal | Arduino Mega Pin |
|-----------|--------|-----------------|
| HC-SR04 | TRIG | 9 |
| HC-SR04 | ECHO | 10 |
| L293D | ENA | 6 |
| L293D | IN1 | 7 |
| L293D | IN2 | 8 |
| Buzzer | + | 5 |
| LCD | SDA | 20 |
| LCD | SCL | 21 |
| MAX7219 | DIN | 51 |
| MAX7219 | CLK | 52 |
| MAX7219 | CS | 53 |

> ℹ️ L293D has no ENA jumper — connect ENA directly to pin 6 for PWM speed control.

### 4. Set Up Python

```bash
cd python
python3 -m venv venv
source venv/bin/activate      # macOS/Linux
# OR: venv\Scripts\activate   # Windows
pip install -r requirements.txt
```

### 5. Run Steps (use --simulate if no hardware)

```bash
python step1_distance.py --simulate
python step2_live_plot.py --simulate
python step3_motor_map.py --simulate   # after implementing distance_to_pwm()
python step4_buzzer_alarm.py --simulate   # after implementing classify_alarm()
python step5_protocol.py               # 5520 only, after implementing encode/decode
python step6_dashboard.py --simulate   # 5520 only
python step7_demo.py --simulate
```

---

## Deliverables

Submit on Blackboard:
1. **GitHub repository link** — your fork/submission with all implementations
2. **PDF report** — answer all questions in `docs/lab-writeup.md`

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Distance always 0.0 | Check TRIG→9, ECHO→10, VCC, GND on HC-SR04 |
| Motor doesn't respond | Check VCC2 power supply; verify IN1→7, IN2→8, ENA→6 |
| LCD shows nothing | Try I2C address 0x3F in sketch header |
| LED matrix dark | Verify DIN→51, CLK→52, CS→53 |
| Handshake timeout | Re-flash sketch; close Serial Monitor; check baud 115200 |
| step6 ImportError | Complete Step 5 first (encode_frame/decode_frame) |
| pygame missing | `pip install pygame` |
