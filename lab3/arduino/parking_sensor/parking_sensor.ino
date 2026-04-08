/*
 * parking_sensor.ino  — STUDENT RELEASE SKELETON
 * EECE 4520/5520 — Microprocessor II and Embedded System Design
 * Spring 2026, UMass Lowell
 *
 * Smart Parking Sensor: HC-SR04 ultrasonic distance measurement drives a
 * DC motor (via L293D Half-H driver), a passive buzzer, a 16×2 parallel LCD, and a
 * MAX7219 8×8 SPI LED matrix.  Timer1 ISR fires every 100 ms to update the
 * display and motor without blocking the main loop measurement cycle.
 * A 7-byte binary frame is streamed to Python at 10 Hz after handshake.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * WIRING DIAGRAM
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *   HC-SR04 Ultrasonic Sensor
 *   ┌───────────────┐
 *   │ VCC   ────────┼──────► 5V
 *   │ GND   ────────┼──────► GND
 *   │ TRIG  ────────┼──────► Pin 9   (digital output)
 *   │ ECHO  ────────┼──────► Pin 10  (digital input)
 *   └───────────────┘
 *
 *   DC Motor + L293D Half-H Driver
 *   ┌───────────────┐
 *   │ ENA   ────────┼──────► Pin 6   (PWM — motor speed)
 *   │ IN1   ────────┼──────► Pin 7   (direction)
 *   │ IN2   ────────┼──────► Pin 8   (direction)
 *   │ VCC1  ────────┼──────► 5V      (logic supply)
 *   │ VCC2  ────────┼──────► motor supply (5V–12V)
 *   │ GND   ────────┼──────► GND (common with Arduino)
 *   └───────────────┘
 *
 *   16×2 Parallel LCD (HD44780, standard 16-pin header — NO I2C backpack)
 *   ┌───────────────┐
 *   │ Pin 1  VSS ───┼──────► GND
 *   │ Pin 2  VDD ───┼──────► 5V
 *   │ Pin 3  V0  ───┼──────► Middle pin of 10 kΩ contrast pot
 *   │               │        (pot ends: 5V and GND)
 *   │ Pin 4  RS  ───┼──────► Pin 22  (Register Select)
 *   │ Pin 5  RW  ───┼──────► GND     (always read-only from Arduino)
 *   │ Pin 6  E   ───┼──────► Pin 23  (Enable)
 *   │ Pin 7–10  ────┼──────  D0–D3: leave unconnected (4-bit mode)
 *   │ Pin 11 D4  ───┼──────► Pin 24
 *   │ Pin 12 D5  ───┼──────► Pin 25
 *   │ Pin 13 D6  ───┼──────► Pin 26
 *   │ Pin 14 D7  ───┼──────► Pin 27
 *   │ Pin 15 A   ───┼──────► 5V (backlight anode, via 220 Ω if needed)
 *   │ Pin 16 K   ───┼──────► GND (backlight cathode)
 *   └───────────────┘
 *   ⚠ NOTE: Adjust the contrast pot until characters appear.  At one extreme
 *     all characters disappear; at the other you see filled blocks on row 1.
 *
 *   Passive Buzzer
 *   ┌───────────────┐
 *   │ +     ────────┼──────► Pin 5   (tone() / noTone())
 *   │ −     ────────┼──────► GND
 *   └───────────────┘
 *
 *   MAX7219 8×8 LED Matrix (SPI)
 *   ┌───────────────┐
 *   │ VCC   ────────┼──────► 5V
 *   │ GND   ────────┼──────► GND
 *   │ DIN   ────────┼──────► Pin 51  (SPI MOSI)
 *   │ CLK   ────────┼──────► Pin 52  (SPI SCK)
 *   │ CS    ────────┼──────► Pin 53  (SPI SS — chip select)
 *   └───────────────┘
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * TIMER INTERRUPT NOTES
 * ─────────────────────────────────────────────────────────────────────────────
 *   Timer1 fires an ISR (Interrupt Service Routine) every 100 milliseconds.
 *   The ISR calls updateDisplay() and updateMotor().
 *
 *   VOLATILE KEYWORD:
 *   'g_distance' is declared volatile because it is written by the main loop
 *   and read by the ISR.  The volatile qualifier tells the compiler NOT to cache
 *   this variable in a CPU register — every access must go to RAM.  Without it,
 *   the compiler might optimise away "redundant" re-reads, so the ISR would see
 *   a stale value.
 *
 *   ISR CONSTRAINTS — never call inside an ISR:
 *     • Serial.print() — uses interrupts internally; calling from ISR → deadlock.
 *     • delay() — relies on Timer0 overflow interrupt; never returns from ISR.
 *     • Wire.h / I2C functions — the I2C hardware uses interrupts.  Calling
 *       Wire functions inside an ISR will deadlock.  This is why we use a
 *       parallel LCD (bit-bang GPIO) instead of an I2C LCD: LiquidCrystal
 *       only uses delayMicroseconds() (busy-wait) and is ISR-safe.
 *     • Any blocking or long-running function — ISRs must execute quickly.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * BINARY SERIAL PROTOCOL (5520 skill builder)
 * ─────────────────────────────────────────────────────────────────────────────
 *   7-byte frame, sent at 10 Hz after 's'/'r' handshake:
 *     Byte 0:   0xAA  (start marker)
 *     Bytes 1-2: distance × 10, uint16 big-endian
 *     Byte 3:   motor_pwm  (0–255)
 *     Byte 4:   alarm_state  (0=silent, 1=beeping, 2=continuous)
 *     Byte 5:   zone_id  (0–3)
 *     Byte 6:   checksum = XOR of bytes 1–5
 */

#include <LiquidCrystal.h>       // Built-in Arduino library — 4-bit parallel LCD
#include <LedControl.h>          // MAX7219 SPI LED matrix driver
#include <TimerOne.h>            // Hardware Timer1 ISR

// ─── Pin Assignments ─────────────────────────────────────────────────────────
#define TRIG_PIN     9
#define ECHO_PIN    10
#define ENA_PIN      6    // PWM speed control (L293D ENA — no jumper needed)
#define IN1_PIN      7
#define IN2_PIN      8
#define BUZZER_PIN   5
// MAX7219 SPI: DIN=51, CLK=52, CS=53

// LCD parallel interface pins (Mega Port A — no conflicts with other peripherals)
#define LCD_RS  22
#define LCD_E   23
#define LCD_D4  24
#define LCD_D5  25
#define LCD_D6  26
#define LCD_D7  27

// ─── Peripheral objects ───────────────────────────────────────────────────────
LedControl lc = LedControl(51, 52, 53, 1);
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ─── Serial ───────────────────────────────────────────────────────────────────
#define BAUD_RATE             115200
#define STREAM_INTERVAL_MS     100

// ─── Zone thresholds (cm) ────────────────────────────────────────────────────
#define ZONE0_THRESHOLD  75
#define ZONE1_THRESHOLD  50
#define ZONE2_THRESHOLD  25
#define BUZZER_SILENT_THRESHOLD  60
#define BUZZER_ALARM_THRESHOLD   20

// ─── Volatile global — shared between loop() and Timer1 ISR ──────────────────
volatile float g_distance = 0.0f;

unsigned long lastStreamTime = 0;
bool          streaming      = false;

// ─── Prototypes ───────────────────────────────────────────────────────────────
void         initPeripherals();
bool         waitForHandshake();
float        measureDistance();
void         updateMotor(float dist);
void         updateBuzzer(float dist);
void         updateDisplay(float dist);
void         sendFrame(float dist);
int          getZone(float dist);
const char*  getSpeedLevel(int zone);
int          getPWM(int zone);
int          getAlarmState(float dist);
void         isrCallback();


// =============================================================================
// setup()  — provided
// =============================================================================
void setup() {
  Serial.begin(BAUD_RATE);
  delay(2200);

  initPeripherals();

  float testDist = measureDistance();
  Serial.print("Sensor check: ");
  Serial.print(testDist, 1);
  Serial.println(" cm (0.0 = not connected)");

  waitForHandshake();

  streaming = true;
  lastStreamTime = millis();

  Timer1.initialize(100000);
  Timer1.attachInterrupt(isrCallback);
}


// =============================================================================
// loop()  — provided
// =============================================================================
void loop() {
  if (!streaming) return;
  unsigned long now = millis();
  if (now - lastStreamTime < STREAM_INTERVAL_MS) return;
  lastStreamTime = now;

  float dist = measureDistance();
  g_distance = dist;

  updateBuzzer(dist);
  sendFrame(dist);
}


// =============================================================================
// isrCallback()  — Timer1 ISR, fires every 100 ms
// ⚠ Do NOT add Serial.print or delay() here.
// =============================================================================
void isrCallback() {
  float d = g_distance;
  updateDisplay(d);
  updateMotor(d);
}


// =============================================================================
// waitForHandshake()  — provided
// =============================================================================
bool waitForHandshake() {
  while (true) {
    if (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == 's') {
        Serial.write('r');
        Serial.println();
        return true;
      }
    }
  }
}


// =============================================================================
// updateDisplay()  — provided
// =============================================================================
void updateDisplay(float dist) {
  int zone = getZone(dist);

  lcd.setCursor(0, 0);
  lcd.print("Dist:  ");
  lcd.print(dist, 1);
  lcd.print(" cm  ");

  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  lcd.print(getSpeedLevel(zone));
  lcd.print("     ");

  long dClamped = constrain((long)dist, 0L, 100L);
  int  rowsLit  = (int)map(dClamped, 0L, 100L, 8L, 0L);
  for (int row = 0; row < 8; row++) {
    lc.setRow(0, row, (row < rowsLit) ? 0xFF : 0x00);
  }
}


// =============================================================================
// sendFrame()  — provided
// =============================================================================
void sendFrame(float dist) {
  int zone  = getZone(dist);
  uint8_t pwm   = (uint8_t)getPWM(zone);
  uint8_t alarm = (uint8_t)getAlarmState(dist);

  uint16_t d10 = (uint16_t)(dist * 10.0f + 0.5f);
  uint8_t payload[5];
  payload[0] = (uint8_t)(d10 >> 8);
  payload[1] = (uint8_t)(d10 & 0xFF);
  payload[2] = pwm;
  payload[3] = alarm;
  payload[4] = (uint8_t)zone;

  uint8_t checksum = 0;
  for (int i = 0; i < 5; i++) checksum ^= payload[i];

  Serial.write(0xAA);
  Serial.write(payload, 5);
  Serial.write(checksum);
}


// =============================================================================
// getZone()  — provided
// =============================================================================
int getZone(float dist) {
  if (dist > ZONE0_THRESHOLD) return 0;
  if (dist > ZONE1_THRESHOLD) return 1;
  if (dist > ZONE2_THRESHOLD) return 2;
  return 3;
}


// =============================================================================
// getSpeedLevel()  — provided
// =============================================================================
const char* getSpeedLevel(int zone) {
  switch (zone) {
    case 0: return "Full";
    case 1: return "3/4";
    case 2: return "Half";
    default: return "Stop";
  }
}


// =============================================================================
// getPWM()  — provided helper
// =============================================================================
int getPWM(int zone) {
  switch (zone) {
    case 0: return 255;
    case 1: return 191;
    case 2: return 128;
    default: return 0;
  }
}


// =============================================================================
// getAlarmState()  — provided helper
// =============================================================================
int getAlarmState(float dist) {
  if (dist >= BUZZER_SILENT_THRESHOLD) return 0;
  if (dist >= BUZZER_ALARM_THRESHOLD)  return 1;
  return 2;
}


// =============================================================================
// ─────────────────────────────────────────────────────────────────────────────
//  STUDENT TODO SECTION — implement the four functions below
// ─────────────────────────────────────────────────────────────────────────────
// =============================================================================


// =============================================================================
// measureDistance()  ← TODO: implement
//
// Trigger the HC-SR04 and measure the echo pulse width. Convert to centimetres.
//
// Steps:
//   1. Bring TRIG LOW, wait 2 µs
//   2. Bring TRIG HIGH for 10 µs, then LOW
//   3. Call pulseIn(ECHO_PIN, HIGH, 30000) — returns duration in µs
//   4. If duration == 0, return 0.0 (no echo / out-of-range)
//   5. Return duration * 0.034 / 2  (distance in cm)
//
// Hint: delayMicroseconds(n) waits n microseconds.
//       pulseIn(pin, HIGH, timeout) measures pulse width in µs.
// =============================================================================
float measureDistance() {
  // TODO: implement HC-SR04 pulse measurement
  while (true) {
    // Halt with diagnostic message if not implemented
    Serial.println("ERROR: measureDistance() not implemented. See TODO.");
    delay(1000);
  }
  return 0.0f;  // never reached
}


// =============================================================================
// updateMotor()  ← TODO: implement
//
// Set motor speed (ENA PWM) and direction (IN1, IN2) based on distance zone.
//
// Direction: always forward (IN1=HIGH, IN2=LOW)
// Speed:
//   Zone 0 (> 75 cm):  analogWrite(ENA_PIN, 255)   ← Full
//   Zone 1 (50–75 cm): analogWrite(ENA_PIN, 191)   ← 3/4
//   Zone 2 (25–50 cm): analogWrite(ENA_PIN, 128)   ← Half
//   Zone 3 (0–25 cm):  analogWrite(ENA_PIN, 0)     ← Stop
//
// Hint: call getZone(dist) to get the zone, getPWM(zone) to get the PWM value.
//       analogWrite(pin, value) sets PWM; 0=off, 255=full.
// =============================================================================
void updateMotor(float dist) {
  // TODO: set IN1, IN2 direction and analogWrite PWM to ENA_PIN
  (void)dist;  // suppress unused-variable warning
}


// =============================================================================
// updateBuzzer()  ← TODO: implement
//
// Drive the passive buzzer based on alarm state derived from distance.
//
//   dist >= 60 cm → noTone(BUZZER_PIN)   (silent)
//   20 <= dist < 60 cm → tone(BUZZER_PIN, map(dist, 20, 60, 2000, 500))
//                         (closer → higher frequency, range 500–2000 Hz)
//   dist < 20 cm → tone(BUZZER_PIN, 2000)  (continuous alarm)
//
// Hint: map(value, fromLow, fromHigh, toLow, toHigh) linearly maps value.
//       tone(pin, freq) starts a tone; noTone(pin) stops it.
// =============================================================================
void updateBuzzer(float dist) {
  // TODO: implement three-state buzzer logic
  (void)dist;  // suppress unused-variable warning
}


// =============================================================================
// initPeripherals()  ← TODO: implement
//
// Initialize all hardware. Called once from setup().
//
// Required tasks:
//   1. pinMode: TRIG_PIN OUTPUT, ECHO_PIN INPUT
//               ENA_PIN OUTPUT, IN1_PIN OUTPUT, IN2_PIN OUTPUT
//               BUZZER_PIN OUTPUT
//   2. Initial states: TRIG LOW, IN1 LOW, IN2 LOW, analogWrite(ENA_PIN, 0)
//   3. LCD (parallel — use built-in LiquidCrystal, NOT LiquidCrystal_I2C):
//           lcd.begin(16, 2);   ← no lcd.init() or lcd.backlight() needed
//           lcd.setCursor(0,0); lcd.print("Parking Sensor");
//           lcd.setCursor(0,1); lcd.print("Initializing...");
//   4. LED matrix: lc.shutdown(0, false);  ← wake from power save
//                  lc.setIntensity(0, 8);  ← brightness 0–15
//                  lc.clearDisplay(0);     ← all LEDs off
//
// Note: Timer1.initialize() and Timer1.attachInterrupt() are called in setup()
//   AFTER initPeripherals() — do NOT call them here.
// =============================================================================
void initPeripherals() {
  // TODO: configure pins and initialize LCD + LED matrix
}
