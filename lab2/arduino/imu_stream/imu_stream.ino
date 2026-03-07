/*
 * imu_stream.ino
 * EECE 4520/5520 — Microprocessor II and Embedded System Design
 * Spring 2026, UMass Lowell
 *
 * YOUR TASK: Implement raw I2C communication with the MPU-6050 IMU and
 * stream calibrated sensor data over USB-Serial at 50 Hz.
 *
 * Hardware:
 *   Arduino Mega 2560
 *   MPU-6050 breakout: SDA → pin 20, SCL → pin 21
 *
 * Output format (115200 baud):
 *   timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n
 *
 * Handshake (already implemented — do not modify):
 *   Python sends 's' → Arduino replies "IMU_READY\n" → begins streaming
 *
 * ─── What you need to implement ─────────────────────────────────────────────
 *   1. writeRegister()  — write one byte to an I2C register
 *   2. readRegister()   — read  one byte from an I2C register   (bonus check)
 *   3. readAndSendIMU() — burst-read 14 bytes, convert, send CSV
 *
 * Everything else (setup, loop, initMPU6050, Serial handshake) is provided.
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * ─── I2C Background ──────────────────────────────────────────────────────────
 * I2C uses two wires: SDA (data) and SCL (clock).  Every transaction starts
 * with a START condition and ends with a STOP condition.
 *
 * Register WRITE (set a configuration register):
 *   [START] → [device addr + W] → [register addr] → [data byte] → [STOP]
 *
 * Register READ (read back a value):
 *   [START] → [device addr + W] → [register addr] → [repeated START]
 *           → [device addr + R] → [data byte] → [NACK] → [STOP]
 *   The "repeated START" (endTransmission(false)) keeps the bus open so the
 *   master can immediately switch to a read without releasing the bus.
 *
 * Burst READ (read N consecutive registers in one transaction):
 *   Same as register read, but requestFrom() asks for N bytes and the
 *   MPU-6050 auto-increments its internal register pointer after each byte.
 *
 * Wire library quick reference:
 *   Wire.beginTransmission(addr)          — open tx to device at 7-bit addr
 *   Wire.write(byte)                      — queue a byte to send
 *   Wire.endTransmission(true)            — send queued bytes + STOP
 *   Wire.endTransmission(false)           — send queued bytes + repeated START
 *   Wire.requestFrom(addr, count, true)   — request `count` bytes, then STOP
 *   Wire.available()                      — number of bytes waiting to be read
 *   Wire.read()                           — read and return the next byte
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Wire.h>

// ─── MPU-6050 I2C Address ────────────────────────────────────────────────────
#define MPU6050_ADDR     0x68   // AD0 pin pulled LOW → address 0x68

// ─── MPU-6050 Register Addresses (from datasheet) ────────────────────────────
#define REG_WHO_AM_I     0x75   // Device identity — always reads 0x68
#define REG_PWR_MGMT_1   0x6B   // Power management — write 0x00 to wake device
#define REG_SMPLRT_DIV   0x19   // Sample rate divider
#define REG_CONFIG       0x1A   // Digital low-pass filter (DLPF) config
#define REG_GYRO_CONFIG  0x1B   // Gyroscope full-scale range select
#define REG_ACCEL_CONFIG 0x1C   // Accelerometer full-scale range select
#define REG_ACCEL_XOUT_H 0x3B   // First byte of the 14-byte sensor data block

// ─── Sensor Sensitivity (at default full-scale range) ────────────────────────
// Accelerometer ±2 g  → 16384 LSB per g
// Gyroscope     ±250°/s → 131 LSB per °/s
// Dividing the raw 16-bit integer by these constants gives physical units.
#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY    131.0f

// ─── Streaming Rate ───────────────────────────────────────────────────────────
#define PERIOD_MS 20UL   // 20 ms between samples = 50 Hz output rate

// ─── Global State ─────────────────────────────────────────────────────────────
unsigned long lastTime = 0;
bool streaming = false;

// ─── Function Prototypes ──────────────────────────────────────────────────────
void    initMPU6050();
void    writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void    readAndSendIMU();


// =============================================================================
// setup()
// Initialises Serial, Wire, the MPU-6050, and waits for the Python handshake.
// PROVIDED — do not modify.
// =============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(100);          // let power rails stabilise

  initMPU6050();       // configure the sensor (see below)

  // Handshake: hold here until Python sends 's'
  Serial.println("WAITING");
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("IMU_READY");
        streaming = true;
        lastTime  = millis();
        break;
      }
    }
  }
}


// =============================================================================
// loop()
// Calls readAndSendIMU() at PERIOD_MS intervals.
// PROVIDED — do not modify.
// =============================================================================
void loop() {
  if (!streaming) return;

  unsigned long now = millis();
  if (now - lastTime >= PERIOD_MS) {
    lastTime = now;
    readAndSendIMU();
  }
}


// =============================================================================
// initMPU6050()
// Wakes the device and sets the sample rate, DLPF, and full-scale ranges.
// PROVIDED — do not modify.  Study each register write to understand the config.
// Two lines at the bottom are TODO — set the gyro and accel full-scale ranges.
// =============================================================================
void initMPU6050() {
  // Wake device: PWR_MGMT_1 = 0x00 clears the SLEEP bit (device starts asleep)
  writeRegister(REG_PWR_MGMT_1, 0x00);
  delay(10);

  // Verify device identity: WHO_AM_I (0x75) must return 0x68.
  // This call exercises your readRegister() implementation.
  // If it halts here, check wiring (SDA→pin20, SCL→pin21) and your readRegister().
  uint8_t whoAmI = readRegister(REG_WHO_AM_I);
  if (whoAmI != 0x68) {
    Serial.print("ERROR: WHO_AM_I = 0x");
    Serial.print(whoAmI, HEX);
    Serial.println(" (expected 0x68). Check wiring: SDA→pin20, SCL→pin21.");
    while (true);   // halt
  }

  // Sample Rate Divider: SMPLRT_DIV = 0 → internal 8 kHz gyro clock / (1+0)
  // We gate the output to 50 Hz in software via PERIOD_MS.
  writeRegister(REG_SMPLRT_DIV, 0x00);

  // DLPF: CONFIG = 3 → 44 Hz bandwidth (filters high-frequency vibration noise)
  writeRegister(REG_CONFIG, 0x03);

  // TODO: Set Gyroscope full-scale range.
  // Hint: Write to REG_GYRO_CONFIG. Bits [4:3] select the range:
  //   0b00 (0x00) → ±250 °/s  ← use this (matches GYRO_SENSITIVITY = 131 LSB/°/s)
  //   0b01 (0x08) → ±500 °/s
  //   0b10 (0x10) → ±1000 °/s
  //   0b11 (0x18) → ±2000 °/s

  // TODO: Set Accelerometer full-scale range.
  // Hint: Write to REG_ACCEL_CONFIG. Bits [4:3] select the range:
  //   0b00 (0x00) → ±2 g  ← use this (matches ACCEL_SENSITIVITY = 16384 LSB/g)
  //   0b01 (0x08) → ±4 g
  //   0b10 (0x10) → ±8 g
  //   0b11 (0x18) → ±16 g
}


// =============================================================================
// writeRegister()
// Write a single byte to an MPU-6050 register via I2C.
//
// TODO: Implement this function using the Wire library.
//
// Steps:
//   1. Call Wire.beginTransmission(MPU6050_ADDR)
//        Opens a transmission to the device at address MPU6050_ADDR.
//   2. Call Wire.write(reg)
//        Sends the register address you want to write to.
//   3. Call Wire.write(value)
//        Sends the data byte.
//   4. Call Wire.endTransmission(true)
//        Sends a STOP condition, completing the transaction.
//
// Refer to the Wire library quick reference in the header comment above.
// =============================================================================
void writeRegister(uint8_t reg, uint8_t value) {
  // TODO: implement I2C register write
}


// =============================================================================
// readRegister()
// Read a single byte from an MPU-6050 register via I2C.
//
// TODO: Implement this function using the Wire library.
//
// Steps:
//   1. Wire.beginTransmission(MPU6050_ADDR)
//        Open a write-phase transaction to set the register pointer.
//   2. Wire.write(reg)
//        Send the register address you want to read from.
//   3. Wire.endTransmission(false)
//        Send a REPEATED START (not STOP) — this keeps the bus open so you
//        can immediately switch to read mode without another arbitration.
//   4. Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1, (uint8_t)true)
//        Ask for 1 byte from the device, then release the bus (STOP).
//   5. Return Wire.read() if Wire.available() > 0, otherwise return 0.
//
// Note: the casts to uint8_t in requestFrom() avoid an ambiguous-overload
// compiler warning on some Arduino cores.
// =============================================================================
uint8_t readRegister(uint8_t reg) {
  // TODO: implement I2C register read
  return 0;
}


// =============================================================================
// readAndSendIMU()
// Burst-reads all 14 sensor bytes from the MPU-6050, converts them to
// physical units, and sends one CSV line over Serial.
//
// Register block layout (big-endian 16-bit signed, starting at 0x3B):
//   Offset  Register     Contents
//   0-1     0x3B-0x3C    ACCEL_XOUT  (high byte first)
//   2-3     0x3D-0x3E    ACCEL_YOUT
//   4-5     0x3F-0x40    ACCEL_ZOUT
//   6-7     0x41-0x42    TEMP_OUT    ← read but not used
//   8-9     0x43-0x44    GYRO_XOUT
//   10-11   0x45-0x46    GYRO_YOUT
//   12-13   0x47-0x48    GYRO_ZOUT
//
// TODO Part A — Burst read (I2C):
//   1. Open a write transaction and send REG_ACCEL_XOUT_H to set the register
//      pointer, then issue a REPEATED START:
//        Wire.beginTransmission(MPU6050_ADDR);
//        Wire.write(REG_ACCEL_XOUT_H);
//        Wire.endTransmission(false);          // repeated START, keep bus
//   2. Request 14 bytes with a STOP at the end:
//        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
//   3. Guard against incomplete reads:
//        if (Wire.available() < 14) return;
//   4. Read all 14 bytes into a buffer:
//        uint8_t buf[14];
//        for (int i = 0; i < 14; i++) { buf[i] = Wire.read(); }
//
// TODO Part B — Combine bytes into 16-bit signed integers:
//   Each sensor value spans two consecutive bytes, high byte first, for example:
//        int16_t rawAX = (int16_t)((buf[0] << 8) | buf[1]);
//        int16_t rawAY = ...
//        int16_t rawAZ = ...
//        // buf[6], buf[7] = temperature — skip
//        int16_t rawGX = ...
//        int16_t rawGY = ...
//        int16_t rawGZ = ...
//   Question: Why do we need to cast? 
//
// TODO Part C — Convert to physical units, for example:
//        float ax = rawAX / ACCEL_SENSITIVITY;  // units: g
//        float ay = ...
//        float az = ...
//
//        float gx = rawGX / GYRO_SENSITIVITY;   // units: °/s
//        float gy = ...
//        float gz = ...
//
// TODO: The Serial output should emit one CSV line with the timestamp and all six sensor values:
// Once you have ax..gz defined and directed to your PC, it will stream correctly.
// =============================================================================
void readAndSendIMU() {
  // TODO Part A: burst-read 14 bytes via I2C (see instructions above)

  // TODO Part B: combine high/low bytes into int16_t raw values

  // TODO Part C: convert raw values to physical units (float ax, ay, az, gx, gy, gz)

  // TODO Part D: Serial output 
  // Emits one CSV line: timestamp_ms,ax,ay,az,gx,gy,gz
  Serial.print(millis());
  Serial.print(',');
  Serial.print(ax, 4);
  //...
}
