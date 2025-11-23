/*
  Self-Balancing (Reaction-Wheel) Robot
  - Uses MPU6050 (pitch around X-gyro, accel Y/Z)
  - Float math (UNO/Nano: float == double)
  - Complementary filter + PID with anti-windup
  - Serial tuning + quick invert toggles

  Serial commands (send in Serial Monitor, 115200 baud):
    p<val>   -> set Kp (e.g., p18.5)
    i<val>   -> set Ki (e.g., i0.10)
    d<val>   -> set Kd (e.g., d0.9)
    s<val>   -> set setpoint angle in deg (e.g., s0)
    r        -> reset integral
    c        -> recalibrate MPU (keep robot steady)
    m        -> toggle motor invert
    g        -> toggle gyro invert
*/

#include <Wire.h>

// ------------ Pins (TB6612FNG, using B-channel only here) ------------
const int PWMB = 5;   // PWM (speed)
const int BIN1 = 3;   // Direction 1
const int BIN2 = 8;   // Direction 2

// ------------ MPU6050 ------------
const uint8_t MPU_ADDR     = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

// ------------ Filter / control options ------------
float CF_GYRO_WEIGHT = 0.98f;   // Complementary filter (gyro)
float CF_ACC_WEIGHT  = 0.02f;   // Complementary filter (accel)

// If the wheel moves the wrong direction, flip ONE of these (start with MOTOR_INVERT)
bool MOTOR_INVERT = false;      // flips output sign before sending to motor
bool GYRO_INVERT  = false;      // flips gyro rate sign

// ------------ PID gains (start here and tune) ------------
float Kp = 45.0f;
float Ki = 1.80f;
float Kd = 0.80f;

// ------------ PID/State variables ------------
float setpoint   = 0.0f;   // upright angle target (deg)
float angle      = 0.0f;   // filtered angle (deg)
float gyroRate   = 0.0f;   // deg/s
float input      = 0.0f;   // = angle
float output     = 0.0f;   // PID output (mapped to motor command)
float lastInput  = 0.0f;
float integral   = 0.0f;
unsigned long lastPIDms = 0;

// Time base
unsigned long lastLoopMs = 0;
float dt = 0.0f;

// ------------ Sensor raw/offsets ------------
float accelX = 0, accelY = 0, accelZ = 0;
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
float gyroXOffset  = 0, gyroYOffset  = 0, gyroZOffset  = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor pins
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  stopMotor();

  initMPU6050();

  Serial.println("Calibrating MPU6050... Keep robot steady.");
  calibrateMPU6050();
  Serial.println("Calibration complete.");

  lastLoopMs = millis();
  lastPIDms  = millis();

  Serial.println("READY. Send p/i/d/s/r/c (or m/g to invert) via Serial.");
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // ---- dt in seconds
  unsigned long now = millis();
  dt = (now - lastLoopMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f; // guard
  lastLoopMs = now;

  // ---- Read sensors + compute angle
  readMPU6050();
  calculateAngle();       // updates global 'angle'

  // ---- PID control (run every ~10ms)
  if ((now - lastPIDms) >= 10) {
    input = angle;
    float error = setpoint - input;

    // integral with anti-windup
    integral += error * ((now - lastPIDms) / 1000.0f);
    integral = constrain(integral, -100.0f, 100.0f);

    // derivative (on measurement to reduce noise kick)
    float derivative = (input - lastInput) / ((now - lastPIDms) / 1000.0f);

    output = Kp * error + Ki * integral - Kd * derivative; // note minus on derivative of measurement

    // Constrain to motor range
    output = constrain(output, -255.0f, 255.0f);

    lastInput = input;
    lastPIDms = now;

    // ---- Drive motor
    setMotor(output);
  }

  // ---- Serial debug (about every 100ms)
  if ((now % 100) < 8) {
    Serial.print("ang=");
    Serial.print(angle, 2);
    Serial.print("  out=");
    Serial.print(output, 1);
    Serial.print("  Kp=");
    Serial.print(Kp, 2);
    Serial.print(" Ki=");
    Serial.print(Ki, 3);
    Serial.print(" Kd=");
    Serial.print(Kd, 2);
    Serial.print("  invM=");
    Serial.print(MOTOR_INVERT);
    Serial.print(" invG=");
    Serial.println(GYRO_INVERT);
  }

  // ---- Handle serial tuning / commands
  serialTuning();
}

// -------------------- MPU6050 SETUP --------------------
void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);          // wake up
  Wire.endTransmission(true);

  // ACCEL_CONFIG: ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // GYRO_CONFIG: ±250 dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
}

// -------------------- CALIBRATION --------------------
void calibrateMPU6050() {
  const int N = 1000;
  long ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

  for (int i=0; i<N; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t axr = (Wire.read() << 8) | Wire.read();
    int16_t ayr = (Wire.read() << 8) | Wire.read();
    int16_t azr = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temp
    int16_t gxr = (Wire.read() << 8) | Wire.read();
    int16_t gyr = (Wire.read() << 8) | Wire.read();
    int16_t gzr = (Wire.read() << 8) | Wire.read();

    ax += axr; ay += ayr; az += azr;
    gx += gxr; gy += gyr; gz += gzr;
    delay(2);
  }

  accelXOffset = (float)ax / N;
  accelYOffset = (float)ay / N;
  // 1g on Z at rest in ±2g -> 16384 LSB
  accelZOffset = ((float)az / N) - 16384.0f;

  gyroXOffset  = (float)gx / N;
  gyroYOffset  = (float)gy / N;
  gyroZOffset  = (float)gz / N;
}

// -------------------- SENSOR READ --------------------
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t axr = (Wire.read() << 8) | Wire.read();
  int16_t ayr = (Wire.read() << 8) | Wire.read();
  int16_t azr = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t gxr = (Wire.read() << 8) | Wire.read();
  int16_t gyr = (Wire.read() << 8) | Wire.read();
  int16_t gzr = (Wire.read() << 8) | Wire.read();

  // Convert to g's (±2g -> 16384 LSB/g)
  accelX = (axr - accelXOffset) / 16384.0f;
  accelY = (ayr - accelYOffset) / 16384.0f;
  accelZ = (azr - accelZOffset) / 16384.0f;

  // Pitch control uses gyro X (deg/s), ±250 dps -> 131 LSB/(deg/s)
  float rate = (gxr - gyroXOffset) / 131.0f;
  gyroRate = GYRO_INVERT ? -rate : rate;
}

// -------------------- ANGLE (COMPLEMENTARY FILTER) --------------------
void calculateAngle() {
  // Pitch from accelerometer: atan2(Y, Z)
  float accelAngle = atan2f(accelY, accelZ) * 57.2957795f; // rad->deg
  // Fuse with gyro
  angle = CF_GYRO_WEIGHT * (angle + gyroRate * dt)
        + CF_ACC_WEIGHT  * accelAngle;
}

// -------------------- PID -> MOTOR --------------------
void setMotor(float cmd) {
  // Optional deadband to avoid jitter
  if (fabs(cmd) < 5.0f) {
    stopMotor();
    return;
  }

  if (MOTOR_INVERT) cmd = -cmd;

  int speed = (int)constrain(fabs(cmd), 0.0f, 255.0f);

  if (cmd > 0) {
    // Forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else {
    // Backward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed);
  }
}

void stopMotor() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

// -------------------- SERIAL TUNING --------------------
void serialTuning() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'p': {
      float v = Serial.parseFloat();
      if (v != 0 || Serial.peek() == '\n' || Serial.peek() == '\r')
        Kp = v;
      Serial.print("Kp=");
      Serial.println(Kp, 3);
      break;
    }
    case 'i': {
      float v = Serial.parseFloat();
      Ki = v;
      Serial.print("Ki=");
      Serial.println(Ki, 4);
      break;
    }
    case 'd': {
      float v = Serial.parseFloat();
      Kd = v;
      Serial.print("Kd=");
      Serial.println(Kd, 3);
      break;
    }
    case 's': {
      float v = Serial.parseFloat();
      setpoint = v;
      Serial.print("setpoint=");
      Serial.println(setpoint, 2);
      break;
    }
    case 'r': {
      integral = 0;
      Serial.println("Integral reset.");
      break;
    }
    case 'c': {
      Serial.println("Recalibrating... keep steady.");
      calibrateMPU6050();
      Serial.println("Calibration done.");
      break;
    }
    case 'm': {
      MOTOR_INVERT = !MOTOR_INVERT;
      Serial.print("MOTOR_INVERT=");
      Serial.println(MOTOR_INVERT);
      break;
    }
    case 'g': {
      GYRO_INVERT = !GYRO_INVERT;
      Serial.print("GYRO_INVERT=");
      Serial.println(GYRO_INVERT);
      break;
    }
    default:
      // ignore
      break;
  }
}
