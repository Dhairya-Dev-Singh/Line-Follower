#include <Arduino.h>
#include <QTRSensors.h>

// ============================================================
// SENSOR CONFIGURATION
// ============================================================

#define SENSOR_COUNT 8

// Software order: LEFTMOST to RIGHTMOST
const uint8_t sensorPins[SENSOR_COUNT] = {
  22,  // S8 - leftmost
  23,  // S7
  21,  // S6
  4,   // S5
  19,  // S4
  18,  // S3
  33,  // S2
  32   // S1 - rightmost
};

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// ============================================================
// TB6612FNG MOTOR DRIVER PINS
// ============================================================

#define PWMA 25
#define AIN1 26
#define AIN2 27

#define PWMB 14
#define BIN1 12
#define BIN2 13

#define START_BUTTON 15

// Motor channels are physically swapped
#define MOTORS_SWAPPED true

#define LEFT_MOTOR_DIRECTION  1
#define RIGHT_MOTOR_DIRECTION 1

// ============================================================
// SPEED SETTINGS
// ============================================================

#define BASE_SPEED       120
#define MIN_CURVE_SPEED   45
#define TURN_SPEED       110
#define RECOVERY_SPEED   100
#define MAX_SPEED        255

// ============================================================
// PID SETTINGS
// ============================================================

#define KP 0.090
#define KD 0.120

// ============================================================
// SENSOR SETTINGS
// ============================================================

#define SENSOR_THRESHOLD    400
#define LINE_SUM_THRESHOLD  450

// ============================================================
// CALIBRATION SETTINGS
// ============================================================

#define CALIBRATION_90_DEG_MS          360
#define CALIBRATION_TURN_SPEED         65
#define CALIBRATION_CENTER_TRIM_MS     25
#define CALIBRATION_CENTER_TRIM_SPEED  50

// ============================================================
// TURN SETTINGS
// ============================================================

#define MIN_TURN_TIME_MS       100
#define TURN_TIMEOUT_MS       1200
#define JUNCTION_LOCKOUT_MS    300
#define JUNCTION_FORWARD_MS     45

// ============================================================
// CONTROL VARIABLES
// ============================================================

float lastError = 0.0;
int lastTurnDirection = -1;

unsigned long lineLostStart = 0;
unsigned long junctionLockoutUntil = 0;

// ============================================================
// MOTOR CONTROL
// ============================================================

void setPhysicalMotor(
  int pwmPin,
  int in1Pin,
  int in2Pin,
  int speedValue
) {
  speedValue = constrain(speedValue, -MAX_SPEED, MAX_SPEED);

  if (speedValue > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speedValue);
  }
  else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed *= LEFT_MOTOR_DIRECTION;
  rightSpeed *= RIGHT_MOTOR_DIRECTION;

#if MOTORS_SWAPPED

  // Logical left motor → physical channel B
  // Logical right motor → physical channel A

  setPhysicalMotor(PWMB, BIN1, BIN2, leftSpeed);
  setPhysicalMotor(PWMA, AIN1, AIN2, rightSpeed);

#else

  setPhysicalMotor(PWMA, AIN1, AIN2, leftSpeed);
  setPhysicalMotor(PWMB, BIN1, BIN2, rightSpeed);

#endif
}

void stopMotors() {
  setMotors(0, 0);
}

// ============================================================
// AUTOMATIC CALIBRATION
// ============================================================

void calibrateTurn(int direction, unsigned long durationMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < durationMs) {
    setMotors(
      direction * CALIBRATION_TURN_SPEED,
      -direction * CALIBRATION_TURN_SPEED
    );

    qtr.calibrate();
    delay(5);
  }

  stopMotors();
  delay(100);
}

void centerTrim() {
  unsigned long startTime = millis();

  while (millis() - startTime < CALIBRATION_CENTER_TRIM_MS) {
    setMotors(
      CALIBRATION_CENTER_TRIM_SPEED,
      -CALIBRATION_CENTER_TRIM_SPEED
    );

    qtr.calibrate();
    delay(5);
  }

  stopMotors();
  delay(100);
}

void automaticCalibration() {
  delay(700);

  // Left 90 degrees
  calibrateTurn(-1, CALIBRATION_90_DEG_MS);

  // Return to center
  calibrateTurn(+1, CALIBRATION_90_DEG_MS);

  // Right 90 degrees
  calibrateTurn(+1, CALIBRATION_90_DEG_MS);

  // Return to center
  calibrateTurn(-1, CALIBRATION_90_DEG_MS);

  centerTrim();
  stopMotors();
}

// ============================================================
// START BUTTON
// ============================================================

void waitForStartButton() {
  while (digitalRead(START_BUTTON) == HIGH) {
    stopMotors();
    delay(10);
  }

  delay(40);

  while (digitalRead(START_BUTTON) == LOW) {
    delay(10);
  }

  delay(150);
}

// ============================================================
// SENSOR READING
// ============================================================

bool sensorActive(uint8_t index) {
  return sensorValues[index] >= SENSOR_THRESHOLD;
}

bool lineDetected(float &position, uint32_t &signalSum) {
  qtr.read(sensorValues);

  signalSum = 0;
  uint32_t weightedSum = 0;
  uint16_t maximumSignal = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t value = sensorValues[i];

    if (value > 1000) {
      value = 1000;
    }

    signalSum += value;

    if (value > maximumSignal) {
      maximumSignal = value;
    }

    weightedSum += (uint32_t)value * (i * 1000);
  }

  if (
    maximumSignal < SENSOR_THRESHOLD ||
    signalSum < LINE_SUM_THRESHOLD
  ) {
    return false;
  }

  position = (float)weightedSum / (float)signalSum;

  return true;
}

// ============================================================
// JUNCTION DETECTION
// ============================================================

bool junctionDetected() {
  bool leftBranch =
    sensorActive(0) || sensorActive(1);

  bool rightBranch =
    sensorActive(6) || sensorActive(7);

  bool centerLine =
    sensorActive(3) || sensorActive(4);

  bool wideLeftBranch =
    sensorActive(0) && sensorActive(1);

  bool wideRightBranch =
    sensorActive(6) && sensorActive(7);

  return (
    (centerLine && wideLeftBranch) ||
    (centerLine && wideRightBranch) ||
    (wideLeftBranch && wideRightBranch)
  );
}

// ============================================================
// TURN UNTIL NEW LINE IS FOUND
// ============================================================

void turnUntilLine(int direction) {
  unsigned long startTime = millis();
  bool centerLineLost = false;

  while (millis() - startTime < TURN_TIMEOUT_MS) {
    setMotors(
      direction * TURN_SPEED,
      -direction * TURN_SPEED
    );

    qtr.read(sensorValues);

    bool centerDetected =
      sensorActive(3) || sensorActive(4);

    unsigned long elapsed =
      millis() - startTime;

    if (
      elapsed > MIN_TURN_TIME_MS &&
      !centerDetected
    ) {
      centerLineLost = true;
    }

    if (
      centerLineLost &&
      centerDetected &&
      elapsed > MIN_TURN_TIME_MS
    ) {
      break;
    }

    delay(3);
  }

  stopMotors();
  delay(30);

  // Center the robot on the new line
  setMotors(75, 75);
  delay(35);

  stopMotors();
  delay(30);
}

// ============================================================
// LEFT-HAND RULE DECISION
// ============================================================

void handleJunction() {
  // Move into the center of the junction
  setMotors(85, 85);
  delay(JUNCTION_FORWARD_MS);

  stopMotors();
  delay(20);

  qtr.read(sensorValues);

  bool leftAvailable =
    sensorActive(0) || sensorActive(1);

  bool rightAvailable =
    sensorActive(6) || sensorActive(7);

  bool straightAvailable =
    sensorActive(3) || sensorActive(4);

  // LEFT-HAND RULE:
  // Left → Straight → Right → U-turn

  if (leftAvailable) {
    turnUntilLine(-1);
  }
  else if (straightAvailable) {
    setMotors(BASE_SPEED, BASE_SPEED);
    delay(80);
    stopMotors();
  }
  else if (rightAvailable) {
    turnUntilLine(+1);
  }
  else {
    // Dead end: U-turn
    turnUntilLine(+1);
  }

  junctionLockoutUntil =
    millis() + JUNCTION_LOCKOUT_MS;
}

// ============================================================
// LINE RECOVERY
// ============================================================

void recoverLine() {
  unsigned long lostTime =
    millis() - lineLostStart;

  if (lastTurnDirection > 0) {
    if (lostTime < 250) {
      setMotors(RECOVERY_SPEED, 35);
    }
    else {
      setMotors(TURN_SPEED, -TURN_SPEED);
    }
  }
  else {
    if (lostTime < 250) {
      setMotors(35, RECOVERY_SPEED);
    }
    else {
      setMotors(-TURN_SPEED, TURN_SPEED);
    }
  }
}

// ============================================================
// LINE FOLLOWING
// ============================================================

void followLine() {
  float position = 3500.0;
  uint32_t signalSum = 0;

  bool found = lineDetected(position, signalSum);

  if (!found) {
    if (lineLostStart == 0) {
      lineLostStart = millis();
    }

    if (millis() - lineLostStart > 350) {
      turnUntilLine(
        lastTurnDirection > 0 ? +1 : -1
      );

      lineLostStart = 0;
    }
    else {
      recoverLine();
    }

    return;
  }

  lineLostStart = 0;

  if (
    millis() > junctionLockoutUntil &&
    junctionDetected()
  ) {
    handleJunction();
    return;
  }

  // Normal PID line following
  float error = position - 3500.0;
  float derivative = error - lastError;

  float correction =
    (KP * error) +
    (KD * derivative);

  lastError = error;

  if (error > 80) {
    lastTurnDirection = 1;
  }
  else if (error < -80) {
    lastTurnDirection = -1;
  }

  int curveReduction =
    abs((int)error) / 55;

  curveReduction =
    constrain(
      curveReduction,
      0,
      BASE_SPEED - MIN_CURVE_SPEED
    );

  int currentBaseSpeed =
    BASE_SPEED - curveReduction;

  int leftSpeed =
    currentBaseSpeed + (int)correction;

  int rightSpeed =
    currentBaseSpeed - (int)correction;

  leftSpeed =
    constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);

  rightSpeed =
    constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);

  stopMotors();

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);

  automaticCalibration();
  waitForStartButton();

  delay(300);
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  followLine();
  delay(5);
}