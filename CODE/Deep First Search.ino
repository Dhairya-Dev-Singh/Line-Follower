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
// TB6612FNG PINS
// ============================================================

#define PWMA 25
#define AIN1 26
#define AIN2 27

#define PWMB 14
#define BIN1 12
#define BIN2 13

#define START_BUTTON 15

// Motors are physically connected to opposite channels
#define MOTORS_SWAPPED true

#define LEFT_MOTOR_DIRECTION  1
#define RIGHT_MOTOR_DIRECTION 1

// ============================================================
// SPEED SETTINGS
// ============================================================

#define BASE_SPEED        120
#define MIN_CURVE_SPEED    45
#define TURN_SPEED        110
#define RECOVERY_SPEED    100
#define MAX_SPEED         255

// ============================================================
// PID SETTINGS
// ============================================================

#define KP 0.090
#define KD 0.120

// ============================================================
// SENSOR SETTINGS
// ============================================================

#define SENSOR_THRESHOLD     400
#define LINE_SUM_THRESHOLD   450

// Finish is a wide black patch
#define FINISH_THRESHOLD       750
#define FINISH_CONFIRM_COUNT    20

// ============================================================
// CALIBRATION SETTINGS
// ============================================================

#define CALIBRATION_90_DEG_MS          360
#define CALIBRATION_TURN_SPEED         65
#define CALIBRATION_CENTER_TRIM_MS     25
#define CALIBRATION_CENTER_TRIM_SPEED  50

// ============================================================
// TURN AND DFS SETTINGS
// ============================================================

#define MIN_TURN_TIME_MS       100
#define TURN_TIMEOUT_MS       1200
#define JUNCTION_LOCKOUT_MS    300
#define JUNCTION_FORWARD_MS     45

#define MAX_DFS_DEPTH           32
#define MAX_PATH_LENGTH        128

// DFS branch preference
// Change order if required
#define FIRST_BRANCH  'R'
#define SECOND_BRANCH 'S'
#define THIRD_BRANCH  'L'

// ============================================================
// TYPES AND VARIABLES
// ============================================================

enum EventType {
  EVENT_JUNCTION,
  EVENT_DEAD_END,
  EVENT_FINISH,
  EVENT_TIMEOUT
};

#define BRANCH_LEFT      1
#define BRANCH_STRAIGHT  2
#define BRANCH_RIGHT     4

float lastError = 0.0;
int lastTurnDirection = 1;

unsigned long lineLostStart = 0;
unsigned long junctionLockoutUntil = 0;

char explorationPath[MAX_PATH_LENGTH];
int pathLength = 0;

bool finishReached = false;

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
// PATH STORAGE
// ============================================================

void storePathEvent(char eventCode) {
  if (pathLength < MAX_PATH_LENGTH - 1) {
    explorationPath[pathLength++] = eventCode;
    explorationPath[pathLength] = '\0';
  }
}

// ============================================================
// CALIBRATION
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

  calibrateTurn(-1, CALIBRATION_90_DEG_MS);
  calibrateTurn(+1, CALIBRATION_90_DEG_MS);
  calibrateTurn(+1, CALIBRATION_90_DEG_MS);
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
// SENSOR FUNCTIONS
// ============================================================

bool sensorActive(uint8_t index) {
  return sensorValues[index] >= SENSOR_THRESHOLD;
}

bool calculateLinePosition(float &position) {
  uint32_t signalSum = 0;
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

  position =
    (float)weightedSum / (float)signalSum;

  return true;
}

bool finishPatchDetected() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] < FINISH_THRESHOLD) {
      return false;
    }
  }

  return true;
}

// ============================================================
// JUNCTION DETECTION
// ============================================================

bool junctionDetected() {
  bool leftBranch =
    sensorActive(0) && sensorActive(1);

  bool rightBranch =
    sensorActive(6) && sensorActive(7);

  bool centerLine =
    sensorActive(3) || sensorActive(4);

  return (
    (centerLine && leftBranch) ||
    (centerLine && rightBranch) ||
    (leftBranch && rightBranch)
  );
}

uint8_t scanBranches() {
  qtr.read(sensorValues);

  uint8_t branches = 0;

  bool leftAvailable =
    sensorActive(0) || sensorActive(1);

  bool rightAvailable =
    sensorActive(6) || sensorActive(7);

  bool straightAvailable =
    sensorActive(3) || sensorActive(4);

  if (leftAvailable) {
    branches |= BRANCH_LEFT;
  }

  if (straightAvailable) {
    branches |= BRANCH_STRAIGHT;
  }

  if (rightAvailable) {
    branches |= BRANCH_RIGHT;
  }

  return branches;
}

// ============================================================
// TURN CONTROL
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

  setMotors(75, 75);
  delay(35);

  stopMotors();
  delay(30);

  junctionLockoutUntil =
    millis() + JUNCTION_LOCKOUT_MS;
}

void makeUTurn() {
  storePathEvent('U');
  turnUntilLine(+1);
}

// ============================================================
// BRANCH MOVEMENT
// ============================================================

void takeBranch(char branch) {
  storePathEvent(branch);

  if (branch == 'L') {
    turnUntilLine(-1);
  }
  else if (branch == 'R') {
    turnUntilLine(+1);
  }
  else {
    // Straight
    setMotors(BASE_SPEED, BASE_SPEED);
    delay(80);
    stopMotors();
    delay(30);
  }
}

// ============================================================
// FOLLOW UNTIL DFS EVENT
// ============================================================

EventType followUntilEvent() {
  int finishCounter = 0;

  while (true) {
    qtr.read(sensorValues);

    // Check finish patch
    if (finishPatchDetected()) {
      finishCounter++;

      if (finishCounter >= FINISH_CONFIRM_COUNT) {
        stopMotors();
        return EVENT_FINISH;
      }
    }
    else {
      finishCounter = 0;
    }

    float position = 3500.0;
    bool lineFound =
      calculateLinePosition(position);

    if (!lineFound) {
      if (lineLostStart == 0) {
        lineLostStart = millis();
      }

      if (millis() - lineLostStart > 350) {
        stopMotors();
        lineLostStart = 0;
        return EVENT_DEAD_END;
      }

      if (lastTurnDirection > 0) {
        setMotors(RECOVERY_SPEED, 35);
      }
      else {
        setMotors(35, RECOVERY_SPEED);
      }

      continue;
    }

    lineLostStart = 0;

    if (
      millis() > junctionLockoutUntil &&
      junctionDetected()
    ) {
      setMotors(75, 75);
      delay(JUNCTION_FORWARD_MS);

      stopMotors();
      delay(20);

      junctionLockoutUntil =
        millis() + JUNCTION_LOCKOUT_MS;

      return EVENT_JUNCTION;
    }

    // Normal line following
    float error =
      position - 3500.0;

    float derivative =
      error - lastError;

    float correction =
      KP * error +
      KD * derivative;

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

    delay(5);
  }
}

// ============================================================
// DFS BRANCH ORDER
// ============================================================

char getBranchByOrder(uint8_t index) {
  if (index == 0) return FIRST_BRANCH;
  if (index == 1) return SECOND_BRANCH;
  return THIRD_BRANCH;
}

bool branchAvailable(
  uint8_t branchMask,
  char branch
) {
  if (branch == 'L') {
    return branchMask & BRANCH_LEFT;
  }

  if (branch == 'S') {
    return branchMask & BRANCH_STRAIGHT;
  }

  if (branch == 'R') {
    return branchMask & BRANCH_RIGHT;
  }

  return false;
}

// ============================================================
// DFS RECURSION
// ============================================================

bool dfsExplore(uint8_t depth) {
  if (depth >= MAX_DFS_DEPTH) {
    return false;
  }

  uint8_t branchMask =
    scanBranches();

  // Fallback if sensor readings are uncertain
  if (branchMask == 0) {
    branchMask = BRANCH_STRAIGHT;
  }

  for (uint8_t i = 0; i < 3; i++) {
    char branch =
      getBranchByOrder(i);

    if (!branchAvailable(branchMask, branch)) {
      continue;
    }

    // Take selected branch
    takeBranch(branch);

    EventType event =
      followUntilEvent();

    if (event == EVENT_FINISH) {
      finishReached = true;
      return true;
    }

    if (event == EVENT_DEAD_END) {
      // Dead end: turn around and return
      makeUTurn();

      EventType returnEvent =
        followUntilEvent();

      if (returnEvent == EVENT_FINISH) {
        finishReached = true;
        return true;
      }

      continue;
    }

    if (event == EVENT_JUNCTION) {
      bool childFinished =
        dfsExplore(depth + 1);

      if (childFinished) {
        return true;
      }

      // Child junction completely explored.
      // Return to the parent junction.
      makeUTurn();

      EventType returnEvent =
        followUntilEvent();

      if (returnEvent == EVENT_FINISH) {
        finishReached = true;
        return true;
      }
    }

    if (event == EVENT_TIMEOUT) {
      makeUTurn();
      followUntilEvent();
    }
  }

  // All branches at this junction explored
  return false;
}

// ============================================================
// PRINT STORED EXPLORATION PATH
// ============================================================

void printStoredPath() {
  Serial.println();
  Serial.println("DFS EXPLORATION COMPLETED");
  Serial.print("Stored path events: ");

  for (int i = 0; i < pathLength; i++) {
    Serial.print(explorationPath[i]);
    Serial.print(' ');
  }

  Serial.println();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

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

  Serial.println("DFS STARTED");

  EventType firstEvent =
    followUntilEvent();

  if (firstEvent == EVENT_FINISH) {
    finishReached = true;
  }
  else if (firstEvent == EVENT_DEAD_END) {
    makeUTurn();
    firstEvent = followUntilEvent();
  }

  if (!finishReached && firstEvent == EVENT_JUNCTION) {
    dfsExplore(0);
  }

  stopMotors();

  if (finishReached) {
    Serial.println("FINISH FOUND");
  }
  else {
    printStoredPath();
  }
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  stopMotors();
  delay(100);
}