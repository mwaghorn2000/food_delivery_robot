#include <Pixy2.h>
#include <Servo.h>

#include "config.h"

Pixy2 pixy;
Servo mandible;

enum class RobotState {
  SCAN_FOR_BASE,
  SEARCH_FOR_BALL,
  APPROACH_BALL,
  PICKUP_BALL,
  RETURN_TO_BASE,
  DROP_BALL
};

RobotState state = RobotState::SCAN_FOR_BASE;
uint8_t currentBaseSignature = 0;
uint8_t currentBallSignature = 0;
bool carryingBall = false;
unsigned long stateStartMillis = 0;
unsigned long lastTargetSeen = 0;

Block trackedBlock;

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------
void setMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t forwardLevel, uint8_t reverseLevel, int speed) {
  speed = constrain(speed, -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
  bool forward = speed >= 0;
  digitalWrite(dirPin, forward ? forwardLevel : reverseLevel);
  delayMicroseconds(DIR_SETTLE_DELAY);
  analogWrite(pwmPin, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  int adjustedLeft = static_cast<int>(leftSpeed * MOTOR_SPEED_RATIO);
  adjustedLeft = constrain(adjustedLeft, -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
  setMotor(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_FORWARD, MOTOR1_REVERSE, adjustedLeft);
  setMotor(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_FORWARD, MOTOR2_REVERSE, rightSpeed);
}

void stopMotion() {
  drive(0, 0);
  delay(MOTOR_STOP_DELAY);
}

void driveForward(uint8_t speed) {
  drive(speed, speed);
}

void driveReverse(uint8_t speed) {
  drive(-speed, -speed);
}

void rotateLeft(uint8_t speed) {
  drive(-speed, speed);
}

void rotateRight(uint8_t speed) {
  drive(speed, -speed);
}

void mandibleOpen() {
  mandible.write(SERVO_OPEN_ANGLE);
  delay(SERVO_ACTION_DELAY);
}

void mandibleClose() {
  mandible.write(SERVO_CLOSED_ANGLE);
  delay(SERVO_ACTION_DELAY);
}

void setState(RobotState newState) {
  state = newState;
  stateStartMillis = millis();
}

// ---------------------------------------------------------------------------
// Pixy helpers
// ---------------------------------------------------------------------------
bool acquireSignature(uint8_t signature, Block &result) {
  if (!pixy.ccc.getBlocks()) {
    return false;
  }

  Block *largest = nullptr;
  for (uint16_t i = 0; i < pixy.ccc.numBlocks; i++) {
    Block &block = pixy.ccc.blocks[i];
    if (block.m_signature == signature) {
      if (!largest || block.m_width * block.m_height > largest->m_width * largest->m_height) {
        largest = &block;
      }
    }
  }

  if (largest) {
    result = *largest;
    lastTargetSeen = millis();
    return true;
  }
  return false;
}

bool findAny(const uint8_t *signatures, uint8_t count, uint8_t &matchedSignature, Block &result) {
  if (!pixy.ccc.getBlocks()) {
    return false;
  }

  Block *largest = nullptr;
  uint8_t signature = 0;
  for (uint16_t i = 0; i < pixy.ccc.numBlocks; i++) {
    Block &block = pixy.ccc.blocks[i];
    for (uint8_t idx = 0; idx < count; idx++) {
      if (block.m_signature == signatures[idx]) {
        if (!largest || block.m_width * block.m_height > largest->m_width * largest->m_height) {
          largest = &block;
          signature = signatures[idx];
        }
      }
    }
  }

  if (largest) {
    result = *largest;
    matchedSignature = signature;
    lastTargetSeen = millis();
    return true;
  }
  return false;
}

bool isCentered(const Block &block) {
  return abs(block.m_x - PIXY_CENTER_X) <= CENTER_TOLERANCE;
}

bool atPickupDistance(const Block &block, uint8_t targetSize) {
  return abs(block.m_width - targetSize) <= SIZE_TOLERANCE;
}

void steerToTarget(const Block &block, uint8_t baseSpeed) {
  int error = block.m_x - PIXY_CENTER_X;
  int correction = map(error, -60, 60, -baseSpeed, baseSpeed);
  correction = constrain(correction, -static_cast<int>(baseSpeed), static_cast<int>(baseSpeed));
  drive(baseSpeed - correction, baseSpeed + correction);
}

bool timeoutReached(unsigned long startTime, uint16_t timeout) {
  return millis() - startTime >= timeout;
}

// ---------------------------------------------------------------------------
// State handlers
// ---------------------------------------------------------------------------
void handleScanForBase() {
  uint8_t signature = 0;
  if (findAny(BASE_SIGNATURES, BASE_SIGNATURE_COUNT, signature, trackedBlock)) {
    currentBaseSignature = signature;
    digitalWrite(LED_PIN, HIGH);
    setState(RobotState::SEARCH_FOR_BALL);
    return;
  }

  rotateLeft(SPEED_ROTATE_CONT);
}

void handleSearchForBall() {
  if (findAny(BALL_SIGNATURES, BALL_SIGNATURE_COUNT, currentBallSignature, trackedBlock)) {
    setState(RobotState::APPROACH_BALL);
    return;
  }

  if (timeoutReached(stateStartMillis, BALL_ACQUIRE_TIMEOUT)) {
    rotateRight(SPEED_ROTATE_CONT);
  } else {
    driveForward(SPEED_SLOW);
  }
}

void handleApproachBall() {
  if (!acquireSignature(currentBallSignature, trackedBlock)) {
    if (timeoutReached(lastTargetSeen, TARGET_LOST_TIMEOUT)) {
      setState(RobotState::SEARCH_FOR_BALL);
    } else {
      rotateLeft(SPEED_ROTATE_CONT);
    }
    return;
  }

  if (isCentered(trackedBlock)) {
    if (atPickupDistance(trackedBlock, TARGET_BALL_SIZE)) {
      stopMotion();
      setState(RobotState::PICKUP_BALL);
      return;
    }
    driveForward(SPEED_CREEP);
  } else {
    steerToTarget(trackedBlock, SPEED_SLOW);
  }
}

void handlePickupBall() {
  mandibleClose();
  carryingBall = true;
  setState(RobotState::RETURN_TO_BASE);
}

void handleReturnToBase() {
  if (!acquireSignature(currentBaseSignature, trackedBlock)) {
    if (timeoutReached(lastTargetSeen, TARGET_LOST_TIMEOUT)) {
      setState(RobotState::SCAN_FOR_BASE);
    } else {
      rotateRight(SPEED_ROTATE_CONT);
    }
    return;
  }

  if (isCentered(trackedBlock) && atPickupDistance(trackedBlock, TARGET_BASE_SIZE)) {
    stopMotion();
    setState(RobotState::DROP_BALL);
    return;
  }

  steerToTarget(trackedBlock, carryingBall ? SPEED_SLOW : SPEED_NORMAL);
}

void handleDropBall() {
  driveReverse(SPEED_CREEP);
  delay(NUDGE_TIME);
  stopMotion();
  mandibleOpen();
  carryingBall = false;
  setState(RobotState::SEARCH_FOR_BALL);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  mandible.attach(SERVO_PIN);
  mandibleOpen();

  pixy.init();
  pixy.setLamp(1, 0);

  digitalWrite(LED_PIN, LOW);
  setState(RobotState::SCAN_FOR_BASE);
}

void loop() {
  switch (state) {
    case RobotState::SCAN_FOR_BASE:
      handleScanForBase();
      break;
    case RobotState::SEARCH_FOR_BALL:
      handleSearchForBall();
      break;
    case RobotState::APPROACH_BALL:
      handleApproachBall();
      break;
    case RobotState::PICKUP_BALL:
      handlePickupBall();
      break;
    case RobotState::RETURN_TO_BASE:
      handleReturnToBase();
      break;
    case RobotState::DROP_BALL:
      handleDropBall();
      break;
  }
}
