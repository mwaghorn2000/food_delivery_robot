#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Motor configuration
// ---------------------------------------------------------------------------
constexpr uint8_t MOTOR2_PWM = 9;     // Right motor PWM pin
constexpr uint8_t MOTOR1_PWM = 10;    // Left motor PWM pin
constexpr uint8_t MOTOR2_DIR = 4;     // Right motor DIR pin
constexpr uint8_t MOTOR1_DIR = 7;     // Left motor DIR pin

constexpr uint8_t SERVO_PIN = 11;     // Mandible servo pin
constexpr uint8_t LED_PIN = 3;        // Status LED

// Direction logic (makerverse driver)
constexpr uint8_t MOTOR2_FORWARD = LOW;
constexpr uint8_t MOTOR2_REVERSE = HIGH;
constexpr uint8_t MOTOR1_FORWARD = HIGH;  // wired backwards
constexpr uint8_t MOTOR1_REVERSE = LOW;   // wired backwards

// Speed configuration
constexpr float MOTOR_SPEED_RATIO = 0.82f; // Left to right scaling factor
constexpr uint8_t MAX_PWM = 255;

constexpr uint8_t SPEED_FAST = 110;
constexpr uint8_t SPEED_NORMAL = 85;
constexpr uint8_t SPEED_SLOW = 65;
constexpr uint8_t SPEED_CREEP = 55;
constexpr uint8_t SPEED_ROTATE_CONT = 60;
constexpr uint8_t SPEED_NUDGE = 70;

// Timing
constexpr uint16_t ROTATE_180_TIME = 2000;
constexpr uint16_t ROTATE_90_TIME = 900;
constexpr uint16_t NUDGE_TIME = 140;
constexpr uint16_t SHIMMY_TIME = 250;
constexpr uint16_t SERVO_ACTION_DELAY = 450;

constexpr uint16_t DIR_SETTLE_DELAY = 40;
constexpr uint16_t MOTOR_STOP_DELAY = 100;

// Servo positions
constexpr uint8_t SERVO_OPEN_ANGLE = 35;
constexpr uint8_t SERVO_CLOSED_ANGLE = 110;

// Pixy camera configuration
constexpr uint8_t BASE_SIGNATURES[] = {3, 4, 5, 6}; // Up to 4 possible base colors
constexpr uint8_t BASE_SIGNATURE_COUNT = sizeof(BASE_SIGNATURES);
constexpr uint8_t BALL_SIGNATURES[] = {1, 2};       // Blue + Green balls
constexpr uint8_t BALL_SIGNATURE_COUNT = sizeof(BALL_SIGNATURES);

constexpr int PIXY_CENTER_X = 59;
constexpr int PIXY_CENTER_Y = 177;
constexpr uint8_t CENTER_TOLERANCE = 4;
constexpr uint8_t TARGET_BALL_SIZE = 58;
constexpr uint8_t TARGET_BASE_SIZE = 70;
constexpr uint8_t SIZE_TOLERANCE = 4;

constexpr uint16_t BALL_ACQUIRE_TIMEOUT = 9000;   // ms
constexpr uint16_t BASE_ACQUIRE_TIMEOUT = 6000;   // ms
constexpr uint16_t TARGET_LOST_TIMEOUT = 2500;    // ms

#endif
