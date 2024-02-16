#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/// PINOUT ////////////////////////////////////////////////////////////////////

const size_t MOTION_MOTOR_COUNT = 6;
const size_t SERVO_COUNT = 6;
const size_t LEAK_SENSOR_COUNT = 2;

const unsigned long MAX_COPI_DELAY_MS = 100;

const int PIN_MOTION_MOTORS[MOTION_MOTOR_COUNT] = {8, 9, 10, 11, 12, 13};
const int PIN_SERVOS[SERVO_COUNT] = {2, 3, 4, 5, 6, 7};
const int PIN_LEAK_SENSORS[LEAK_SENSOR_COUNT] = {A0, A1};

const int PIN_CONFIG_SWITCHES[] = {22, 23, 24, 25, 26};
const size_t NUM_CONFIG_SWITCHES = sizeof(PIN_CONFIG_SWITCHES) / sizeof(PIN_CONFIG_SWITCHES[0]);

/// STARTUP CONFIGURATION /////////////////////////////////////////////////////

const uint16_t MOTION_MOTORS_ZERO_VAL = 1500;

const uint8_t SERVO_ZERO_POSITIONS[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

/// COMMUNICATION CONFIGURATION ///////////////////////////////////////////////

const uint8_t MPU_DATA_BYTES = 14;

const uint8_t READ_BUFFER_LENGTH = (2 * MOTION_MOTOR_COUNT) + 1 + SERVO_COUNT;
const uint8_t WRITE_BUFFER_LENGTH = 1 + MPU_DATA_BYTES;
// Masking prevents errors on overflow
const uint8_t EXPECTED_HANDSHAKE_START = 0xFF & (READ_BUFFER_LENGTH * WRITE_BUFFER_LENGTH);

/// SERIAL CONFIGURATION //////////////////////////////////////////////////////

const unsigned long LOGGING_SERIAL_BAUD_RATE = 115200;

const unsigned long DATA_SERIAL_BAUD_RATE_SLOW = 115200;
const unsigned long DATA_SERIAL_BAUD_RATE_FAST = 2000000;

#define LOGGING_SERIAL Serial
#define DATA_SERIAL Serial1

#endif // #ifndef CONFIG_HPP