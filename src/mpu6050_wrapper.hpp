#ifndef MPU6050_WRAPPER_HPP
#define MPU6050_WRAPPER_HPP

#include <Arduino.h>

void initMpu6050();

extern uint8_t fifoBuffer[64];  // FIFO storage buffer

bool readMpu6050();

#endif  // End of include guard for MPU6050_WRAPPER_HPP