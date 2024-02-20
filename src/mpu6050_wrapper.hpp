#ifndef MPU6050_WRAPPER_HPP
#define MPU6050_WRAPPER_HPP

#include <Arduino.h>

void initMpu6050(); // Initializes Gyroscope and Accelerometer device

extern uint8_t fifoBuffer[64]; // FIFO storage buffer

bool readMpu6050(); // Reads information from the MPU050

#endif // #ifndef MPU6050_WRAPPER_HPP