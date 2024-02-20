#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
// class default I2C address is 0x68

void initMpu6050()
{
	uint8_t devStatus;

	// Initialize device
	mpu.initialize();

	// Verify connection
	if (!mpu.testConnection())
	{
		// If connection fails, loop infinitely
		Serial.println(F("MPU6050 connection failed"));
		while (1)
			;
	}

	// Load and configure the DMP
	Serial.println(F("Initializing MPU6050..."));
	devStatus = mpu.dmpInitialize();

	// TODO: get the actual calibration values (which are printed using 'PrintActiveOffsets' below)
	// Don't know what that's printing though since it's a different number of offsets.
	// mpu.setXGyroOffset(220);
	// mpu.setYGyroOffset(76);
	// mpu.setZGyroOffset(-85);
	// mpu.setZAccelOffset(1788);

	if (devStatus == 0)
	{ // Successful DMP initialization
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();

		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);
	}
	else
	{
		// Error in DMP init:
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));

		while (1)
			;
	}
}

uint8_t fifoBuffer[64]; // FIFO storage buffer

bool readMpu6050()
{
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) == 1) // Get the Latest packet
		return true;								  // Read successfully
	else
		return false; // Failed to read
}
