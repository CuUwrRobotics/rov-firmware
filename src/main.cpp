#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>

#include "cipo-driver.hpp"
#include "config.hpp"
#include "mpu6050_wrapper.hpp"
#include "reset.hpp"

#define MODE_COPI 0
#define MODE_COPI_SUCCESSFUL 1
#define MODE_CIPO 2

Servo motion_motors[MOTION_MOTOR_COUNT];
Servo servos[SERVO_COUNT];

char writebuf[256];

uint8_t mode = MODE_COPI;

RovCommsPeripheral cipo_driver;

bool reset_requested = false;

void setup()
{
  reset_requested = false;

  // Initialize movement motors
  for (uint8_t i = 0; i < MOTION_MOTOR_COUNT; i++)
  {
    motion_motors[i].attach(PIN_MOTION_MOTORS[i]);
    motion_motors[i].writeMicroseconds(MOTION_MOTORS_ZERO_VAL);
  }

  for (uint8_t i = 0; i < NUM_CONFIG_SWITCHES; i++)
  {
    pinMode(PIN_CONFIG_SWITCHES[i], INPUT_PULLUP);
  }

  // Initialize data serial connection with topside
  if (digitalRead(PIN_CONFIG_SWITCHES[0]) == LOW)
  {
    DATA_SERIAL.begin(DATA_SERIAL_BAUD_RATE_SLOW);
  }
  else
  {
    DATA_SERIAL.begin(DATA_SERIAL_BAUD_RATE_FAST);
  }

  // Initialize logging serial connection with topside
  LOGGING_SERIAL.begin(LOGGING_SERIAL_BAUD_RATE);

  // Initalize Servo Motors
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
  {
    servos[i].attach(PIN_SERVOS[i]);
    servos[i].write(SERVO_ZERO_POSITIONS[i]);
  }

  // Initalizes IC2 Devices
  Wire.begin();
  // Sets communication frequency
  Wire.setClock(400000);
  // Initalizes MPU6050
  initMpu6050();

  // Calibrates motors
  if (digitalRead(40) == LOW)
  {
    LOGGING_SERIAL.println(F("Calibrating motors."));
    for (uint8_t i = 0; i < MOTION_MOTOR_COUNT; i++)
      motion_motors[i].writeMicroseconds(MOTION_MOTORS_ZERO_VAL);
    delay(500);
    for (uint8_t i = 0; i < MOTION_MOTOR_COUNT; i++)
      motion_motors[i].writeMicroseconds(2000);
    delay(3500);
    for (uint8_t i = 0; i < MOTION_MOTOR_COUNT; i++)
      motion_motors[i].writeMicroseconds(1000);
    delay(4000);
    for (uint8_t i = 0; i < MOTION_MOTOR_COUNT; i++)
      motion_motors[i].writeMicroseconds(MOTION_MOTORS_ZERO_VAL);
  }

  // Initializes connection to topside
  cipo_driver.init();

  // Attempts handshake with topside
  while (cipo_driver.awaitHandshake() != 0)
  {
    LOGGING_SERIAL.println(F("Handshake failed, retrying."));
  }

  LOGGING_SERIAL.println(F("Handshake successful."));

  // Enters CIPO (Controller In, Peripherals Out) mode
  mode = MODE_CIPO;
}

// Checks to see if there are any leaks in the main chamber
uint8_t lookForLeaks()
{
  uint8_t leaks = (uint8_t)0;

  // Checks to make sure that the leak sensor count
  // doesn't exceed the capacity of a uint8_t
  if (LEAK_SENSOR_COUNT > 8)
  {
    LOGGING_SERIAL.println(F("Leak sensor count exceeds 8. `uint8_t` no longer suitable."));
    return 255;
  }

  // Loops through each of the sensors
  for (size_t i = 0; i < LEAK_SENSOR_COUNT; i++)
  {
    // If a leak is detected, the leak is recorded
    if (analogRead(PIN_LEAK_SENSORS[i]) < 512)
    {
      leaks |= 0x1u << i;
    }
  }
  return leaks;
}

// Resets connection with topside controller
// Requires handshake to be redone
void reset()
{
  LOGGING_SERIAL.println(F("Resetting..."));

  // Stops servos
  for (uint8_t i = 0; i < MOTION_MOTOR_COUNT; i++)
  {
    motion_motors[i].writeMicroseconds(MOTION_MOTORS_ZERO_VAL);
  }

  // Waits for handshake to occur
  while (cipo_driver.awaitHandshake() != 0)
  {
    LOGGING_SERIAL.println(F("Handshake failed, retrying."));
  }

  // Enters CIPO (Controller In, Peripheral Out) mode
  mode = MODE_CIPO;
  LOGGING_SERIAL.println(F("Handshake successful, beginning."));
}

// Core event loop
void loop()
{
  // Checks for reset request
  // If present, resets and clears request
  if (reset_requested)
  {
    reset();
    reset_requested = false;
  }

  // Reads information from topside
  if (mode == MODE_COPI)
  {
    if (cipo_driver.readBlocks())
    {
      if (cipo_driver.checksumGood())
      {
        // If checksum succeeds, reset buffer and switch to COPI_SUCCESSFUL
        cipo_driver.resetReadBuffer();
        mode = MODE_COPI_SUCCESSFUL;
      }
      else
      {
        // If checksum fails, reset connection with topside
        LOGGING_SERIAL.println(F("Checksum failed, resetting."));
        requestReset();
      }
    }
  }
  // Report back to topside
  else if (mode == MODE_CIPO)
  {
    uint8_t leakresult = lookForLeaks();
    readMpu6050(); // TODO: takes 2ms, ideally this would be more asynchonous

    // Sends information from the robot back to topside
    cipo_driver.sendBlock(leakresult);          // @0: Leak sensors
    cipo_driver.sendBlocks(&fifoBuffer[0], 2);  // @1-2:   Quaterion A
    cipo_driver.sendBlocks(&fifoBuffer[4], 2);  // @3-4:   Quaterion B
    cipo_driver.sendBlocks(&fifoBuffer[8], 2);  // @5-6:   Quaterion C
    cipo_driver.sendBlocks(&fifoBuffer[12], 2); // @7-8:   Quaterion D
    cipo_driver.sendBlocks(&fifoBuffer[28], 2); // @9-10:  Accelerometer X
    cipo_driver.sendBlocks(&fifoBuffer[32], 2); // @11-12: Accelerometer Y
    cipo_driver.sendBlocks(&fifoBuffer[36], 2); // @13-14: Accelerometer Z

    // Sets mode back to COPI
    mode = MODE_COPI;
  }
  // Write information from topside to pins
  else if (mode == MODE_COPI_SUCCESSFUL)
  {
    // Handle motion motor updates from last cycle
    motion_motors[0].writeMicroseconds(cipo_driver.popReadBuffer<uint16_t>());
    motion_motors[1].writeMicroseconds(cipo_driver.popReadBuffer<uint16_t>());
    motion_motors[2].writeMicroseconds(cipo_driver.popReadBuffer<uint16_t>());
    motion_motors[3].writeMicroseconds(cipo_driver.popReadBuffer<uint16_t>());
    motion_motors[4].writeMicroseconds(cipo_driver.popReadBuffer<uint16_t>());
    motion_motors[5].writeMicroseconds(cipo_driver.popReadBuffer<uint16_t>());

    // Handle servo updates from last cycle
    servos[0].write(cipo_driver.popReadBuffer<uint8_t>());
    servos[1].write(cipo_driver.popReadBuffer<uint8_t>());
    servos[2].write(cipo_driver.popReadBuffer<uint8_t>());
    servos[3].write(cipo_driver.popReadBuffer<uint8_t>());
    servos[4].write(cipo_driver.popReadBuffer<uint8_t>());
    servos[5].write(cipo_driver.popReadBuffer<uint8_t>());

    // Sets mode to CIPO (Controller In, Peripheral Out)
    mode = MODE_CIPO;
  }
}
