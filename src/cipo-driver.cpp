#include "cipo-driver.hpp"

#include "reset.hpp"

void RovCommsPeripheral::init() {
}  // init

int RovCommsPeripheral::awaitHandshake() {
  LOGGING_SERIAL.print(F("Awaiting handshake 0x"));
  LOGGING_SERIAL.println(EXPECTED_HANDSHAKE_START, HEX);

  uint8_t data;

  do {
    // Wait for data
    while (!mySerial.available())
      ;

    // Read and verify data
    data = mySerial.read();
    if (data != (EXPECTED_HANDSHAKE_START)) {
      LOGGING_SERIAL.print(F("Handshake failed; expected 0x"));
      LOGGING_SERIAL.print(EXPECTED_HANDSHAKE_START, HEX);
      LOGGING_SERIAL.print(F(" but got 0x"));
      LOGGING_SERIAL.println(data, HEX);
    }
  } while (data != (EXPECTED_HANDSHAKE_START));

  // Send handshake response and wait for echo

  unsigned long timestamp = millis() & UINT32_MAX, recieved_timestamp = 0;
  mySerial.flush();
  for (uint8_t i = 0; i < sizeof(timestamp); i++) {
    mySerial.write((uint8_t)(timestamp >> (i * 8)) & 0xFFu);
  }

  // Read echo bytes
  for (uint8_t i = 0; i < sizeof(timestamp); i++) {
    while (!mySerial.available())
      ;  // Wait for echo bytes
    recieved_timestamp |= ((unsigned long)mySerial.read()) << (i * 8);
  }
  if (recieved_timestamp != timestamp) {
    // Failed; restart handshake
    LOGGING_SERIAL.print(F("Handshake failed; expected timestamp"));
    LOGGING_SERIAL.print(timestamp);
    LOGGING_SERIAL.print(F(" but got "));
    LOGGING_SERIAL.println(recieved_timestamp);
    return 1;
  }

  return 0;
}

void RovCommsPeripheral::sendChecksum() {
  mySerial.write(cipo_checksum);
  mySerial.flush();

#if DEBUG_CIPO_DRIVER
  LOGGING_SERIAL.print(F("Sent checksum 0x"));
  LOGGING_SERIAL.println(cipo_checksum, HEX);
#endif

  cipo_checksum = 0;
};  // sendChecksum

void RovCommsPeripheral::sendBlock(uint8_t data) {
  cipo_checksum += data;
  mySerial.write(data);

#if DEBUG_CIPO_DRIVER
  LOGGING_SERIAL.print(F("CIPO @"));
  LOGGING_SERIAL.print(cipo_index);
  LOGGING_SERIAL.print(F(": 0x"));
  LOGGING_SERIAL.println(data, HEX);
#endif

  cipo_index++;

  if (cipo_index >= WRITE_BUFFER_LENGTH) {
    this->sendChecksum();
    cipo_index = 0;
  }
};  // sendBlock

void RovCommsPeripheral::sendBlocks(const uint8_t data[], size_t length) {
  for (size_t i = 0; i < length; i++)
    this->sendBlock(data[i]);
};  // sendBlocks

bool RovCommsPeripheral::readBlocks() {
  uint8_t data;
  while (mySerial.available()) {
    data = mySerial.read();
    if (data == -1) break;

    if (copi_index < READ_BUFFER_LENGTH) {
      // Store everything but the checksum
      read_buffer[copi_index] = data;
      copi_checksum += data;
    } else {
      // We've read the entire buffer, verify the checksum and return true
      copi_checksum_status = (copi_checksum == data);
      copi_index = 0;
      copi_checksum = 0;

#if DEBUG_CIPO_DRIVER
      LOGGING_SERIAL.print(F("Read checksum 0x"));
      LOGGING_SERIAL.println(data, HEX);
      LOGGING_SERIAL.print(F("Calculated checksum 0x"));
      LOGGING_SERIAL.println(copi_checksum, HEX);
      LOGGING_SERIAL.print(F("checksum status: "));
      LOGGING_SERIAL.println(copi_checksum_status);
#endif

      return true;
    }
    copi_index++;

#if DEBUG_CIPO_DRIVER
    LOGGING_SERIAL.print(F("Read @"));
    LOGGING_SERIAL.println(copi_index);
#endif
  }
  // We haven't read the entire buffer yet, return false
  return false;
};  // readBlocks
