#include "cipo-driver.hpp"
#include "reset.hpp"

/**
 * @brief World's greatest mystery
 * @details Seemingly does nothing
 *
 */
void RovCommsPeripheral::init()
{
}

/**
 * @brief Handles handshake with topside
 *
 * @return 1 if handshake fails because timestamp is missing; 0 if handshake is successful
 */
int RovCommsPeripheral::awaitHandshake()
{
    // Prints expected value
    LOGGING_SERIAL.print(F("Awaiting handshake 0x"));
    LOGGING_SERIAL.println(EXPECTED_HANDSHAKE_START, HEX);

    // Place where handshake message from topside is stored
    uint8_t topsideHandshakeValue;

    do
    {
        // Wait for serial message from topside
        while (!mySerial.available())
            ;

        // Read and verify handshake value from topside
        topsideHandshakeValue = mySerial.read();
        if (topsideHandshakeValue != (EXPECTED_HANDSHAKE_START))
        {
            LOGGING_SERIAL.print(F("Handshake failed; expected 0x"));
            LOGGING_SERIAL.print(EXPECTED_HANDSHAKE_START, HEX);
            LOGGING_SERIAL.print(F(" but got 0x"));
            LOGGING_SERIAL.println(topsideHandshakeValue, HEX);
        }
        // Loops until a correct handshake value is sent
    } while (topsideHandshakeValue != EXPECTED_HANDSHAKE_START);

    // Send handshake response and wait for echo+
    {
        // Sets timestamp to current time, capped to 32 bits
        unsigned long timestamp = millis() & UINT32_MAX, recieved_timestamp = 0;
        mySerial.flush();

        // Sends timestamp one byte at a time to topside
        for (uint8_t i = 0; i < sizeof(timestamp); i++)
        {
            mySerial.write((uint8_t)(timestamp >> (i * 8)) & 0xFFu);
        }

        // Read echoed timestamp
        for (uint8_t i = 0; i < sizeof(timestamp); i++)
        {
            while (!mySerial.available())
                ; // Wait for echo bytes
            recieved_timestamp |= ((unsigned long)mySerial.read()) << (i * 8);
        }

        // Compares value of sent timestamp and echoed timestamp
        if (recieved_timestamp != timestamp)
        {
            // Failed; restart handshake
            LOGGING_SERIAL.print(F("Handshake failed; expected timestamp"));
            LOGGING_SERIAL.print(timestamp);
            LOGGING_SERIAL.print(F(" but got "));
            LOGGING_SERIAL.println(recieved_timestamp);
            return 1;
        }
    }

    return 0;
}

/**
 * @brief Sends checksum of current message to topside to check for data corruption
 *
 */
void RovCommsPeripheral::sendChecksum()
{
    // Sends the checksum for the current data block
    mySerial.write(cipo_checksum);
    mySerial.flush();

#if DEBUG_CIPO_DRIVER
    LOGGING_SERIAL.print(F("Sent checksum 0x"));
    LOGGING_SERIAL.println(cipo_checksum, HEX);
#endif

    // Resets the checksum after it is sent
    cipo_checksum = 0;
}

/**
 * @brief Sends blocks of data to topside. Sends checksum after 16 blocks have been sent
 *
 * @param data Data to send to topside (1 byte)
 */
void RovCommsPeripheral::sendBlock(uint8_t data)
{
    cipo_checksum += data;
    mySerial.write(data);

#if DEBUG_CIPO_DRIVER
    LOGGING_SERIAL.print(F("CIPO @"));
    LOGGING_SERIAL.print(cipo_index);
    LOGGING_SERIAL.print(F(": 0x"));
    LOGGING_SERIAL.println(data, HEX);
#endif

    // Increments current block count by 1
    cipo_index++;

    // When 16 blocks have been sent, the checksum is sent to allow for data verification
    if (cipo_index >= WRITE_BUFFER_LENGTH)
    {
        this->sendChecksum();
        cipo_index = 0;
    }
}

/**
 * @brief Divides data into blocks that can be sent to topside
 *
 * @param data Array of integer values to send to topside
 * @param length Length of the aformentioned array
 */
void RovCommsPeripheral::sendBlocks(const uint8_t data[], size_t length)
{
    for (size_t i = 0; i < length; i++)
        this->sendBlock(data[i]);
}

/**
 * @brief Reads information being sent by topside
 *
 * @return true on successful read; false on unsuccessful read
 */
bool RovCommsPeripheral::readBlocks()
{
    uint8_t data;
    while (mySerial.available())
    {
        data = mySerial.read();
        if (data == -1)
            break;

        if (copi_index < READ_BUFFER_LENGTH)
        {
            // Store everything but the checksum
            read_buffer[copi_index] = data;
            copi_checksum += data;
        }
        else
        {
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
}
