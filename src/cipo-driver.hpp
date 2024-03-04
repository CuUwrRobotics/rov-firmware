#ifndef CIPO_DRIVER_HPP
#define CIPO_DRIVER_HPP

#include <Arduino.h>

#include "config.hpp"
#include "reset.hpp"

#define DEBUG_CIPO_DRIVER 0

/**
 * @brief Manages communication with topside
 *
 */
class RovCommsPeripheral
{
    uint8_t cipo_checksum, copi_checksum;

    uint8_t copi_index, cipo_index;

    uint8_t read_buffer_index;
    bool copi_checksum_status;

    uint8_t read_buffer[READ_BUFFER_LENGTH];

    HardwareSerial &mySerial;

    void sendChecksum();

public:
    /**
     * @brief Construct a new default Rov Comms Peripheral object
     *
     * @param ser
     */
    RovCommsPeripheral(HardwareSerial &ser = DATA_SERIAL)
        : cipo_checksum(0),
          copi_checksum(0),
          copi_index(0),
          cipo_index(0),
          read_buffer_index(0),
          copi_checksum_status(false),
          mySerial(ser){};

    void init();

    int awaitHandshake();

    void sendBlock(uint8_t data);

    void sendBlocks(const uint8_t data[], size_t length);

    /**
     * @brief Sends an 8 bit chunk of data to Topside
     *
     * @tparam T Type of data being sent
     * @param data An 8 bit chunk of data
     */
    template <typename T>
    void send(const T &data)
    {
        this->sendBlocks((uint8_t *)&data, sizeof(T));
    }

    bool readBlocks();

    /**
     * @brief Checks the current checksum status
     *
     * @return true
     * @return false
     */
    inline bool checksumGood() const { return copi_checksum_status; }

    /**
     * @brief Sets the read buffer index back to the beginning of the array
     *
     */
    inline void resetReadBuffer() { read_buffer_index = 0; }

    /**
     * @brief Reads the first item in the read buffer and increases the index
     *
     * @return uint8_t
     */
    uint8_t popReadBuffer()
    {
        uint8_t data = read_buffer[read_buffer_index++];
        if (read_buffer_index > READ_BUFFER_LENGTH)
        {
            LOGGING_SERIAL.println(F("Read buffer overflow in RovCommsPeripheral::popReadBuffer()"));
            read_buffer_index = 0;
            requestReset();
        }
        return data;
    }

    /**
     * @brief Gets a pointer to a piece of data from the read buffer
     *
     * @param length length of the data to read
     * @return uint8_t*
     */
    uint8_t *popReadBuffer(size_t length = 1)
    {
        uint8_t *data = read_buffer + read_buffer_index;
        read_buffer_index += length;
        if (read_buffer_index > READ_BUFFER_LENGTH)
        {
            LOGGING_SERIAL.println(F("Read buffer overflow in RovCommsPeripheral::popReadBuffer()"));
            read_buffer_index = 0;
            requestReset();
        }
        return data;
    }

    /**
     * @brief Interprets bits from the read buffer and casts them to a T
     *
     * @tparam T type of data being read
     * @return The data read from the read buffer
     */
    template <typename T>
    T popReadBuffer()
    {
        T data = *reinterpret_cast<T *>(&read_buffer[read_buffer_index]);
        // Serial.println("Pop: " + String(data) + " at " + String(read_buffer_index) + " of " + String(READ_BUFFER_LENGTH) + " with size " + String(sizeof(T)) + "");
        read_buffer_index += sizeof(T);
        if (read_buffer_index > READ_BUFFER_LENGTH)
            ;
        {
            LOGGING_SERIAL.println(F("Read buffer overflow in RovCommsPeripheral::popReadBuffer()"));
            read_buffer_index = 0;
            requestReset();
        }
        return data;
    }

}; // class RovCommsPeripheral

#endif // #ifndef CIPO_DRIVER_HPP