/**
 * @file i2c_software.h
 * @brief Software (bit-banged) I2C implementation for ESP32
 *
 * This implementation allows any two GPIO pins to act as an I2C bus.
 * Useful when hardware I2C peripherals are exhausted or when you need
 * I2C on non-standard pins.
 *
 * Key features:
 * - Works on any GPIO pair
 * - Supports clock stretching
 * - Configurable speed (limited by GPIO toggle speed)
 * - Compatible with standard I2C devices
 * - Thread-safe with FreeRTOS mutex (safe for multi-task use)
 *
 * Note: Software I2C is slower than hardware I2C and uses more CPU.
 * Use hardware I2C when available for time-critical applications.
 */

#ifndef I2C_SOFTWARE_H
#define I2C_SOFTWARE_H

#include "i2c_interface.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SoftwareI2C : public I2CInterface {
public:
    static constexpr size_t RX_BUFFER_SIZE = 32;

    /**
     * @brief Construct a software I2C instance
     * @param sdaPin GPIO pin for SDA (data)
     * @param sclPin GPIO pin for SCL (clock)
     */
    SoftwareI2C(int sdaPin, int sclPin);

    // I2CInterface implementation
    bool begin(uint32_t frequency = 100000) override;
    void end() override;
    void setClock(uint32_t frequency) override;
    void beginTransmission(uint8_t address) override;
    uint8_t endTransmission(bool sendStop = true) override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t* data, size_t length) override;
    size_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) override;
    int read() override;
    int available() override;
    int peek() override;
    void flush() override;

private:
    int _sdaPin;
    int _sclPin;
    uint32_t _delayUs;      // Half-period delay for clock
    uint8_t _address;       // Current transmission address
    uint8_t _error;         // Transmission error code
    bool _transmitting;     // Currently in transmission
    SemaphoreHandle_t _mutex;  // FreeRTOS mutex for thread safety

    // Receive buffer
    uint8_t _rxBuffer[RX_BUFFER_SIZE];
    size_t _rxBufferIndex;
    size_t _rxBufferLength;

    // Transmit buffer (for collecting data before sending)
    static constexpr size_t TX_BUFFER_SIZE = 32;
    uint8_t _txBuffer[TX_BUFFER_SIZE];
    size_t _txBufferLength;

    // Low-level bit-banging functions
    void sdaHigh();
    void sdaLow();
    void sclHigh();
    void sclLow();
    bool readSDA();
    bool readSCL();
    void i2cDelay();

    // I2C protocol primitives
    void startCondition();
    void stopCondition();
    bool writeByte(uint8_t data);
    uint8_t readByte(bool ack);
    bool waitClockStretch(uint32_t timeoutUs = 1000);
};

#endif // I2C_SOFTWARE_H
