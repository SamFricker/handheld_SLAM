/**
 * @file i2c_hardware.h
 * @brief Hardware I2C wrapper for ESP32
 *
 * This wraps the ESP32's hardware I2C peripheral (Wire library) to conform
 * to our I2CInterface. ESP32 has two hardware I2C peripherals (Wire and Wire1).
 *
 * Advantages over software I2C:
 * - Faster (up to 1MHz on ESP32)
 * - Lower CPU usage
 * - More reliable timing
 *
 * Limitations:
 * - ESP32 has only 2 hardware I2C buses
 * - Pins must be capable of I2C function (most ESP32 GPIOs work)
 */

#ifndef I2C_HARDWARE_H
#define I2C_HARDWARE_H

#include "i2c_interface.h"
#include <Wire.h>

class WireI2C : public I2CInterface {
public:
    /**
     * @brief Construct using default I2C pins
     * @param wireInstance Reference to Wire or Wire1
     */
    explicit WireI2C(TwoWire& wireInstance = Wire);

    /**
     * @brief Construct with custom pins
     * @param wireInstance Reference to Wire or Wire1
     * @param sdaPin GPIO pin for SDA
     * @param sclPin GPIO pin for SCL
     */
    WireI2C(TwoWire& wireInstance, int sdaPin, int sclPin);

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
    TwoWire& _wire;
    int _sdaPin;
    int _sclPin;
    bool _customPins;
};

#endif // I2C_HARDWARE_H
