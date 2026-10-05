/**
 * @file i2c_hardware.cpp
 * @brief Hardware I2C wrapper implementation for ESP32
 */

#include "i2c_hardware.h"

WireI2C::WireI2C(TwoWire& wireInstance)
    : _wire(wireInstance)
    , _sdaPin(-1)
    , _sclPin(-1)
    , _customPins(false)
{
}

WireI2C::WireI2C(TwoWire& wireInstance, int sdaPin, int sclPin)
    : _wire(wireInstance)
    , _sdaPin(sdaPin)
    , _sclPin(sclPin)
    , _customPins(true)
{
}

bool WireI2C::begin(uint32_t frequency) {
    if (_customPins) {
        // ESP32 Wire.begin() accepts custom pins
        _wire.begin(_sdaPin, _sclPin);
    } else {
        _wire.begin();
    }
    _wire.setClock(frequency);
    return true;
}

void WireI2C::end() {
    _wire.end();
}

void WireI2C::setClock(uint32_t frequency) {
    _wire.setClock(frequency);
}

void WireI2C::beginTransmission(uint8_t address) {
    _wire.beginTransmission(address);
}

uint8_t WireI2C::endTransmission(bool sendStop) {
    return _wire.endTransmission(sendStop);
}

size_t WireI2C::write(uint8_t data) {
    return _wire.write(data);
}

size_t WireI2C::write(const uint8_t* data, size_t length) {
    return _wire.write(data, length);
}

size_t WireI2C::requestFrom(uint8_t address, size_t quantity, bool sendStop) {
    return _wire.requestFrom(address, quantity, sendStop);
}

int WireI2C::read() {
    return _wire.read();
}

int WireI2C::available() {
    return _wire.available();
}

int WireI2C::peek() {
    return _wire.peek();
}

void WireI2C::flush() {
    _wire.flush();
}
