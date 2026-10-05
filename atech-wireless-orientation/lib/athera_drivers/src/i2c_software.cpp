/**
 * @file i2c_software.cpp
 * @brief Software (bit-banged) I2C implementation for ESP32
 */

#include "i2c_software.h"

SoftwareI2C::SoftwareI2C(int sdaPin, int sclPin)
    : _sdaPin(sdaPin)
    , _sclPin(sclPin)
    , _delayUs(5)  // Default ~100kHz
    , _address(0)
    , _error(0)
    , _transmitting(false)
    , _rxBufferIndex(0)
    , _rxBufferLength(0)
    , _txBufferLength(0)
    , _mutex(nullptr)
{
}

bool SoftwareI2C::begin(uint32_t frequency) {
    // Create mutex for thread safety (if not already created)
    if (_mutex == nullptr) {
        _mutex = xSemaphoreCreateMutex();
        if (_mutex == nullptr) {
            return false;  // Failed to create mutex
        }
    }

    setClock(frequency);

    // Configure pins as open-drain with pull-ups
    // ESP32 doesn't have true open-drain, so we simulate with INPUT/OUTPUT switching
    pinMode(_sdaPin, INPUT_PULLUP);
    pinMode(_sclPin, INPUT_PULLUP);

    // Ensure bus is idle (both lines high)
    sdaHigh();
    sclHigh();

    // Small delay to let lines settle
    delayMicroseconds(10);

    // Check if bus is stuck (SDA low while SCL high)
    if (!readSDA()) {
        // Try to recover by clocking SCL
        for (int i = 0; i < 9; i++) {
            sclLow();
            i2cDelay();
            sclHigh();
            i2cDelay();
            if (readSDA()) break;
        }
        // Generate stop condition
        sdaLow();
        i2cDelay();
        sclHigh();
        i2cDelay();
        sdaHigh();
        i2cDelay();
    }

    return true;
}

void SoftwareI2C::end() {
    // Return pins to input (high-Z)
    pinMode(_sdaPin, INPUT);
    pinMode(_sclPin, INPUT);
}

void SoftwareI2C::setClock(uint32_t frequency) {
    // Calculate half-period delay
    // Each bit requires 2 delays (high and low), plus some overhead
    if (frequency >= 400000) {
        _delayUs = 1;   // ~400kHz max for software I2C
    } else if (frequency >= 100000) {
        _delayUs = 5;   // ~100kHz
    } else {
        _delayUs = 500000 / frequency;  // Slower speeds
    }
}

void SoftwareI2C::beginTransmission(uint8_t address) {
    // Take mutex for thread safety (wait indefinitely)
    if (_mutex != nullptr) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }
    _address = address;
    _txBufferLength = 0;
    _error = 0;
    _transmitting = true;
}

uint8_t SoftwareI2C::endTransmission(bool sendStop) {
    if (!_transmitting) {
        // Release mutex if we somehow got here without transmitting
        if (_mutex != nullptr && sendStop) {
            xSemaphoreGive(_mutex);
        }
        return 4;  // Other error
    }

    // Send start condition
    startCondition();

    // Send address with write bit (0)
    if (!writeByte((_address << 1) | 0)) {
        if (sendStop) stopCondition();
        // On error, always release mutex and mark not transmitting
        _transmitting = false;
        if (_mutex != nullptr) {
            xSemaphoreGive(_mutex);
        }
        return 2;  // NACK on address
    }

    // Send buffered data
    for (size_t i = 0; i < _txBufferLength; i++) {
        if (!writeByte(_txBuffer[i])) {
            if (sendStop) stopCondition();
            // On error, always release mutex and mark not transmitting
            _transmitting = false;
            if (_mutex != nullptr) {
                xSemaphoreGive(_mutex);
            }
            return 3;  // NACK on data
        }
    }

    if (sendStop) {
        stopCondition();
        _transmitting = false;
        // Release mutex when transmission complete with stop
        if (_mutex != nullptr) {
            xSemaphoreGive(_mutex);
        }
    }
    // Note: If sendStop is false (repeated start), mutex stays held
    // and _transmitting stays true so requestFrom knows not to take mutex again

    return 0;  // Success
}

size_t SoftwareI2C::write(uint8_t data) {
    if (_txBufferLength >= TX_BUFFER_SIZE) {
        return 0;  // Buffer full
    }
    _txBuffer[_txBufferLength++] = data;
    return 1;
}

size_t SoftwareI2C::write(const uint8_t* data, size_t length) {
    size_t written = 0;
    for (size_t i = 0; i < length; i++) {
        if (write(data[i]) == 0) break;
        written++;
    }
    return written;
}

size_t SoftwareI2C::requestFrom(uint8_t address, size_t quantity, bool sendStop) {
    // Take mutex if not already held (standalone read, not after repeated start)
    // _transmitting == true means we're in a repeated start sequence
    if (_mutex != nullptr && !_transmitting) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }

    if (quantity > RX_BUFFER_SIZE) {
        quantity = RX_BUFFER_SIZE;
    }

    _rxBufferIndex = 0;
    _rxBufferLength = 0;

    // Send start condition (or repeated start if already transmitting)
    startCondition();

    // Send address with read bit (1)
    if (!writeByte((address << 1) | 1)) {
        if (sendStop) stopCondition();
        // On error, always release mutex
        _transmitting = false;
        if (_mutex != nullptr) {
            xSemaphoreGive(_mutex);
        }
        return 0;  // NACK on address
    }

    // Read bytes
    for (size_t i = 0; i < quantity; i++) {
        // Send ACK for all but last byte
        bool ack = (i < quantity - 1);
        _rxBuffer[i] = readByte(ack);
        _rxBufferLength++;
    }

    if (sendStop) {
        stopCondition();
        _transmitting = false;
        // Release mutex when done with stop condition
        if (_mutex != nullptr) {
            xSemaphoreGive(_mutex);
        }
    }

    return _rxBufferLength;
}

int SoftwareI2C::read() {
    if (_rxBufferIndex >= _rxBufferLength) {
        return -1;
    }
    return _rxBuffer[_rxBufferIndex++];
}

int SoftwareI2C::available() {
    return _rxBufferLength - _rxBufferIndex;
}

int SoftwareI2C::peek() {
    if (_rxBufferIndex >= _rxBufferLength) {
        return -1;
    }
    return _rxBuffer[_rxBufferIndex];
}

void SoftwareI2C::flush() {
    // Nothing to do for software I2C
}

// ============================================================================
// Low-level bit-banging functions
// ============================================================================

void SoftwareI2C::sdaHigh() {
    pinMode(_sdaPin, INPUT_PULLUP);  // Release line, pull-up brings it high
}

void SoftwareI2C::sdaLow() {
    pinMode(_sdaPin, OUTPUT);
    digitalWrite(_sdaPin, LOW);
}

void SoftwareI2C::sclHigh() {
    pinMode(_sclPin, INPUT_PULLUP);  // Release line, pull-up brings it high
    waitClockStretch();              // Wait if slave is stretching clock
}

void SoftwareI2C::sclLow() {
    pinMode(_sclPin, OUTPUT);
    digitalWrite(_sclPin, LOW);
}

bool SoftwareI2C::readSDA() {
    pinMode(_sdaPin, INPUT_PULLUP);
    return digitalRead(_sdaPin) == HIGH;
}

bool SoftwareI2C::readSCL() {
    return digitalRead(_sclPin) == HIGH;
}

void SoftwareI2C::i2cDelay() {
    delayMicroseconds(_delayUs);
}

bool SoftwareI2C::waitClockStretch(uint32_t timeoutUs) {
    uint32_t start = micros();
    while (digitalRead(_sclPin) == LOW) {
        if ((micros() - start) > timeoutUs) {
            return false;  // Timeout
        }
    }
    return true;
}

// ============================================================================
// I2C protocol primitives
// ============================================================================

void SoftwareI2C::startCondition() {
    // Start: SDA goes LOW while SCL is HIGH
    sdaHigh();
    i2cDelay();
    sclHigh();
    i2cDelay();
    sdaLow();
    i2cDelay();
    sclLow();
    i2cDelay();
}

void SoftwareI2C::stopCondition() {
    // Stop: SDA goes HIGH while SCL is HIGH
    sdaLow();
    i2cDelay();
    sclHigh();
    i2cDelay();
    sdaHigh();
    i2cDelay();
}

bool SoftwareI2C::writeByte(uint8_t data) {
    // Write 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) {
            sdaHigh();
        } else {
            sdaLow();
        }
        i2cDelay();
        sclHigh();
        i2cDelay();
        sclLow();
        i2cDelay();
    }

    // Release SDA and read ACK
    sdaHigh();
    i2cDelay();
    sclHigh();
    i2cDelay();
    bool ack = !readSDA();  // ACK = SDA LOW
    sclLow();
    i2cDelay();

    return ack;
}

uint8_t SoftwareI2C::readByte(bool ack) {
    uint8_t data = 0;

    sdaHigh();  // Release SDA for slave to drive

    // Read 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        i2cDelay();
        sclHigh();
        i2cDelay();
        if (readSDA()) {
            data |= (1 << i);
        }
        sclLow();
    }

    // Send ACK or NACK
    if (ack) {
        sdaLow();   // ACK = SDA LOW
    } else {
        sdaHigh();  // NACK = SDA HIGH
    }
    i2cDelay();
    sclHigh();
    i2cDelay();
    sclLow();
    i2cDelay();
    sdaHigh();  // Release SDA

    return data;
}
