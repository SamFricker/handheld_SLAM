/**
 * @file i2c_interface.h
 * @brief Abstract I2C interface for Athera module system
 *
 * This interface allows modules to work with either hardware I2C or
 * software (bit-banged) I2C transparently. Each motherboard port has
 * only 2 data lines, so each port can only support ONE I2C device.
 * By abstracting I2C, we can use hardware I2C when available and
 * fall back to software I2C on any GPIO pair.
 */

#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Abstract I2C interface
 *
 * All I2C implementations (hardware and software) must implement this interface.
 * Modules receive a pointer to this interface and don't need to know which
 * implementation is being used.
 */
class I2CInterface {
public:
    virtual ~I2CInterface() {}

    /**
     * @brief Initialize the I2C bus
     * @param frequency Bus frequency in Hz (default 100kHz)
     * @return true on success
     */
    virtual bool begin(uint32_t frequency = 100000) = 0;

    /**
     * @brief End/release the I2C bus
     */
    virtual void end() = 0;

    /**
     * @brief Set the bus clock frequency
     * @param frequency Frequency in Hz
     */
    virtual void setClock(uint32_t frequency) = 0;

    /**
     * @brief Begin a transmission to a device
     * @param address 7-bit I2C device address
     */
    virtual void beginTransmission(uint8_t address) = 0;

    /**
     * @brief End a transmission
     * @param sendStop Send stop condition (true) or repeated start (false)
     * @return 0=success, 1=data too long, 2=NACK on address, 3=NACK on data, 4=other error
     */
    virtual uint8_t endTransmission(bool sendStop = true) = 0;

    /**
     * @brief Write a single byte
     * @param data Byte to write
     * @return Number of bytes written (1 on success)
     */
    virtual size_t write(uint8_t data) = 0;

    /**
     * @brief Write multiple bytes
     * @param data Pointer to data buffer
     * @param length Number of bytes to write
     * @return Number of bytes written
     */
    virtual size_t write(const uint8_t* data, size_t length) = 0;

    /**
     * @brief Request bytes from a device
     * @param address 7-bit I2C device address
     * @param quantity Number of bytes to request
     * @param sendStop Send stop condition after read
     * @return Number of bytes received
     */
    virtual size_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) = 0;

    /**
     * @brief Read a byte from the receive buffer
     * @return Next byte or -1 if buffer empty
     */
    virtual int read() = 0;

    /**
     * @brief Check bytes available in receive buffer
     * @return Number of bytes available
     */
    virtual int available() = 0;

    /**
     * @brief Peek at the next byte without removing it
     * @return Next byte or -1 if buffer empty
     */
    virtual int peek() = 0;

    /**
     * @brief Flush the transmit buffer
     */
    virtual void flush() = 0;

    // Convenience methods (implemented in terms of above primitives)

    /**
     * @brief Write a byte to a register
     * @param address Device address
     * @param reg Register address
     * @param value Value to write
     * @return true on success
     */
    bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
        beginTransmission(address);
        write(reg);
        write(value);
        return endTransmission() == 0;
    }

    /**
     * @brief Write multiple bytes to a register
     * @param address Device address
     * @param reg Register address
     * @param data Data buffer
     * @param length Number of bytes
     * @return true on success
     */
    bool writeRegister(uint8_t address, uint8_t reg, const uint8_t* data, size_t length) {
        beginTransmission(address);
        write(reg);
        write(data, length);
        return endTransmission() == 0;
    }

    /**
     * @brief Read a byte from a register
     * @param address Device address
     * @param reg Register address
     * @param value Pointer to store value
     * @return true on success
     */
    bool readRegister(uint8_t address, uint8_t reg, uint8_t* value) {
        beginTransmission(address);
        write(reg);
        if (endTransmission(false) != 0) return false;

        if (requestFrom(address, 1) != 1) return false;
        *value = read();
        return true;
    }

    /**
     * @brief Read multiple bytes from a register
     * @param address Device address
     * @param reg Register address
     * @param buffer Buffer to store data
     * @param length Number of bytes to read
     * @return Number of bytes read
     */
    size_t readRegister(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) {
        beginTransmission(address);
        write(reg);
        if (endTransmission(false) != 0) return 0;

        size_t received = requestFrom(address, length);
        for (size_t i = 0; i < received; i++) {
            buffer[i] = read();
        }
        return received;
    }

    /**
     * @brief Check if a device is present at an address
     * @param address 7-bit device address
     * @return true if device ACKs
     */
    bool devicePresent(uint8_t address) {
        beginTransmission(address);
        return endTransmission() == 0;
    }
};

#endif // I2C_INTERFACE_H
