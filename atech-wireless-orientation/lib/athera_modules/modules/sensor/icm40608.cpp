/**
 * @file icm40608.cpp
 * @brief 6-Axis IMU implementation — auto-detects ICM-40608 or LSM6DSOX.
 *
 * At begin() we probe 0x68/WHO_AM_I=0x39 first, then 0x6A/WHO_AM_I=0x6C.
 * Whichever chip responds is the one we configure and read from. A
 * background FreeRTOS task polls it at ~100Hz; public getters return
 * cached values for non-blocking access.
 */

#include "icm40608.h"
#include <Arduino.h>
#include <math.h>

static void pollTaskTrampoline(void* param) {
    ((ICM40608*)param)->_pollTaskLoop();
}

ICM40608::ICM40608(I2CInterface* i2c)
    : _i2c(i2c)
    , _address(0)
    , _chip(CHIP_NONE)
    , _pollTask(nullptr)
    , _accelX(0), _accelY(0), _accelZ(0)
    , _gyroX(0), _gyroY(0), _gyroZ(0)
    , _temperature(0)
    , _gyroOffsetX(0), _gyroOffsetY(0), _gyroOffsetZ(0)
    , _shakeFlag(false)
    , _shakeThreshold(1.8f)
    , _mountRotation(0)
    , _accelScale(16384.0f)
    , _gyroScale(131.0f)
{
}

const char* ICM40608::getChipName() const {
    switch (_chip) {
        case CHIP_ICM40608: return "ICM-40608";
        case CHIP_LSM6DSOX: return "LSM6DSOX";
        default:            return "none";
    }
}

bool ICM40608::begin() {
    if (!detectChip()) {
        Serial.println("[IMU] No supported 6-axis IMU detected (tried ICM-40608 @ 0x68, LSM6DSOX @ 0x6A)");
        return false;
    }

    bool ok = false;
    switch (_chip) {
        case CHIP_ICM40608: ok = initIcm40608(); break;
        case CHIP_LSM6DSOX: ok = initLsm6dsox(); break;
        default:            ok = false;
    }
    if (!ok) {
        Serial.print("[IMU] Chip init failed for ");
        Serial.println(getChipName());
        return false;
    }

    xTaskCreatePinnedToCore(
        pollTaskTrampoline,
        "IMU",
        2048,
        this,
        2,
        &_pollTask,
        0
    );

    Serial.print("[IMU] ");
    Serial.print(getChipName());
    Serial.print(" detected at 0x");
    Serial.print(_address, HEX);
    Serial.println(" — polling task started");
    return true;
}

// ========== Public getters ==========

float ICM40608::readAccelX() { return _accelX; }
float ICM40608::readAccelY() { return _accelY; }
float ICM40608::readAccelZ() { return _accelZ; }
float ICM40608::readGyroX() { return _gyroX - _gyroOffsetX; }
float ICM40608::readGyroY() { return _gyroY - _gyroOffsetY; }
float ICM40608::readGyroZ() { return _gyroZ - _gyroOffsetZ; }
float ICM40608::readTemperature() { return _temperature; }

void ICM40608::readAll(float* ax, float* ay, float* az,
                       float* gx, float* gy, float* gz,
                       float* temp) {
    if (ax)   *ax = _accelX;
    if (ay)   *ay = _accelY;
    if (az)   *az = _accelZ;
    if (gx)   *gx = _gyroX - _gyroOffsetX;
    if (gy)   *gy = _gyroY - _gyroOffsetY;
    if (gz)   *gz = _gyroZ - _gyroOffsetZ;
    if (temp) *temp = _temperature;
}

void ICM40608::calibrateGyro(int samples) {
    _gyroOffsetX = 0;
    _gyroOffsetY = 0;
    _gyroOffsetZ = 0;

    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < samples; i++) {
        sumX += _gyroX;
        sumY += _gyroY;
        sumZ += _gyroZ;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    _gyroOffsetX = sumX / samples;
    _gyroOffsetY = sumY / samples;
    _gyroOffsetZ = sumZ / samples;
}

bool ICM40608::isConnected() {
    return _chip != CHIP_NONE;
}

// ========== Derived helpers ==========

float ICM40608::getPitch() {
    float ax = _accelX, ay = _accelY, az = _accelZ;
    return atan2f(ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
}

float ICM40608::getRoll() {
    float ay = _accelY, az = _accelZ;
    return atan2f(ay, az) * 57.2957795f;
}

float ICM40608::getTiltAngle() {
    float ax = _accelX, ay = _accelY, az = _accelZ;
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag < 0.01f) return 0.0f;
    float c = az / mag;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c) * 57.2957795f;
}

ICM40608::Orientation ICM40608::getOrientation() {
    float ax = _accelX, ay = _accelY, az = _accelZ;
    float absX = fabsf(ax), absY = fabsf(ay), absZ = fabsf(az);
    // Dominant-axis must reach 0.7g — below that the board is between faces.
    const float MIN_G = 0.7f;
    if (absZ >= absX && absZ >= absY && absZ > MIN_G) {
        return az > 0 ? ORI_FACE_UP : ORI_FACE_DOWN;
    }
    if (absX >= absY && absX > MIN_G) {
        return ax > 0 ? ORI_EDGE_RIGHT : ORI_EDGE_LEFT;
    }
    if (absY > MIN_G) {
        return ay > 0 ? ORI_EDGE_FRONT : ORI_EDGE_BACK;
    }
    return ORI_UNKNOWN;
}

const char* ICM40608::getOrientationName() {
    switch (getOrientation()) {
        case ORI_FACE_UP:    return "face_up";
        case ORI_FACE_DOWN:  return "face_down";
        case ORI_EDGE_RIGHT: return "edge_right";
        case ORI_EDGE_LEFT:  return "edge_left";
        case ORI_EDGE_FRONT: return "edge_front";
        case ORI_EDGE_BACK:  return "edge_back";
        default:             return "unknown";
    }
}

bool ICM40608::wasShaken() {
    bool flag = _shakeFlag;
    _shakeFlag = false;
    return flag;
}

void ICM40608::setShakeThreshold(float g) {
    if (g > 0.1f) _shakeThreshold = g;
}

void ICM40608::setMountRotation(int degrees) {
    // Snap to the nearest quarter turn in {0,90,180,270}.
    int d = ((degrees % 360) + 360) % 360;
    _mountRotation = ((d + 45) / 90 % 4) * 90;
}

void ICM40608::applyMountRotation() {
    // Rotate the cached X-Y about the board-normal Z so readings are board-frame.
    // Z (accelZ/gyroZ) is the rotation axis and stays put. Applied to both accel
    // and gyro so getPitch/getRoll/getOrientation and raw reads all stay aligned.
    if (_mountRotation == 0) return;
    float ax = _accelX, ay = _accelY, gx = _gyroX, gy = _gyroY;
    switch (_mountRotation) {
        case 90:
            _accelX = -ay; _accelY = ax;  _gyroX = -gy; _gyroY = gx;
            break;
        case 180:
            _accelX = -ax; _accelY = -ay; _gyroX = -gx; _gyroY = -gy;
            break;
        case 270:
            _accelX = ay;  _accelY = -ax; _gyroX = gy;  _gyroY = -gx;
            break;
    }
}

// ========== Chip detection ==========

bool ICM40608::detectChip() {
    if (readRegisterAt(ICM_ADDRESS, ICM_WHO_AM_I_REG) == ICM_WHO_AM_I_VAL) {
        _address = ICM_ADDRESS;
        _chip = CHIP_ICM40608;
        return true;
    }
    // LSM6DSOX: probe both SA0=0 (0x6A) and SA0=1 (0x6B) straps.
    const uint8_t lsmCandidates[] = { LSM_ADDRESS, LSM_ADDRESS_ALT };
    for (uint8_t i = 0; i < sizeof(lsmCandidates); i++) {
        uint8_t candidate = lsmCandidates[i];
        if (readRegisterAt(candidate, LSM_WHO_AM_I_REG) == LSM_WHO_AM_I_VAL) {
            _address = candidate;
            _chip = CHIP_LSM6DSOX;
            return true;
        }
    }
    _chip = CHIP_NONE;
    return false;
}

// ========== ICM-40608 ==========

bool ICM40608::initIcm40608() {
    // Soft reset
    writeRegister(ICM_REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Gyro: ±250°/s, ODR 1kHz
    writeRegister(ICM_REG_GYRO_CONFIG0, (0x03 << 5) | 0x06);
    // Accel: ±2g, ODR 1kHz
    writeRegister(ICM_REG_ACCEL_CONFIG0, (0x03 << 5) | 0x06);
    // Power on: gyro + accel, low-noise mode
    writeRegister(ICM_REG_PWR_MGMT0, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    _accelScale = 16384.0f;  // ±2g
    _gyroScale  = 131.0f;    // ±250°/s
    return true;
}

bool ICM40608::readIcmData() {
    // Burst 14 bytes from TEMP_DATA1: 2 temp + 6 accel + 6 gyro, big-endian.
    _i2c->beginTransmission(_address);
    _i2c->write(ICM_REG_TEMP_DATA1);
    if (_i2c->endTransmission(false) != 0) return false;
    if (_i2c->requestFrom(_address, (size_t)14) < 14) return false;

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) buf[i] = _i2c->read();

    int16_t rawTemp = (int16_t)((buf[0] << 8) | buf[1]);
    _temperature = (rawTemp / 132.48f) + 25.0f;

    int16_t rawAX = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t rawAY = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t rawAZ = (int16_t)((buf[6]  << 8) | buf[7]);
    _accelX = rawAX / _accelScale;
    _accelY = rawAY / _accelScale;
    _accelZ = rawAZ / _accelScale;

    int16_t rawGX = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t rawGY = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t rawGZ = (int16_t)((buf[12] << 8) | buf[13]);
    _gyroX = rawGX / _gyroScale;
    _gyroY = rawGY / _gyroScale;
    _gyroZ = rawGZ / _gyroScale;

    return true;
}

// ========== LSM6DSOX ==========

bool ICM40608::initLsm6dsox() {
    // Software reset via CTRL3_C.SW_RESET
    writeRegister(LSM_REG_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    // BDU=1 (block data update) + IF_INC=1 (multi-byte auto-increment).
    writeRegister(LSM_REG_CTRL3_C, 0x44);
    // Accel: ODR 416Hz (0110), FS ±2g (00).
    writeRegister(LSM_REG_CTRL1_XL, 0x60);
    // Gyro: ODR 416Hz, FS ±250 dps.
    writeRegister(LSM_REG_CTRL2_G, 0x60);
    vTaskDelay(pdMS_TO_TICKS(10));

    _accelScale = 16384.0f;      // ±2g — same sensitivity as ICM
    _gyroScale  = 114.285714f;   // ±250°/s = 8.75 mdps/LSB
    return true;
}

bool ICM40608::readLsmData() {
    // OUT_TEMP_L (0x20) through OUTZ_H_A (0x2D) are consecutive, so one
    // 14-byte burst from 0x20 gets temp + gyro + accel. LSM is little-endian.
    _i2c->beginTransmission(_address);
    _i2c->write(LSM_REG_OUT_TEMP_L);
    if (_i2c->endTransmission(false) != 0) return false;
    if (_i2c->requestFrom(_address, (size_t)14) < 14) return false;

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) buf[i] = _i2c->read();

    int16_t rawTemp = (int16_t)((buf[1] << 8) | buf[0]);
    _temperature = (rawTemp / 256.0f) + 25.0f;

    int16_t rawGX = (int16_t)((buf[3]  << 8) | buf[2]);
    int16_t rawGY = (int16_t)((buf[5]  << 8) | buf[4]);
    int16_t rawGZ = (int16_t)((buf[7]  << 8) | buf[6]);
    _gyroX = rawGX / _gyroScale;
    _gyroY = rawGY / _gyroScale;
    _gyroZ = rawGZ / _gyroScale;

    int16_t rawAX = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t rawAY = (int16_t)((buf[11] << 8) | buf[10]);
    int16_t rawAZ = (int16_t)((buf[13] << 8) | buf[12]);
    _accelX = rawAX / _accelScale;
    _accelY = rawAY / _accelScale;
    _accelZ = rawAZ / _accelScale;

    return true;
}

// ========== Dispatcher + background task ==========

bool ICM40608::readSensorData() {
    bool ok;
    switch (_chip) {
        case CHIP_ICM40608: ok = readIcmData(); break;
        case CHIP_LSM6DSOX: ok = readLsmData(); break;
        default:            return false;
    }
    if (ok) applyMountRotation();
    return ok;
}

void ICM40608::_pollTaskLoop() {
    while (true) {
        if (readSensorData()) {
            // Shake/tap: latch when total accel magnitude deviates from 1g
            // beyond the threshold. wasShaken() reads-and-clears.
            float ax = _accelX, ay = _accelY, az = _accelZ;
            float mag = sqrtf(ax * ax + ay * ay + az * az);
            if (fabsf(mag - 1.0f) > (_shakeThreshold - 1.0f)) {
                _shakeFlag = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // ~100Hz
    }
}

// ========== I2C helpers ==========

uint8_t ICM40608::readRegisterAt(uint8_t addr, uint8_t reg) {
    _i2c->beginTransmission(addr);
    _i2c->write(reg);
    if (_i2c->endTransmission(false) != 0) return 0;
    if (_i2c->requestFrom(addr, (size_t)1) < 1) return 0;
    return _i2c->read();
}

uint8_t ICM40608::readRegister(uint8_t reg) {
    return readRegisterAt(_address, reg);
}

void ICM40608::writeRegister(uint8_t reg, uint8_t value) {
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    _i2c->write(value);
    _i2c->endTransmission();
}
