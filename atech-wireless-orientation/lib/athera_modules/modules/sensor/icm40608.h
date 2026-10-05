/**
 * @file icm40608.h
 * @brief 6-Axis IMU driver for Athera — supports ICM-40608 or LSM6DSOX
 *
 * The driver auto-detects which chip is physically attached by probing
 * both candidate I2C addresses at begin() and reading their respective
 * WHO_AM_I registers. The same public API is exposed regardless of
 * which chip is present, so generated user code doesn't care.
 *
 * Supported chips:
 * - TDK InvenSense ICM-40608: addr 0x68, WHO_AM_I 0x75 → 0x39
 * - STMicro LSM6DSOX:         addr 0x6A, WHO_AM_I 0x0F → 0x6C
 *
 * Both expose 3-axis accelerometer, 3-axis gyroscope, and on-die
 * temperature at default full-scale ranges (±2g, ±250°/s).
 *
 * A background FreeRTOS task polls the sensor at ~100Hz so public
 * getters return cached values instantly.
 *
 * Athera Connector:
 * - Line A: SDA
 * - Line B: SCL
 */

#ifndef ICM40608_MODULE_H
#define ICM40608_MODULE_H

#include <i2c_interface.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ICM40608 {
public:
    enum ChipType {
        CHIP_NONE,
        CHIP_ICM40608,
        CHIP_LSM6DSOX,
    };

    enum Orientation {
        ORI_UNKNOWN,
        ORI_FACE_UP,
        ORI_FACE_DOWN,
        ORI_EDGE_RIGHT,
        ORI_EDGE_LEFT,
        ORI_EDGE_FRONT,
        ORI_EDGE_BACK,
    };

    // ICM-40608 identity
    static constexpr uint8_t ICM_ADDRESS       = 0x68;
    static constexpr uint8_t ICM_WHO_AM_I_REG  = 0x75;
    static constexpr uint8_t ICM_WHO_AM_I_VAL  = 0x39;

    // LSM6DSOX identity. SA0 strap selects 0x6A (low) or 0x6B (high); probe both.
    static constexpr uint8_t LSM_ADDRESS       = 0x6A;
    static constexpr uint8_t LSM_ADDRESS_ALT   = 0x6B;
    static constexpr uint8_t LSM_WHO_AM_I_REG  = 0x0F;
    static constexpr uint8_t LSM_WHO_AM_I_VAL  = 0x6C;

    /**
     * @brief Construct the sensor. Which chip/address is in use is
     *        decided at begin() via auto-detection.
     */
    explicit ICM40608(I2CInterface* i2c);

    /**
     * @brief Detect the attached chip, initialize it, and start the
     *        background polling task.
     * @return true on success (one of the two chips responded).
     */
    bool begin();

    /** @brief Read cached X-axis acceleration in g */
    float readAccelX();
    /** @brief Read cached Y-axis acceleration in g */
    float readAccelY();
    /** @brief Read cached Z-axis acceleration in g */
    float readAccelZ();

    /** @brief Read cached X-axis angular velocity in deg/s */
    float readGyroX();
    /** @brief Read cached Y-axis angular velocity in deg/s */
    float readGyroY();
    /** @brief Read cached Z-axis angular velocity in deg/s */
    float readGyroZ();

    /** @brief Read cached die temperature in Celsius */
    float readTemperature();

    /**
     * @brief Read all cached sensor values at once. Any out pointer may be null.
     */
    void readAll(float* ax, float* ay, float* az,
                 float* gx, float* gy, float* gz,
                 float* temp);

    /**
     * @brief Calibrate gyroscope zero-offset (sensor must be stationary).
     * @param samples Number of samples to average (default 100)
     */
    void calibrateGyro(int samples = 100);

    // ==========================================================================
    // Derived helpers — computed from cached accel/gyro. All return instantly.
    // Axes default to chip-frame: +Z points out of the module's top face when
    // the board is laid flat, +X along Line A→B, +Y along the connector.
    // setMountRotation() (called by codegen) re-expresses X-Y into the board
    // frame so tilt direction matches the screen regardless of mount side.
    // ==========================================================================

    /** @brief Pitch (nose up/down) in degrees, -90..+90, from accelerometer. */
    float getPitch();
    /** @brief Roll (left/right tilt) in degrees, -180..+180, from accelerometer. */
    float getRoll();
    /** @brief Total tilt from upright in degrees, 0 = flat face-up, 180 = flat face-down. */
    float getTiltAngle();

    /** @brief Coarse 6-way orientation classification (face up/down + 4 edges). */
    Orientation getOrientation();
    /** @brief Human-readable orientation: "face_up", "face_down", "edge_right", etc. */
    const char* getOrientationName();

    /**
     * @brief Edge-triggered shake/tap detector. Returns true ONCE per event,
     *        then resets. Fires when total accel magnitude deviates from 1g
     *        beyond the threshold (default 1.8g — covers shakes, taps, drops).
     *        Updated by the background poll task at ~100Hz.
     */
    bool wasShaken();

    /** @brief Set shake-detection threshold in g. Default 1.8. Lower = more sensitive. */
    void setShakeThreshold(float g);

    /**
     * @brief Re-express accel/gyro X-Y into the board reference frame.
     *
     * Left-column modules are mounted 180° rotated relative to right-column
     * modules, so their raw chip axes point the opposite way along the board.
     * Codegen calls this with the module's mount rotation about the board-normal
     * Z axis — picked from the port's board side — so that getPitch()/getRoll()/
     * getOrientation() and the raw X/Y reads agree with the screen's orientation.
     * The result: a physical tilt looks the same way on screen no matter which
     * port the IMU sits on. Z (board normal) is untouched. Default 0 (chip-frame).
     *
     * Accepts 0/90/180/270 (snapped). Left/right mounts use 0 or 180, which only
     * sign-flip X-Y — pitch/roll/orientation stay semantically correct. 90/270
     * additionally swap the X and Y axes, so pitch and roll exchange roles; those
     * angles are supported but not emitted by codegen today.
     */
    void setMountRotation(int degrees);

    /** @brief True once begin() has successfully detected and configured a chip */
    bool isConnected();

    /** @brief Which chip was detected. Useful for diagnostics. */
    ChipType getChipType() const { return _chip; }

    /** @brief Human-readable chip name ("ICM-40608", "LSM6DSOX", or "none"). */
    const char* getChipName() const;

    // Background task entry point (public for the static trampoline; don't call directly).
    void _pollTaskLoop();

private:
    I2CInterface* _i2c;
    uint8_t _address;     // Populated after successful detectChip()
    ChipType _chip;
    TaskHandle_t _pollTask;

    // Cached sensor values (updated by background task)
    volatile float _accelX, _accelY, _accelZ;
    volatile float _gyroX, _gyroY, _gyroZ;
    volatile float _temperature;

    // Gyro calibration offsets (set by calibrateGyro)
    float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;

    // Shake detection state (updated by background poll task)
    volatile bool _shakeFlag;
    float _shakeThreshold;

    // Mount rotation about board-normal Z, in degrees {0,90,180,270}. Applied to
    // cached X-Y after each raw read so all getters report board-frame values.
    int _mountRotation;

    // Scale factors — set per chip during init (gyro differs between chips)
    float _accelScale;
    float _gyroScale;

    // ICM-40608 registers
    static constexpr uint8_t ICM_REG_DEVICE_CONFIG = 0x11;
    static constexpr uint8_t ICM_REG_TEMP_DATA1    = 0x1D;
    static constexpr uint8_t ICM_REG_PWR_MGMT0     = 0x4E;
    static constexpr uint8_t ICM_REG_GYRO_CONFIG0  = 0x4F;
    static constexpr uint8_t ICM_REG_ACCEL_CONFIG0 = 0x50;

    // LSM6DSOX registers
    static constexpr uint8_t LSM_REG_CTRL1_XL   = 0x10;
    static constexpr uint8_t LSM_REG_CTRL2_G    = 0x11;
    static constexpr uint8_t LSM_REG_CTRL3_C    = 0x12;
    static constexpr uint8_t LSM_REG_OUT_TEMP_L = 0x20;

    bool detectChip();
    bool initIcm40608();
    bool initLsm6dsox();
    bool readIcmData();
    bool readLsmData();
    bool readSensorData();  // dispatches on _chip, then applies mount rotation
    void applyMountRotation();  // rotate cached X-Y into board frame

    uint8_t readRegisterAt(uint8_t addr, uint8_t reg);
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
};

#endif // ICM40608_MODULE_H
