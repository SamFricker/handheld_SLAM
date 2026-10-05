/**
 * @file vl53l5cx.h
 * @brief VL53L5CX 8x8 Time-of-Flight Distance Sensor for Athera
 *
 * Multi-zone ToF sensor providing an 8x8 grid of distance measurements.
 * Uses a background FreeRTOS task for continuous ranging — public getters
 * return cached distance values instantly.
 *
 * Specifications:
 * - 64 distance zones (8x8 grid)
 * - Ranging frequency: 15Hz
 * - Range: up to 4000mm
 * - Interface: I2C (via Wire)
 *
 * Note: This module uses the SparkFun VL53L5CX library, which talks to the
 * Arduino Wire library directly (not our I2CInterface abstraction). It accepts
 * a TwoWire& reference so it can share the per-port Wire instance that codegen
 * already initialized — it must NOT re-initialize the bus itself, or it will
 * clobber any other I2C device sharing that peripheral.
 *
 * Athera Connector:
 * - Line A: SDA
 * - Line B: SCL
 */

#ifndef VL53L5CX_MODULE_H
#define VL53L5CX_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class VL53L5CX_Sensor {
public:
    static constexpr int GRID_SIZE = 64;  // 8x8

    /**
     * @brief Construct VL53L5CX sensor
     * @param wirePort TwoWire instance for this port (Wire or Wire1), already
     *                 initialized by codegen with the port's SDA/SCL pins.
     */
    explicit VL53L5CX_Sensor(TwoWire& wirePort);

    /**
     * @brief Initialize sensor, start ranging, and launch background task
     * @return true on success
     */
    bool begin();

    /**
     * @brief Get distance for a specific zone (0-63)
     * @param zone Zone index (row*8 + col, 0-63)
     * @return Distance in mm, or -1 if invalid zone
     */
    int16_t getDistance(int zone);

    /**
     * @brief Copy full 8x8 distance grid into buffer
     * @param buffer Array of at least 64 int16_t values
     */
    void getDistanceGrid(int16_t* buffer);

    /** @brief Check if sensor has provided at least one reading */
    bool isReady();

    /** @brief Get minimum distance across all zones in mm */
    int16_t getMinDistance();

    /** @brief Get maximum distance across all zones in mm */
    int16_t getMaxDistance();

    // Background task function (public for static callback, do not call directly)
    void _rangeTaskLoop();

private:
    TwoWire& _wire;
    SparkFun_VL53L5CX _sensor;
    TaskHandle_t _rangeTask;

    volatile int16_t _distances[GRID_SIZE];
    volatile bool _dataReady;
};

#endif // VL53L5CX_MODULE_H
