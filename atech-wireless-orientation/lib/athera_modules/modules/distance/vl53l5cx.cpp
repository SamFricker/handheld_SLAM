/**
 * @file vl53l5cx.cpp
 * @brief VL53L5CX 8x8 ToF Distance Sensor implementation for Athera
 *
 * Background FreeRTOS task polls sensor at ~15Hz and caches the 8x8
 * distance grid. Public methods return cached values for non-blocking access.
 */

#include "vl53l5cx.h"

static void rangeTaskTrampoline(void* param) {
    ((VL53L5CX_Sensor*)param)->_rangeTaskLoop();
}

VL53L5CX_Sensor::VL53L5CX_Sensor(TwoWire& wirePort)
    : _wire(wirePort)
    , _rangeTask(nullptr)
    , _dataReady(false)
{
    memset((void*)_distances, 0, sizeof(_distances));
}

bool VL53L5CX_Sensor::begin() {
    // The Wire bus has already been initialized by codegen for this port —
    // do NOT call _wire.begin() here, that would re-pinmux the bus and break
    // any other I2C device sharing this peripheral.
    Serial.println("Initializing VL53L5CX (this takes a few seconds)...");

    if (!_sensor.begin(0x29, _wire)) {
        Serial.println("VL53L5CX not found!");
        return false;
    }

    _sensor.setResolution(64);       // 8x8 mode
    _sensor.setRangingFrequency(15); // 15Hz
    _sensor.startRanging();

    // Start background ranging task on Core 0
    xTaskCreatePinnedToCore(
        rangeTaskTrampoline,
        "VL53L5CX",
        4096,      // Needs more stack for the SparkFun library
        this,
        1,
        &_rangeTask,
        0
    );

    Serial.println("VL53L5CX initialized");
    return true;
}

int16_t VL53L5CX_Sensor::getDistance(int zone) {
    if (zone < 0 || zone >= GRID_SIZE) return -1;
    return _distances[zone];
}

void VL53L5CX_Sensor::getDistanceGrid(int16_t* buffer) {
    memcpy(buffer, (const void*)_distances, sizeof(int16_t) * GRID_SIZE);
}

bool VL53L5CX_Sensor::isReady() {
    return _dataReady;
}

int16_t VL53L5CX_Sensor::getMinDistance() {
    int16_t minDist = 32767;
    for (int i = 0; i < GRID_SIZE; i++) {
        if (_distances[i] > 0 && _distances[i] < minDist) {
            minDist = _distances[i];
        }
    }
    return minDist;
}

int16_t VL53L5CX_Sensor::getMaxDistance() {
    int16_t maxDist = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        if (_distances[i] > maxDist) {
            maxDist = _distances[i];
        }
    }
    return maxDist;
}

// ========== Background task ==========

void VL53L5CX_Sensor::_rangeTaskLoop() {
    VL53L5CX_ResultsData results;

    while (true) {
        if (_sensor.isDataReady()) {
            if (_sensor.getRangingData(&results)) {
                for (int i = 0; i < GRID_SIZE; i++) {
                    _distances[i] = results.distance_mm[i];
                }
                _dataReady = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Poll at ~20Hz (sensor runs at 15Hz)
    }
}
