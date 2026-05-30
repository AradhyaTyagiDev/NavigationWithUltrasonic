#pragma once

#include <stdint.h>

struct FilteredSensorData
{
    float rawDistanceCm = 0.0f;

    float filteredDistanceCm = 0.0f;

    float velocityCmPerSec = 0.0f;

    float confidence = 0.0f;

    bool isStable = false;

    bool obstacleDetected = false;

    uint32_t timestampMs = 0;

    /// slow down during repeated timeouts. degraded navigation mode
    /// 3 Consecutive Timeouts => Reduce confidence
    /// 10 Consecutive Timeouts => Enter degraded mode (e.g., stop movement, alert user) Sensor unhealthy
    /// 20 Consecutive Timeouts => Critical failure (e.g., require manual reset, alert user) Sensor failure
    bool timeoutOccurred = false;
    uint32_t consecutiveTimeouts = 0;
};