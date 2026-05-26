#pragma once

#include <stdint.h>
#include "DangerLevel.hpp"

struct ObstacleAnalysis
{
    // Obstacle state
    bool obstacleDetected = false;
    bool obstacleRemembered = false;
    bool emergencyDetected = false;

    // Environment classification
    DangerLevel dangerLevel = DangerLevel::Safe;
    // Obstacle properties
    float obstacleDistanceCm = 0.0f;
    float obstacleVelocityCmPerSec = 0.0f;

    float confidence = 0.0f;

    // Stability / reliability
    bool isStable = false;

    // Sensor health monitoring: Sensor health / reliability
    bool sensorHealthy = true;
    bool timeoutOccurred = false;
    uint32_t consecutiveTimeouts = 0;

    // Temporal information
    uint32_t timestampMs = 0;
};