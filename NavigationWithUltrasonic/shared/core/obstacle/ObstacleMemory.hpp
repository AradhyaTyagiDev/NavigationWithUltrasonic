#pragma once

#include <stdint.h>
#include "DangerLevel.hpp"

struct ObstacleMemory
{
    // Obstacle persistence
    bool obstacleActive = false;
    bool obstacleRemembered = false;

    // Last known obstacle state
    DangerLevel lastDangerLevel = DangerLevel::Safe;

    // Last known obstacle properties
    /// Even if obstacle disappears briefly: retain environmental memory
    float lastKnownDistanceCm = 0.0f;
    // predictive avoidance, collision urgency, danger escalation
    float lastKnownVelocityCmPerSec = 0.0f;
    float lastKnownConfidence = 0.0f;

    // Temporal persistence
    uint32_t firstDetectedTimestampMs = 0;
    uint32_t lastSeenTimestampMs = 0;

    // Environment persistence metrics
    uint32_t consecutiveDetectionFrames = 0;
    // temporal obstacle memory, hysteresis, smooth disappearance
    uint32_t consecutiveLostFrames = 0;

    // Stability information
    bool obstacleStable = false;

    // Sensor reliability snapshot
    bool sensorHealthy = true;
};