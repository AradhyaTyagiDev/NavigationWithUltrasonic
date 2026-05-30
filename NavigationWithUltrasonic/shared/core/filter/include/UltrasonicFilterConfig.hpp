#pragma once

#include <stdint.h>

// Define signal processing behavior and thresholds for ultrasonic sensor data
struct UltrasonicFilterConfig
{
    float minDistanceCm = 2.0f;

    float maxDistanceCm = 400.0f;

    float maxRealisticVelocityCmPerSec = 300.0f;

    float deadZoneCm = 2.0f;

    float emaAlpha = 0.6f;

    float largeJumpThresholdCm = 50.0f;

    float smallJumpThresholdCm = 10.0f;

    float obstacleThresholdCm = 30.0f;

    uint32_t requiredStableFrames = 3;

    uint32_t timeoutDistanceCm = 400;

    float confidenceDecayAlpha = 0.3f;

    float adaptiveEMAStableDeltaCm = 3.0f;

    float adaptiveEMAModerateDeltaCm = 15.0f;
};