#pragma once

#include <stdint.h>

// environment interpretation, hysteresis, danger zones obstacle classification, decision thresholds,
//  - temporal persistence, sensor health management
struct ObstacleManagerConfig
{
    //-----------------------------------------
    // Danger zone thresholds
    // Defines semantic environment regions.
    // Safe:
    //      > cautionDistanceCm
    // Caution:
    //      cautionDistanceCm -> avoidDistanceCm
    // Avoid:
    //      avoidDistanceCm -> emergencyDistanceCm
    // Emergency:
    //      < emergencyDistanceCm
    float cautionDistanceCm = 100.0f;
    float avoidDistanceCm = 50.0f;
    float emergencyDistanceCm = 20.0f;

    //-----------------------------------------
    // Hysteresis
    // Prevents rapid danger-level oscillation.
    // Example:
    //      Avoid enters at 50cm
    //      Avoid exits at 55cm
    float hysteresisCm = 5.0f;

    //-----------------------------------------
    // Confidence gating
    // Minimum trusted perception confidence.
    // Lower-confidence obstacles may be:
    //      ignored
    //      downgraded
    //      remembered cautiously
    float minimumConfidence = 0.5f;

    //-----------------------------------------
    // Stability requirements
    // Minimum stable frames before obstacle
    // becomes trusted by decision layer.
    uint32_t minimumStableFrames = 3;

    //-----------------------------------------
    // Temporal obstacle persistence
    // Remembers obstacle briefly after loss.
    // Prevents:
    //      flickering
    //      unstable navigation
    //      rapid state switching
    uint32_t obstacleMemoryMs = 500;

    //-----------------------------------------
    // Consecutive detection requirements
    // Helps validate obstacle persistence.
    uint32_t minimumDetectionFrames = 2;

    //-----------------------------------------
    // Consecutive lost-frame tolerance
    // Prevents immediate obstacle clearing.
    uint32_t maximumLostFrames = 3;

    //-----------------------------------------
    // Sensor health management
    // Maximum tolerated timeout count before
    // sensor becomes unhealthy.
    uint32_t maxTimeoutTolerance = 10;

    //-----------------------------------------
    // Emergency escalation
    // Rapidly approaching obstacles may trigger
    // emergency behavior even before entering
    // emergency distance.
    float emergencyVelocityThresholdCmPerSec =
        -150.0f;

    //-----------------------------------------
    // Dynamic safety scaling
    // Future-ready support for:
    //      speed-adaptive danger zones
    //      cautious mode
    //      aggressive mode
    bool enableDynamicThresholdScaling = true;

    //-----------------------------------------
    // Obstacle memory behavior
    // Allows remembered obstacles to continue
    // influencing navigation after brief loss.
    bool enableObstacleMemory = true;

    //-----------------------------------------
    // Confidence-aware obstacle handling
    bool enableConfidenceGating = true;

    //-----------------------------------------
    // Stability-aware obstacle handling
    bool enableStabilityValidation = true;

    //-----------------------------------------
    // Emergency override
    // Allows emergency danger level to bypass
    // normal confidence/stability constraints.
    bool enableEmergencyOverride = true;
};