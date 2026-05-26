// Motion generation:
//  - Speed control, acceleration limits, path planning parameters, behavior modes (aggressive vs conservative)
//  - Turning curves, trajectory smoothing
/*
Controls:
 - acceleration
 - smoothing
 - steering aggressiveness
 - wheel saturation
 - braking
*/

//====================================================
// File: MotionPlannerConfig.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// MotionPlannerConfig
//====================================================
//
// Motion behavior tuning configuration.
//
// Controls:
//      - acceleration
//      - smoothing
//      - steering
//      - braking
//      - wheel saturation
//      - locomotion stability
//
//====================================================

struct MotionPlannerConfig
{
    // Maximum wheel speed
    float maxWheelSpeed = 1.0f;

    // Maximum reverse speed
    float maxReverseWheelSpeed = 0.6f;

    // Maximum acceleration
    float maxAccelerationPerSec = 0.15f;

    // Maximum deceleration
    float maxDecelerationPerSec = 0.25f;

    // Emergency braking deceleration
    float emergencyBrakeDecelerationPerSec = 0.5f;

    // Maximum steering aggressiveness
    float maxSteeringAggressiveness = 1.0f;

    // Maximum steering rate
    float maxSteeringRateDegPerSec = 120.0f;

    // Maximum steering curvature
    float maxSteeringCurvature = 1.0f;

    // Motion smoothing alpha
    float smoothingAlpha = 0.7f;

    // Wheel speed smoothing alpha
    float wheelSpeedSmoothingAlpha = 0.6f;

    // Steering smoothing alpha
    float steeringSmoothingAlpha = 0.5f;

    // Predictive braking distance
    float predictiveBrakingDistanceCm = 25.0f;

    // Stable motion frames
    uint32_t minimumStableFrames = 3;

    // Minimum motion persistence
    uint32_t minimumMotionDurationMs = 250;

    // Motion cooldown
    uint32_t motionCooldownMs = 200;

    // Escape maneuver duration
    uint32_t escapeManeuverDurationMs = 700;

    // Emergency braking duration
    uint32_t emergencyBrakeDurationMs = 500;

    // Enable locomotion smoothing
    bool enableMotionSmoothing = true;

    // Enable predictive braking
    bool enablePredictiveBraking = true;

    // Enable wheel stabilization
    bool enableWheelStabilization = true;

    // Enable motion persistence
    bool enableMotionPersistence = true;

    // Enable emergency braking
    bool enableEmergencyBraking = true;

    // Enable stability control
    bool enableStabilityControl = true;

    // Pivot turning
    float pivotTurnThresholdDeg = 45.0f;
    float pivotTurnSpeed = 0.5f;

    // Wheel normalization
    float wheelNormalizationLimit = 1.0f;

    // Steering protection
    float steeringRateLimitPerSec = 4.0f;

    // Wheel deadzone
    float wheelDeadzone = 0.05f;

    // Emergency braking
    uint32_t minimumEmergencyBrakeDurationMs = 300;
};