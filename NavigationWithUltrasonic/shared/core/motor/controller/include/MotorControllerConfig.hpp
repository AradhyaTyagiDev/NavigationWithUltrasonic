/**
 * Motor hardware execution: PWM limits, ramp rates, inversion, motor constraints, safety cutoffs
 */

//====================================================
// File: MotorControllerConfig.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// MotorControllerConfig: Real-time locomotion execution configuration.
//====================================================
struct MotorControllerConfig
{
    // Update frequency
    uint32_t controlFrequencyHz = 50;

    // Maximum acceleration rate: percent/sec
    float maximumAccelerationPercentPerSec = 1.5f;

    // Maximum deceleration rate
    float maximumDecelerationPercentPerSec = 2.0f;

    // Emergency braking deceleration
    float emergencyBrakeDecelerationPercentPerSec = 5.0f;

    // Minimum effective wheel speed
    float minimumEffectiveSpeedPercent = 0.15f;

    // Startup boost enable
    bool enableStartupBoost = true;

    // Startup boost duration
    uint32_t startupBoostDurationMs = 120;

    // Differential synchronization
    bool enableWheelSynchronization = true;

    // Brake coordination
    bool enableCoordinatedBraking = true;

    // Motion smoothing
    bool enableMotionSmoothing = true;

    // Emergency override
    bool enableEmergencyOverride = true;

    // Reverse transition protection
    bool enableSafeReverseTransition = true;

    // Motor command timeout
    uint32_t motorCommandTimeoutMs = 500;

    // Fault recovery enable
    bool enableFaultRecovery = true;
};