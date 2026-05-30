//====================================================
// File: MotorDriverStatus.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "MotorDriverTypes.hpp"

// MotorDriverStatus: Runtime hardware driver state.
struct MotorDriverStatus
{
    // Driver state
    MotorDriverState state = MotorDriverState::Uninitialized;

    // Fault state
    bool faultDetected = false;

    // Emergency stop active
    bool emergencyStopActive = false;

    // Left motor running
    bool leftMotorRunning = false;

    // Right motor running
    bool rightMotorRunning = false;

    // Current left PWM
    uint32_t currentLeftPWM = 0;

    // Current right PWM
    uint32_t currentRightPWM = 0;

    // Last update timestamp
    uint32_t lastUpdateTimestampMs = 0;
};