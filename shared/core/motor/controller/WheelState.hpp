//====================================================
// File: WheelState.hpp
//====================================================

#pragma once

#include "../driver/MotorDriverTypes.hpp"

#include <stdint.h>

//====================================================
// WheelState: Runtime wheel execution state.
// Represents: actual wheel execution runtime state
//====================================================
struct WheelState
{
    // Current speed percentage
    float currentSpeedPercent = 0.0f;

    // Target speed percentage
    float targetSpeedPercent = 0.0f;

    // Current PWM duty
    uint32_t currentPWMDuty = 0;

    // Current direction
    MotorDirection currentDirection = MotorDirection::Stop;

    // Current brake mode
    BrakeMode currentBrakeMode = BrakeMode::Coast;

    // Ramp active
    bool rampActive = false;

    // Braking active
    bool brakingActive = false;

    // Emergency brake active
    bool emergencyBrakeActive = false;

    // Wheel enabled
    bool enabled = true;

    // Last update timestamp
    uint32_t lastUpdateTimestampMs = 0;
};