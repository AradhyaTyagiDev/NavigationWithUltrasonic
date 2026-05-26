//====================================================
// File: TB6612DriverMemory.hpp
//====================================================

#pragma once

#include "../MotorDriverTypes.hpp"

#include <stdint.h>

//====================================================
// TB6612DriverMemory
// Runtime memory/state tracking for
// TB6612 motor driver.
// Tracks:
//  - current motor state
//  - PWM state
//  - direction state
//  - braking state
//  - emergency state
//  - startup boost state
//  - transition protection
// This is:
//  - runtime state memory
//  - deterministic
//  - lightweight
struct TB6612DriverMemory
{
    //================================================
    // DRIVER STATE

    // Current driver state
    MotorDriverState currentState = MotorDriverState::Uninitialized;

    // Driver initialized
    bool initialized = false;

    // Emergency stop active
    bool emergencyStopActive = false;

    // Fault detected
    bool faultDetected = false;

    //================================================
    // LEFT MOTOR STATE

    // Current direction
    MotorDirection leftMotorDirection = MotorDirection::Stop;

    // Previous direction
    MotorDirection previousLeftMotorDirection = MotorDirection::Stop;

    // Current PWM duty
    uint32_t currentLeftPWMDuty = 0;

    // Previous PWM duty
    uint32_t previousLeftPWMDuty = 0;

    // Left motor enabled
    bool leftMotorEnabled = false;

    // Left motor braking active
    bool leftMotorBrakingActive = false;

    // Left startup boost active
    bool leftStartupBoostActive = false;

    // Last left startup timestamp
    uint32_t lastLeftStartupTimestampMs = 0;

    // Last left direction change
    uint32_t lastLeftDirectionChangeTimestampMs = 0;

    //================================================
    // RIGHT MOTOR STATE

    // Current direction
    MotorDirection rightMotorDirection = MotorDirection::Stop;

    // Previous direction
    MotorDirection previousRightMotorDirection = MotorDirection::Stop;

    // Current PWM duty
    uint32_t currentRightPWMDuty = 0;

    // Previous PWM duty
    uint32_t previousRightPWMDuty = 0;

    // Right motor enabled
    bool rightMotorEnabled = false;

    // Right motor braking active
    bool rightMotorBrakingActive = false;

    // Right startup boost active
    bool rightStartupBoostActive = false;

    // Last right startup timestamp
    uint32_t lastRightStartupTimestampMs = 0;

    // Last right direction change
    uint32_t lastRightDirectionChangeTimestampMs = 0;

    //================================================
    // DRIVER EXECUTION

    // Last update timestamp
    uint32_t lastUpdateTimestampMs = 0;

    // Last emergency brake timestamp
    uint32_t lastEmergencyBrakeTimestampMs = 0;

    // Active brake mode
    BrakeMode currentBrakeMode =
        BrakeMode::Coast;

    // Total executed commands
    uint64_t executedCommandCount = 0;

    // Rejected command count
    uint64_t rejectedCommandCount = 0;

    // Invalid command count
    uint64_t invalidCommandCount = 0;

    // Last command timestamp
    uint32_t lastCommandTimestampMs = 0;

    // State transition lock
    bool transitionInProgress = false;

    // Safe reverse active
    bool safeReverseSequenceActive = false;

    // PWM ramp targets
    uint32_t targetLeftPWMDuty = 0;
    uint32_t targetRightPWMDuty = 0;
};