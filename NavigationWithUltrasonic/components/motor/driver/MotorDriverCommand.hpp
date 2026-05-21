//====================================================
// File: MotorDriverCommand.hpp
//====================================================

#pragma once

#include "MotorDriverTypes.hpp"

//====================================================
// MotorDriverCommand
//====================================================
//
// Low-level hardware-safe motor command.
//
// Produced by:
//      MotorController
//
// Consumed by:
//      IMotorDriver
//
//====================================================

struct MotorDriverCommand
{
    // Target motor channel
    MotorChannel channel = MotorChannel::Left;

    // Direction
    MotorDirection direction = MotorDirection::Stop;

    // Brake mode
    BrakeMode brakeMode = BrakeMode::Coast;

    //-----------------------------------------
    // Normalized speed
    // Range:
    //      0.0 → 1.0
    float normalizedSpeed = 0.0f;

    // PWM duty
    uint32_t pwmDuty = 0;

    // Enable braking
    bool brakingEnabled = false;

    // Emergency stop
    bool emergencyStop = false;

    // Timestamp
    uint32_t timestampMs = 0;
};