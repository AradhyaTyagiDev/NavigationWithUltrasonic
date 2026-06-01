#pragma once

#include "interfaces/include/motor/driver/MotorDriverTypes.hpp"

#include <stdint.h>

//====================================================
// MotorDriverCommand
// Hardware-neutral motor command.
// Produced by: MotorController, Consumed by: IMotorDriver
//====================================================

struct MotorDriverCommand
{
    // Target channel
    MotorChannel channel = MotorChannel::Left;

    // Direction
    MotorDirection direction = MotorDirection::Stop;

    // Braking
    BrakeMode brakeMode = BrakeMode::Coast;

    // Normalized motor Speed: 0.0f -> 1.0f
    // Hardware layer converts:  normalizedSpeed -> PWM duty
    // Examples: 0.0f = stop, 0.5f = 50% speed, 1.0f = maximum speed
    // Hardware drivers convert this value
    // into platform-specific PWM duty cycles.
    float normalizedSpeed = 0.0f;

    // Motor output enabled. Enable state
    // false: Driver disables motor output.
    // true: Driver may drive motor according to command parameters.
    bool enabled = true;

    // Emergency stop request.
    // When true: Driver immediately enters, emergency stop state and ignores, normal motion commands.
    bool emergencyStop = false;

    // Sequence identifier
    // Useful for: - debugging,  - telemetry, - synchronization
    uint32_t sequenceId = 0;
};