//====================================================
// File: MotorDriverCapabilities.hpp
//====================================================

#pragma once

// MotorDriverCapabilities: Describes hardware driver capabilities.
struct MotorDriverCapabilities
{
    // Active braking support
    bool supportsActiveBraking = false;

    // Reverse support
    bool supportsReverse = true;

    // PWM support
    bool supportsPWM = true;

    // Dual channel support
    bool supportsDualChannel = true;

    // Current sensing
    bool supportsCurrentFeedback = false;

    // Encoder support
    bool supportsEncoderFeedback = false;

    // Emergency stop support
    bool supportsEmergencyStop = true;
};