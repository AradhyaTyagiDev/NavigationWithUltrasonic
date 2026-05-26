//====================================================
// File: EmergencyState.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// EmergencySeverity
//====================================================
enum class EmergencySeverity
{
    // No emergency
    None,

    // Minor runtime issue
    Warning,

    // Critical runtime issue
    Critical,

    // Fatal runtime issue
    Fatal
};

//====================================================
// EmergencySource
//====================================================
enum class EmergencySource
{
    // Unknown source
    Unknown,

    // Sensor subsystem
    Sensor,

    // Obstacle subsystem
    Obstacle,

    // Navigation subsystem
    Navigation,

    // Motion subsystem
    Motion,

    // Motor subsystem
    Motor,

    // Driver subsystem
    Driver,

    // Runtime timing subsystem
    Timing,

    // Robot controller subsystem
    Runtime
};

enum class EmergencyCode
{
    None,

    SensorTimeout,

    SensorFailure,

    ObstacleCollisionRisk,

    MotionInstability,

    MotorDriverFault,

    RuntimeTimingViolation,

    PipelineTimeout,

    EmergencyStopTriggered,

    Unknown
};

//====================================================
// EmergencyState
//====================================================
// Global robotics emergency runtime state.
//====================================================
struct EmergencyState
{
    // Emergency active
    bool emergencyActive = false;

    // Emergency severity
    EmergencySeverity severity = EmergencySeverity::None;

    // Emergency source
    EmergencySource source = EmergencySource::Unknown;

    EmergencyCode code = EmergencyCode::None;

    // Emergency stop triggered
    bool emergencyStopTriggered = false;

    // Recovery required
    bool recoveryRequired = false;

    // Manual reset required
    bool manualResetRequired = false;

    // Consecutive emergency count
    uint32_t consecutiveEmergencyCount = 0;

    // Last emergency timestamp
    uint32_t lastEmergencyTimestampMs = 0;

    // Emergency recovery timestamp
    uint32_t recoveryTimestampMs = 0;
};