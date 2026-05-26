//====================================================
// File: RobotState.hpp
//====================================================

#pragma once

//====================================================
// RobotState
//====================================================
//
// Global robotics runtime state.
//
// Controlled by:
//      RobotController
//
//====================================================

enum class RobotState
{
    // Power-on startup
    Booting,
    // Runtime initialization
    Initializing,
    // Fully operational
    Active,
    // Temporarily paused
    Paused,
    // Reduced capability mode
    Degraded,
    // Emergency runtime state
    Emergency,
    // Fatal runtime fault
    Fault,
    // Graceful shutdown
    Shutdown
};