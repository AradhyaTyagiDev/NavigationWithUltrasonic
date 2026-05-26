//====================================================
// File: RobotRuntimeFlags.hpp
//====================================================

#pragma once

//====================================================
// RobotRuntimeFlags
//====================================================
//
// Global runtime flags.
//
// Tracks:
//      emergency runtime
//      degraded mode
//      safe mode
//      shutdown request
//
//====================================================

struct RobotRuntimeFlags
{
    bool emergencyActive = false;
    // Degraded runtime mode
    bool degradedModeActive = false;
    bool safeModeActive = false;
    bool shutdownRequested = false;
    // Runtime paused
    bool paused = false;
    bool faultRecoveryActive = false;
    // Sensor timeout active
    bool sensorTimeoutActive = false;
    // Motor timeout active
    bool motorTimeoutActive = false;
    // Autonomous navigation active
    bool autonomousModeActive = true;
};