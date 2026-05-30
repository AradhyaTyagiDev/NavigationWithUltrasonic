//====================================================
// File: MotionState.hpp
//====================================================

/*
Defines locomotion state machine.
 - Idle
 - Accelerating
 - Cruising
 - Turning
 - Braking
 - Escaping
 - EmergencyBraking
*/

//====================================================
// MotionState
//====================================================
// Physical locomotion state machine.
// Represents:
//      HOW robot is physically moving.
//====================================================

#pragma once

enum class MotionState
{
    // No active locomotion
    Idle,
    // Increasing velocity
    Accelerating,
    // Stable continuous movement
    Cruising,
    // Steering / curvature motion
    Turning,
    // Controlled deceleration
    Braking,
    // Escape / recovery maneuver
    Escaping,
    // Immediate safety braking
    EmergencyBraking
};