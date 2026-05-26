//====================================================
// File: MotorState.hpp
//====================================================

#pragma once

//====================================================
// MotorState
//====================================================
//
// High-level locomotion execution state.
//
// Represents:
//      what robot locomotion system
//      is currently doing.
//
// NOT:
//      low-level hardware state
//
// IMPORTANT 🚀: Some states like Forward, Reverse, Cruising, Executing overlap slightly. But That is NORMAL in robotics.
//          Because: execution state machines are semantic, NOT purely mathematical.
//====================================================

enum class MotorState
{
    Idle,
    Accelerating,
    Cruising, // Stable movement
    Forward,
    Reverse,
    Braking,
    Executing,
    EmergencyStop,
    Fault // Motor execution fault
};