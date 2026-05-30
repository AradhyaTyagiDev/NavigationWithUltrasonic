//

//====================================================
// File: MotorControllerMemory.hpp
//====================================================

#pragma once

#include "MotorState.hpp"
#include "WheelState.hpp"

#include <stdint.h>

//====================================================
// MotorControllerMemory: Store runtime locomotion execution state
// Tracks: execution state, synchronization, braking, emergency state, runtime coordination
//====================================================
struct MotorControllerMemory
{
    // Current motor execution state
    MotorState currentState = MotorState::Idle;

    // Previous execution state
    MotorState previousState = MotorState::Idle;

    // Left wheel runtime state
    WheelState leftWheelState;

    // Right wheel runtime state
    WheelState rightWheelState;

    // Emergency state active
    bool emergencyStopActive = false;

    // Coordinated braking active
    bool coordinatedBrakingActive = false;

    // Reverse transition active
    bool reverseTransitionActive = false;

    // Motion execution active
    bool motionExecutionActive = false;

    // Synchronization active
    bool wheelSynchronizationActive = false;

    // Fault state active
    bool faultActive = false;

    // Current execution sequence ID
    uint32_t currentSequenceId = 0;

    // Executed command count
    uint64_t executedCommandCount = 0;

    // Rejected command count
    uint64_t rejectedCommandCount = 0;

    // Last command timestamp
    uint32_t lastCommandTimestampMs = 0;

    // Last state transition timestamp
    uint32_t lastStateTransitionTimestampMs = 0;

    // Last emergency timestamp
    uint32_t lastEmergencyTimestampMs = 0;

    // Last synchronization timestamp
    uint32_t lastSynchronizationTimestampMs = 0;
};