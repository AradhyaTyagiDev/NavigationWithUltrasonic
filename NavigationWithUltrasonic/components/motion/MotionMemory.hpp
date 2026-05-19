//====================================================
// File: MotionMemory.hpp
//====================================================

/*
current wheel speeds
previous targets
steering persistence
braking state
acceleration history
*/

#pragma once

#include <stdint.h>

#include "MotionState.hpp"

//====================================================
// MotionMemory
// Persistent locomotion memory.
//
// Tracks:
//      - wheel history
//      - acceleration history
//      - steering persistence
//      - braking persistence
//      - stable locomotion state
//
//====================================================

struct MotionMemory
{
    // Current locomotion state
    MotionState currentState =
        MotionState::Idle;

    // Previous locomotion state
    MotionState previousState =
        MotionState::Idle;

    // Stable locomotion state
    MotionState stableState =
        MotionState::Idle;

    // Current left wheel speed
    float currentLeftWheelSpeed = 0.0f;

    // Current right wheel speed
    float currentRightWheelSpeed = 0.0f;

    // Previous left wheel speed
    float previousLeftWheelSpeed = 0.0f;

    // Previous right wheel speed
    float previousRightWheelSpeed = 0.0f;

    // Current linear speed
    float currentLinearSpeed = 0.0f;

    // Current angular speed
    float currentAngularSpeedDegPerSec = 0.0f;

    // Previous steering curvature
    float previousSteeringCurvature = 0.0f;

    // State transition timing
    uint32_t stateEntryTimestampMs = 0;

    // Last state transition
    uint32_t lastStateChangeTimestampMs = 0;

    // Stable frame tracking
    uint32_t stableFrameCount = 0;

    // Emergency braking tracking
    uint32_t consecutiveEmergencyBrakingCount = 0;

    // Escape maneuver tracking
    uint32_t consecutiveEscapeManeuvers = 0;

    // Motion cooldown
    uint32_t cooldownUntilTimestampMs = 0;

    // Persistent motion active
    bool persistentMotionActive = false;

    // Braking currently active
    bool brakingActive = false;

    // Stability control active
    bool stabilityControlActive = true;

    // Timing
    uint32_t lastUpdateTimestampMs = 0;
};
