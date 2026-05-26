//====================================================
// File: MotionCommand.hpp
//====================================================

/*
Module Output: MotionCommand
This structure defines:
 - wheel targets
 - braking
 - steering
 - acceleration
 - motion safety
*/

#pragma once

#include <stdint.h>

#include "MotionState.hpp"

//====================================================
// MotionCommand
// Final executable locomotion target.
//
// Produced by:
//      MotionPlanner
//
// Consumed by:
//      MotorController
//
// IMPORTANT:
//      This is NOT raw PWM.
//====================================================

struct MotionCommand
{
    // Motion state
    MotionState state = MotionState::Idle;

    // Left wheel target
    // Range:
    //      -1.0 → full reverse
    //       0.0 → stop
    //       1.0 → full forward
    float leftWheelSpeed = 0.0f;

    // Right wheel target
    float rightWheelSpeed = 0.0f;

    // Target forward speed
    float targetLinearSpeed = 0.0f;

    // Target turning speed
    float targetAngularSpeedDegPerSec = 0.0f;

    // Desired steering curvature: Positive: right turn, Negative: left turn
    float steeringCurvature = 0.0f;

    // Desired turn angle
    float desiredTurnAngleDeg = 0.0f;

    // Acceleration target
    float targetAcceleration = 0.0f;

    // Deceleration target
    float targetDeceleration = 0.0f;

    // Smooth locomotion enabled
    bool smoothingEnabled = true;

    // Braking active
    bool brakingActive = false;

    // Emergency braking active
    bool emergencyBrakingActive = false;

    // Reverse motion active
    bool reverseMotionActive = false;

    // Escape maneuver active
    bool escapeManeuverActive = false;

    // Motion stability active
    bool stabilityControlActive = true;

    // Motion confidence
    float motionConfidence = 1.0f;

    // Timestamp
    uint32_t timestampMs = 0;
};