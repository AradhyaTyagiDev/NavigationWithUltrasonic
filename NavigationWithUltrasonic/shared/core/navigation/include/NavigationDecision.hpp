/*
Final behavioral output.
Consumed later by: MotionPlanner
*/

#pragma once

#include <stdint.h>

#include "NavigationState.hpp"
#include "NavigationAction.hpp"
#include "TurnDirection.hpp"
#include "MovementProfile.hpp"

struct NavigationDecision
{
    // Navigation state
    NavigationState state = NavigationState::Idle;

    // Desired movement action
    NavigationAction action = NavigationAction::Stop;

    // Turn recommendation
    TurnDirection turnDirection = TurnDirection::None;

    // Speed behavior profile
    MovementProfile movementProfile =
        MovementProfile::Normal;

    // Target speed percentage: Desired navigation speed. Useful to Turn and movement
    float targetSpeedPercent = 0.0f;

    // Desired turn angle: Positive: right turn, Negative: left turn
    // 0   = straight, 15  = gentle curve, 45  = normal avoidance, 90  = aggressive turn, 180 = escape rotation
    float desiredTurnAngleDeg = 0.0f;

    // Desired turn aggressiveness, slow smooth turn, aggressive fast turn
    float desiredTurnRateDegPerSec = 0.0f;

    // Desired stopping buffer distance. Useful for obstacle avoidance and emergency stop
    // Supports: predictive braking, dynamic safety margins, cautious mode, smooth navigation
    float stopDistanceCm = 0.0f;

    // Emergency behavior
    bool emergencyOverride = false;

    // Escape behavior active
    bool escapeBehaviorActive = false;

    // Obstacle avoidance active
    bool obstacleAvoidanceActive = false;

    // Decision confidence
    float navigationConfidence = 1.0f;

    // Behavioral persistence
    bool persistentBehavior = false;

    // Timestamp
    uint32_t timestampMs = 0;
};