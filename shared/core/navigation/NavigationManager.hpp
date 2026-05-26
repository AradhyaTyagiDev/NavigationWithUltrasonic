//====================================================
// File: NavigationManager.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "../obstacle/ObstacleAnalysis.hpp"

#include "NavigationDecision.hpp"
#include "NavigationMemory.hpp"
#include "NavigationManagerConfig.hpp"

//====================================================
// NavigationManager
//====================================================
//
// Behavioral decision engine
// that converts environmental semantics
// into navigation intent.
//
// Input:
//      ObstacleAnalysis
//
// Output:
//      NavigationDecision
//
// Responsibilities:
//      - Behavioral decision making
//      - Navigation state machine
//      - Emergency handling
//      - Escape behavior
//      - Behavioral persistence
//      - Direction strategy
//      - Navigation safety adaptation
//      - Stable state transitions
//
// This layer DOES NOT:
//      - Generate motor PWM
//      - Execute trajectories
//      - Control hardware
//      - Perform path planning
//
//====================================================

class NavigationManager
{
public:
    //================================================
    // Constructor
    //================================================

    explicit NavigationManager(
        const NavigationManagerConfig &config);

    //================================================
    // Main behavioral pipeline
    //================================================
    //
    // Converts:
    //      ObstacleAnalysis
    //
    // Into:
    //      NavigationDecision
    //
    //================================================

    NavigationDecision process(
        const ObstacleAnalysis &obstacleAnalysis,
        uint32_t currentTimestampMs);

private:
    //================================================
    // Navigation state determination
    //================================================
    //
    // Interprets environmental semantics
    // into behavioral navigation state.
    //
    //================================================

    NavigationState determineNavigationState(
        const ObstacleAnalysis &obstacleAnalysis) const;

    //================================================
    // State transition validation
    //================================================
    //
    // Prevents:
    //      - rapid oscillation
    //      - unstable transitions
    //      - unsafe behavior flipping
    //
    //================================================

    bool canTransitionToState(
        NavigationState newState,
        uint32_t currentTimestampMs) const;

    //================================================
    // Emergency detection
    //================================================
    //
    // Determines whether emergency override
    // should immediately interrupt navigation.
    //
    //================================================

    bool shouldTriggerEmergency(
        const ObstacleAnalysis &obstacleAnalysis) const;

    //================================================
    // Blocked-state escalation
    //================================================
    //
    // Determines whether robot is trapped
    // and should enter blocked state.
    //
    //================================================

    bool shouldEnterBlockedState() const;

    //================================================
    // Alternating avoidance direction
    //================================================
    //
    // Deterministic:
    //      Left -> Right -> Left
    //
    // Prevents:
    //      repeated turning loops
    //
    //================================================

    TurnDirection determineAlternatingDirection();

    //================================================
    // Randomized escape direction
    //================================================
    //
    // Used during:
    //      EscapeMode
    //
    // Helps robot escape trapped states.
    //
    //================================================

    TurnDirection determineRandomDirection();

    //================================================
    // Movement action generation
    //================================================
    //
    // Converts navigation state
    // into high-level movement intent.
    //
    //================================================

    NavigationAction determineAction(
        NavigationState state,
        TurnDirection direction) const;

    //================================================
    // Movement profile generation
    //================================================
    //
    // Determines:
    //      - navigation aggressiveness
    //      - motion behavior style
    //
    //================================================

    MovementProfile determineMovementProfile(
        NavigationState state) const;

    //================================================
    // Target speed generation
    //================================================
    //
    // Generates:
    //      target speed percentage
    //
    //================================================

    float determineTargetSpeed(
        NavigationState state) const;

    //================================================
    // Turn angle generation
    //================================================
    //
    // Generates:
    //      desired steering angle
    //
    // Convention:
    //      Left  = negative angle
    //      Right = positive angle
    //
    //================================================

    float determineTurnAngle(
        NavigationState state,
        TurnDirection direction) const;

    //================================================
    // Navigation confidence evaluation
    //================================================
    //
    // Applies:
    //      - degraded safety adaptation
    //      - confidence stabilization
    //      - confidence clamping
    //
    //================================================

    float determineNavigationConfidence(
        const ObstacleAnalysis &obstacleAnalysis) const;

    //================================================
    // Persistent behavior evaluation
    //================================================
    //
    // Determines whether behavior
    // should remain stable temporarily.
    //
    //================================================

    bool shouldPersistBehavior(
        NavigationState state,
        uint32_t currentTimestampMs) const;

    //================================================
    // Behavioral memory update
    //================================================
    //
    // Updates:
    //      - state transitions
    //      - stable state tracking
    //      - escape tracking
    //      - turn memory
    //      - cooldown tracking
    //
    //================================================

    void updateNavigationMemory(
        const NavigationDecision &decision,
        uint32_t currentTimestampMs);

private:
    //================================================
    // Configuration
    //================================================

    NavigationManagerConfig m_config;

    //================================================
    // Behavioral memory
    //================================================

    NavigationMemory m_memory;
};