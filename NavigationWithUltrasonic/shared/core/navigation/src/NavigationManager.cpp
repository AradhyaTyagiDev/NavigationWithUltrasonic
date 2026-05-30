//====================================================
// File: NavigationManager.cpp
//====================================================

#include "navigation/include/NavigationManager.hpp"

#include <cmath>
#include <stdlib.h>

//====================================================
// Constructor
//====================================================

NavigationManager::NavigationManager(
    const NavigationManagerConfig &config)
    : m_config(config)
{
}

//====================================================
// Main behavioral pipeline
//====================================================

NavigationDecision
NavigationManager::process(
    const ObstacleAnalysis &obstacleAnalysis,
    uint32_t currentTimestampMs)
{
    NavigationDecision decision;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    decision.timestampMs =
        currentTimestampMs;

    //-----------------------------------------
    // Navigation confidence
    //-----------------------------------------

    decision.navigationConfidence =
        determineNavigationConfidence(
            obstacleAnalysis);

    //-----------------------------------------
    // NaN / Inf protection
    //-----------------------------------------

    if (
        std::isnan(decision.navigationConfidence) ||
        std::isinf(decision.navigationConfidence))
    {
        decision.navigationConfidence = 0.0f;
    }

    //-----------------------------------------
    // Determine desired state
    //-----------------------------------------

    NavigationState targetState =
        determineNavigationState(
            obstacleAnalysis);

    //-----------------------------------------
    // Emergency override
    //-----------------------------------------

    if (
        m_config.enableEmergencyOverride &&
        shouldTriggerEmergency(
            obstacleAnalysis))
    {
        targetState =
            NavigationState::EmergencyStop;

        decision.emergencyOverride = true;
    }

    //-----------------------------------------
    // Blocked-state escalation
    //-----------------------------------------

    if (
        shouldEnterBlockedState())
    {
        targetState =
            NavigationState::Blocked;
    }

    //-----------------------------------------
    // Transition validation
    //-----------------------------------------

    if (
        !canTransitionToState(
            targetState,
            currentTimestampMs))
    {
        targetState =
            m_memory.currentState;
    }

    //-----------------------------------------
    // Final assigned state
    //-----------------------------------------

    decision.state =
        targetState;

    //-----------------------------------------
    // Turn strategy
    //-----------------------------------------

    TurnDirection direction =
        TurnDirection::None;

    if (
        targetState ==
        NavigationState::Avoiding)
    {
        //-------------------------------------
        // Deterministic alternating avoidance
        //-------------------------------------

        direction =
            determineAlternatingDirection();
    }
    else if (
        targetState ==
        NavigationState::EscapeMode)
    {
        //-------------------------------------
        // Randomized escape behavior
        //-------------------------------------

        direction =
            determineRandomDirection();
    }

    //-----------------------------------------
    // Movement action
    //-----------------------------------------

    decision.action =
        determineAction(
            targetState,
            direction);

    //-----------------------------------------
    // Turn direction
    //-----------------------------------------

    decision.turnDirection =
        direction;

    //-----------------------------------------
    // Movement profile
    //-----------------------------------------

    decision.movementProfile =
        determineMovementProfile(
            targetState);

    //-----------------------------------------
    // Speed recommendation
    //-----------------------------------------

    decision.targetSpeedPercent =
        determineTargetSpeed(
            targetState);

    //-----------------------------------------
    // Clamp speed
    //-----------------------------------------

    if (decision.targetSpeedPercent < 0.0f)
    {
        decision.targetSpeedPercent = 0.0f;
    }

    if (decision.targetSpeedPercent > 1.0f)
    {
        decision.targetSpeedPercent = 1.0f;
    }

    //-----------------------------------------
    // Turn angle
    //-----------------------------------------

    decision.desiredTurnAngleDeg =
        determineTurnAngle(
            targetState,
            direction);

    //-----------------------------------------
    // Escape behavior
    //-----------------------------------------

    decision.escapeBehaviorActive =
        (targetState ==
         NavigationState::EscapeMode);

    //-----------------------------------------
    // Avoidance behavior
    //-----------------------------------------

    decision.obstacleAvoidanceActive =
        (targetState ==
         NavigationState::Avoiding);

    //-----------------------------------------
    // Persistence
    //-----------------------------------------

    decision.persistentBehavior =
        shouldPersistBehavior(
            targetState,
            currentTimestampMs);

    //-----------------------------------------
    // Update memory
    //-----------------------------------------

    updateNavigationMemory(
        decision,
        currentTimestampMs);

    return decision;
}

//====================================================
// State determination
//====================================================

NavigationState
NavigationManager::determineNavigationState(
    const ObstacleAnalysis &obstacleAnalysis) const
{
    //-----------------------------------------
    // Unknown / degraded environment
    //-----------------------------------------

    if (
        obstacleAnalysis.dangerLevel ==
        DangerLevel::Unknown)
    {
        return NavigationState::CautiousForward;
    }

    //-----------------------------------------
    // Emergency
    //-----------------------------------------

    if (
        obstacleAnalysis.dangerLevel ==
        DangerLevel::Emergency)
    {
        return NavigationState::EmergencyStop;
    }

    //-----------------------------------------
    // Blocked environment
    //-----------------------------------------

    if (
        obstacleAnalysis.dangerLevel ==
        DangerLevel::Blocked)
    {
        //-------------------------------------
        // Escalate into escape mode first
        //-------------------------------------

        return NavigationState::EscapeMode;
    }

    //-----------------------------------------
    // Obstacle avoidance
    //-----------------------------------------

    if (
        obstacleAnalysis.dangerLevel ==
        DangerLevel::Avoid)
    {
        return NavigationState::Avoiding;
    }

    //-----------------------------------------
    // Cautious navigation
    //-----------------------------------------

    if (
        obstacleAnalysis.dangerLevel ==
        DangerLevel::Caution)
    {
        return NavigationState::CautiousForward;
    }

    //-----------------------------------------
    // Safe environment
    //-----------------------------------------

    return NavigationState::Forward;
}

//====================================================
// Transition validation
//====================================================

bool NavigationManager::canTransitionToState(
    NavigationState newState,
    uint32_t currentTimestampMs) const
{
    //-----------------------------------------
    // Same state
    //-----------------------------------------

    if (
        newState ==
        m_memory.currentState)
    {
        return true;
    }

    //-----------------------------------------
    // Emergency override bypasses cooldown
    //-----------------------------------------

    if (
        newState ==
        NavigationState::EmergencyStop)
    {
        return true;
    }

    //-----------------------------------------
    // Cooldown protection
    //-----------------------------------------

    if (
        currentTimestampMs <
        m_memory.cooldownUntilTimestampMs)
    {
        return false;
    }

    //-----------------------------------------
    // Minimum state persistence
    //-----------------------------------------

    const uint32_t elapsedMs =
        currentTimestampMs -
        m_memory.stateEntryTimestampMs;

    return (
        elapsedMs >=
        m_config.minimumStateDurationMs);
}

//====================================================
// Emergency detection
//====================================================

bool NavigationManager::shouldTriggerEmergency(
    const ObstacleAnalysis &obstacleAnalysis) const
{
    return (
        obstacleAnalysis.emergencyDetected ||
        obstacleAnalysis.dangerLevel ==
            DangerLevel::Emergency);
}

//====================================================
// Blocked-state escalation
//====================================================

bool NavigationManager::shouldEnterBlockedState() const
{
    return (
        m_memory.consecutiveEscapeAttempts >=
        m_config.blockedStateThreshold);
}

//====================================================
// Alternating direction
//====================================================

TurnDirection
NavigationManager::determineAlternatingDirection()
{
    if (
        m_memory.lastTurnDirection ==
        TurnDirection::Left)
    {
        return TurnDirection::Right;
    }

    return TurnDirection::Left;
}

//====================================================
// Random direction
//====================================================

TurnDirection
NavigationManager::determineRandomDirection()
{
    return (
               rand() % 2 == 0)
               ? TurnDirection::Left
               : TurnDirection::Right;
}

//====================================================
// Action generation
//====================================================

NavigationAction
NavigationManager::determineAction(
    NavigationState state,
    TurnDirection direction) const
{
    switch (state)
    {
    case NavigationState::Forward:
    case NavigationState::CautiousForward:

        return NavigationAction::MoveForward;

    case NavigationState::Avoiding:

        return (
                   direction ==
                   TurnDirection::Left)
                   ? NavigationAction::CurveLeft
                   : NavigationAction::CurveRight;

    case NavigationState::EscapeMode:

        return NavigationAction::MoveBackward;

    case NavigationState::EmergencyStop:
    case NavigationState::Blocked:

        return NavigationAction::Stop;

    default:

        return NavigationAction::Stop;
    }
}

//====================================================
// Movement profile
//====================================================

MovementProfile
NavigationManager::determineMovementProfile(
    NavigationState state) const
{
    switch (state)
    {
    case NavigationState::Forward:

        return MovementProfile::Normal;

    case NavigationState::CautiousForward:

        return MovementProfile::Cautious;

    case NavigationState::Avoiding:

        return MovementProfile::Cautious;

    case NavigationState::EscapeMode:

        return MovementProfile::Escape;

    case NavigationState::EmergencyStop:

        return MovementProfile::Emergency;

    default:

        return MovementProfile::Normal;
    }
}

//====================================================
// Speed recommendation
//====================================================

float NavigationManager::determineTargetSpeed(
    NavigationState state) const
{
    switch (state)
    {
    case NavigationState::Forward:

        return m_config.normalSpeedPercent;

    case NavigationState::CautiousForward:

        return m_config.cautiousSpeedPercent;

    case NavigationState::Avoiding:

        return m_config.cautiousSpeedPercent;

    case NavigationState::EscapeMode:

        return m_config.escapeSpeedPercent;

    case NavigationState::EmergencyStop:
    case NavigationState::Blocked:

        return 0.0f;

    default:

        return 0.0f;
    }
}

//====================================================
// Turn angle generation
//====================================================

float NavigationManager::determineTurnAngle(
    NavigationState state,
    TurnDirection direction) const
{
    //-----------------------------------------
    // Cautious forward should remain straight
    //-----------------------------------------

    if (
        state ==
        NavigationState::CautiousForward)
    {
        return 0.0f;
    }

    float angle = 0.0f;

    switch (state)
    {
    case NavigationState::Avoiding:

        angle =
            m_config.avoidanceTurnAngleDeg;

        break;

    case NavigationState::EscapeMode:

        angle =
            m_config.escapeTurnAngleDeg;

        break;

    default:

        return 0.0f;
    }

    //-----------------------------------------
    // Left negative
    // Right positive
    //-----------------------------------------

    return (
               direction ==
               TurnDirection::Left)
               ? -angle
               : angle;
}

//====================================================
// Navigation confidence
//====================================================

float NavigationManager::determineNavigationConfidence(
    const ObstacleAnalysis &obstacleAnalysis) const
{
    float confidence =
        obstacleAnalysis.confidence;

    //-----------------------------------------
    // Clamp
    //-----------------------------------------

    if (confidence < 0.0f)
    {
        confidence = 0.0f;
    }

    if (confidence > 1.0f)
    {
        confidence = 1.0f;
    }

    //-----------------------------------------
    // Confidence adaptation
    //-----------------------------------------

    if (
        m_config.enableConfidenceAdaptation &&
        confidence <
            m_config.lowConfidenceThreshold)
    {
        confidence *= 0.7f;
    }

    return confidence;
}

//====================================================
// Persistent behavior
//====================================================

bool NavigationManager::shouldPersistBehavior(
    NavigationState state,
    uint32_t currentTimestampMs) const
{
    if (!m_config.enableBehaviorPersistence)
    {
        return false;
    }

    switch (state)
    {
    case NavigationState::Avoiding:
    case NavigationState::EscapeMode:
    case NavigationState::EmergencyStop:
    case NavigationState::Blocked:

        return true;

    default:

        return false;
    }
}

//====================================================
// Memory update
//====================================================

void NavigationManager::updateNavigationMemory(
    const NavigationDecision &decision,
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // State transition
    //-----------------------------------------

    if (
        decision.state !=
        m_memory.currentState)
    {
        m_memory.previousState =
            m_memory.currentState;

        m_memory.currentState =
            decision.state;

        m_memory.lastStateChangeTimestampMs =
            currentTimestampMs;

        m_memory.stateEntryTimestampMs =
            currentTimestampMs;

        //-------------------------------------
        // Apply cooldown only for avoidance
        //-------------------------------------

        if (
            decision.state ==
            NavigationState::Avoiding)
        {
            m_memory.cooldownUntilTimestampMs =
                currentTimestampMs +
                m_config.avoidanceCooldownMs;
        }

        m_memory.stableFrameCount = 0;
    }
    else
    {
        //-------------------------------------
        // Stable state tracking
        //-------------------------------------

        if (
            m_memory.stableFrameCount <
            UINT32_MAX)
        {
            m_memory.stableFrameCount++;
        }
    }

    //-----------------------------------------
    // Stable persistent state
    //-----------------------------------------

    if (
        m_memory.stableFrameCount >=
        m_config.minimumStableFrames)
    {
        m_memory.stableState =
            decision.state;
    }

    //-----------------------------------------
    // Turn memory
    //-----------------------------------------

    if (
        decision.turnDirection !=
        TurnDirection::None)
    {
        m_memory.lastTurnDirection =
            decision.turnDirection;
    }

    //-----------------------------------------
    // Escape tracking
    //-----------------------------------------

    if (
        decision.state ==
        NavigationState::EscapeMode)
    {
        if (
            m_memory.consecutiveEscapeAttempts <
            UINT32_MAX)
        {
            m_memory.consecutiveEscapeAttempts++;
        }
    }
    else
    {
        m_memory.consecutiveEscapeAttempts = 0;
    }

    //-----------------------------------------
    // Persistent behavior
    //-----------------------------------------

    m_memory.persistentBehaviorActive =
        decision.persistentBehavior;
}