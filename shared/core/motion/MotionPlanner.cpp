//====================================================
// File: MotionPlanner.cpp
//====================================================

#include "MotionPlanner.hpp"

#include <algorithm>
#include <cmath>

//====================================================
// Constructor
//====================================================

MotionPlanner::MotionPlanner(
    const MotionPlannerConfig &config)
    : m_config(config)
{
}

//====================================================
// Main locomotion planning pipeline
//====================================================

MotionCommand MotionPlanner::process(
    const NavigationDecision &navigationDecision,
    uint32_t currentTimestampMs)
{
    MotionCommand command;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    command.timestampMs =
        currentTimestampMs;

    //-----------------------------------------
    // Delta time
    //-----------------------------------------

    float deltaTimeSec = 0.02f;

    if (
        m_memory.lastUpdateTimestampMs > 0)
    {
        deltaTimeSec =
            static_cast<float>(
                currentTimestampMs -
                m_memory.lastUpdateTimestampMs) /
            1000.0f;

        deltaTimeSec =
            std::clamp(
                deltaTimeSec,
                0.001f,
                0.1f);
    }

    //-----------------------------------------
    // Determine target state
    //-----------------------------------------

    MotionState targetState =
        determineMotionState(
            navigationDecision);

    //-----------------------------------------
    // Emergency override
    //-----------------------------------------

    if (
        m_config.enableEmergencyBraking &&
        shouldTriggerEmergencyBraking(
            navigationDecision))
    {
        targetState =
            MotionState::EmergencyBraking;
    }

    //-----------------------------------------
    // Emergency brake persistence
    //-----------------------------------------

    if (
        isEmergencyBrakeLocked(
            currentTimestampMs))
    {
        targetState =
            MotionState::EmergencyBraking;
    }

    //-----------------------------------------
    // Escape timeout
    //-----------------------------------------

    if (
        targetState ==
        MotionState::Escaping)
    {
        const uint32_t elapsed =
            currentTimestampMs -
            m_memory.stateEntryTimestampMs;

        if (
            elapsed >
            m_config.escapeManeuverDurationMs)
        {
            targetState =
                MotionState::Turning;
        }
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
    // Assign state
    //-----------------------------------------

    command.state =
        targetState;

    //-----------------------------------------
    // Confidence
    //-----------------------------------------

    command.motionConfidence =
        std::clamp(
            navigationDecision.navigationConfidence,
            0.0f,
            1.0f);

    //-----------------------------------------
    // Motion target generation
    //-----------------------------------------

    float linearSpeed =
        calculateTargetLinearSpeed(
            navigationDecision);

    float angularSpeed =
        calculateTargetAngularSpeed(
            navigationDecision);

    float steeringCurvature =
        calculateSteeringCurvature(
            navigationDecision);

    //-----------------------------------------
    // Steering rate limiter
    //-----------------------------------------

    steeringCurvature =
        applySteeringRateLimit(
            steeringCurvature,
            m_memory.previousSteeringCurvature,
            deltaTimeSec);

    //-----------------------------------------
    // Steering smoothing
    //-----------------------------------------

    if (
        m_config.enableMotionSmoothing)
    {
        steeringCurvature =
            applySteeringSmoothing(
                steeringCurvature,
                m_memory.previousSteeringCurvature);
    }

    //-----------------------------------------
    // Differential drive planning
    //-----------------------------------------

    float leftWheelTarget = 0.0f;
    float rightWheelTarget = 0.0f;

    generateDifferentialDriveTargets(
        linearSpeed,
        angularSpeed,
        steeringCurvature,
        leftWheelTarget,
        rightWheelTarget);

    //-----------------------------------------
    // Unsafe reverse transition protection
    //-----------------------------------------

    if (
        isUnsafeDirectionChange(
            leftWheelTarget,
            m_memory.currentLeftWheelSpeed))
    {
        leftWheelTarget = 0.0f;
    }

    if (
        isUnsafeDirectionChange(
            rightWheelTarget,
            m_memory.currentRightWheelSpeed))
    {
        rightWheelTarget = 0.0f;
    }

    //-----------------------------------------
    // Acceleration / deceleration separation
    //-----------------------------------------

    if (
        fabs(leftWheelTarget) >
        fabs(m_memory.currentLeftWheelSpeed))
    {
        leftWheelTarget =
            applyAccelerationLimit(
                leftWheelTarget,
                m_memory.currentLeftWheelSpeed,
                deltaTimeSec);
    }
    else
    {
        leftWheelTarget =
            applyDecelerationLimit(
                leftWheelTarget,
                m_memory.currentLeftWheelSpeed,
                deltaTimeSec);
    }

    if (
        fabs(rightWheelTarget) >
        fabs(m_memory.currentRightWheelSpeed))
    {
        rightWheelTarget =
            applyAccelerationLimit(
                rightWheelTarget,
                m_memory.currentRightWheelSpeed,
                deltaTimeSec);
    }
    else
    {
        rightWheelTarget =
            applyDecelerationLimit(
                rightWheelTarget,
                m_memory.currentRightWheelSpeed,
                deltaTimeSec);
    }

    //-----------------------------------------
    // Wheel smoothing
    //-----------------------------------------

    if (
        m_config.enableMotionSmoothing)
    {
        leftWheelTarget =
            applyWheelSpeedSmoothing(
                leftWheelTarget,
                m_memory.currentLeftWheelSpeed);

        rightWheelTarget =
            applyWheelSpeedSmoothing(
                rightWheelTarget,
                m_memory.currentRightWheelSpeed);
    }

    //-----------------------------------------
    // Saturation
    //-----------------------------------------

    applyWheelSaturation(
        leftWheelTarget,
        rightWheelTarget);

    //-----------------------------------------
    // Deadzone
    //-----------------------------------------

    leftWheelTarget =
        applyWheelDeadzone(
            leftWheelTarget);

    rightWheelTarget =
        applyWheelDeadzone(
            rightWheelTarget);

    //-----------------------------------------
    // NaN / Inf protection
    //-----------------------------------------

    if (
        std::isnan(leftWheelTarget) ||
        std::isinf(leftWheelTarget))
    {
        leftWheelTarget = 0.0f;
    }

    if (
        std::isnan(rightWheelTarget) ||
        std::isinf(rightWheelTarget))
    {
        rightWheelTarget = 0.0f;
    }

    //-----------------------------------------
    // Final wheel targets
    //-----------------------------------------

    command.leftWheelSpeed =
        leftWheelTarget;

    command.rightWheelSpeed =
        rightWheelTarget;

    //-----------------------------------------
    // Motion targets
    //-----------------------------------------

    command.targetLinearSpeed =
        linearSpeed;

    command.targetAngularSpeedDegPerSec =
        angularSpeed;

    command.steeringCurvature =
        steeringCurvature;

    command.desiredTurnAngleDeg =
        navigationDecision.desiredTurnAngleDeg;

    //-----------------------------------------
    // Flags
    //-----------------------------------------

    command.smoothingEnabled =
        m_config.enableMotionSmoothing;

    command.stabilityControlActive =
        m_config.enableStabilityControl;

    command.reverseMotionActive =
        (linearSpeed < 0.0f);

    command.escapeManeuverActive =
        (targetState ==
         MotionState::Escaping);

    //-----------------------------------------
    // Predictive braking
    //-----------------------------------------

    if (
        m_config.enablePredictiveBraking)
    {
        applyPredictiveBraking(
            command,
            navigationDecision);
    }

    //-----------------------------------------
    // Emergency braking
    //-----------------------------------------

    if (
        targetState ==
        MotionState::EmergencyBraking)
    {
        applyEmergencyBraking(
            command);
    }

    //-----------------------------------------
    // Persistence
    //-----------------------------------------

    command.stabilityControlActive =
        shouldPersistMotion(
            targetState,
            currentTimestampMs);

    //-----------------------------------------
    // Update memory
    //-----------------------------------------

    updateMotionMemory(
        command,
        currentTimestampMs);

    return command;
}

//====================================================
// Motion state determination
//====================================================

MotionState MotionPlanner::determineMotionState(
    const NavigationDecision &navigationDecision) const
{
    switch (navigationDecision.state)
    {
    case NavigationState::Forward:

        return MotionState::Cruising;

    case NavigationState::CautiousForward:

        return MotionState::Cruising;

    case NavigationState::Avoiding:

        return MotionState::Turning;

    case NavigationState::EscapeMode:

        return MotionState::Escaping;

    case NavigationState::EmergencyStop:

        return MotionState::EmergencyBraking;

    case NavigationState::Blocked:

        return MotionState::Braking;

    default:

        return MotionState::Idle;
    }
}

//====================================================
// Transition validation
//====================================================

bool MotionPlanner::canTransitionToState(
    MotionState newState,
    uint32_t currentTimestampMs) const
{
    if (
        newState ==
        m_memory.currentState)
    {
        return true;
    }

    //-----------------------------------------
    // Emergency override bypass
    //-----------------------------------------

    if (
        newState ==
        MotionState::EmergencyBraking)
    {
        return true;
    }

    //-----------------------------------------
    // Cooldown
    //-----------------------------------------

    if (
        currentTimestampMs <
        m_memory.cooldownUntilTimestampMs)
    {
        return false;
    }

    //-----------------------------------------
    // Minimum persistence
    //-----------------------------------------

    const uint32_t elapsed =
        currentTimestampMs -
        m_memory.stateEntryTimestampMs;

    return (
        elapsed >=
        m_config.minimumMotionDurationMs);
}

//====================================================
// Emergency braking detection
//====================================================

bool MotionPlanner::shouldTriggerEmergencyBraking(
    const NavigationDecision &navigationDecision) const
{
    return (
        navigationDecision.emergencyOverride);
}

//====================================================
// Emergency brake persistence
//====================================================

bool MotionPlanner::isEmergencyBrakeLocked(
    uint32_t currentTimestampMs) const
{
    if (
        m_memory.currentState !=
        MotionState::EmergencyBraking)
    {
        return false;
    }

    const uint32_t elapsed =
        currentTimestampMs -
        m_memory.stateEntryTimestampMs;

    return (
        elapsed <
        m_config.minimumEmergencyBrakeDurationMs);
}

//====================================================
// Linear speed planning
//====================================================

float MotionPlanner::calculateTargetLinearSpeed(
    const NavigationDecision &navigationDecision) const
{
    float speed =
        navigationDecision.targetSpeedPercent;

    //-----------------------------------------
    // Escape reverse motion
    //-----------------------------------------

    if (
        navigationDecision.escapeBehaviorActive)
    {
        speed *= -1.0f;
    }

    return speed;
}

//====================================================
// Angular speed planning
//====================================================

float MotionPlanner::calculateTargetAngularSpeed(
    const NavigationDecision &navigationDecision) const
{
    return navigationDecision.desiredTurnAngleDeg;
}

//====================================================
// Steering curvature planning
//====================================================

float MotionPlanner::calculateSteeringCurvature(
    const NavigationDecision &navigationDecision) const
{
    float curvature =
        navigationDecision.desiredTurnAngleDeg /
        90.0f;

    return std::clamp(
        curvature,
        -m_config.maxSteeringCurvature,
        m_config.maxSteeringCurvature);
}

//====================================================
// Differential drive generation
//====================================================

void MotionPlanner::generateDifferentialDriveTargets(
    float linearSpeed,
    float angularSpeed,
    float steeringCurvature,
    float &leftWheelTarget,
    float &rightWheelTarget) const
{
    //-----------------------------------------
    // Pivot turning
    //-----------------------------------------

    if (
        fabs(angularSpeed) >=
        m_config.pivotTurnThresholdDeg)
    {
        const float pivotSpeed =
            m_config.pivotTurnSpeed;

        leftWheelTarget =
            (angularSpeed > 0.0f)
                ? -pivotSpeed
                : pivotSpeed;

        rightWheelTarget =
            (angularSpeed > 0.0f)
                ? pivotSpeed
                : -pivotSpeed;

        return;
    }

    //-----------------------------------------
    // Differential steering
    //-----------------------------------------

    leftWheelTarget =
        linearSpeed -
        steeringCurvature;

    rightWheelTarget =
        linearSpeed +
        steeringCurvature;

    //-----------------------------------------
    // Normalize
    //-----------------------------------------

    const float maxMagnitude =
        std::max(
            fabs(leftWheelTarget),
            fabs(rightWheelTarget));

    if (
        maxMagnitude >
        m_config.wheelNormalizationLimit)
    {
        leftWheelTarget /=
            maxMagnitude;

        rightWheelTarget /=
            maxMagnitude;
    }
}

//====================================================
// Wheel smoothing
//====================================================

float MotionPlanner::applyWheelSpeedSmoothing(
    float targetSpeed,
    float currentSpeed) const
{
    return (
               m_config.wheelSpeedSmoothingAlpha *
               targetSpeed) +
           ((1.0f -
             m_config.wheelSpeedSmoothingAlpha) *
            currentSpeed);
}

//====================================================
// Steering smoothing
//====================================================

float MotionPlanner::applySteeringSmoothing(
    float targetCurvature,
    float currentCurvature) const
{
    return (
               m_config.steeringSmoothingAlpha *
               targetCurvature) +
           ((1.0f -
             m_config.steeringSmoothingAlpha) *
            currentCurvature);
}

//====================================================
// Steering rate limiting
//====================================================

float MotionPlanner::applySteeringRateLimit(
    float target,
    float current,
    float deltaTimeSec) const
{
    const float maxDelta =
        m_config.steeringRateLimitPerSec *
        deltaTimeSec;

    float delta =
        target - current;

    delta =
        std::clamp(
            delta,
            -maxDelta,
            maxDelta);

    return current + delta;
}

//====================================================
// Unsafe direction change
//====================================================

bool MotionPlanner::isUnsafeDirectionChange(
    float targetSpeed,
    float currentSpeed) const
{
    return (
        (targetSpeed > 0.0f && currentSpeed < -0.1f) ||
        (targetSpeed < 0.0f && currentSpeed > 0.1f));
}

//====================================================
// Acceleration limiting
//====================================================

float MotionPlanner::applyAccelerationLimit(
    float targetSpeed,
    float currentSpeed,
    float deltaTimeSec) const
{
    const float maxDelta =
        m_config.maxAccelerationPerSec *
        deltaTimeSec;

    float delta =
        targetSpeed -
        currentSpeed;

    delta =
        std::clamp(
            delta,
            -maxDelta,
            maxDelta);

    return currentSpeed + delta;
}

//====================================================
// Deceleration limiting
//====================================================

float MotionPlanner::applyDecelerationLimit(
    float targetSpeed,
    float currentSpeed,
    float deltaTimeSec) const
{
    const float maxDelta =
        m_config.maxDecelerationPerSec *
        deltaTimeSec;

    float delta =
        targetSpeed -
        currentSpeed;

    delta =
        std::clamp(
            delta,
            -maxDelta,
            maxDelta);

    return currentSpeed + delta;
}

//====================================================
// Wheel saturation
//====================================================

void MotionPlanner::applyWheelSaturation(
    float &leftWheelSpeed,
    float &rightWheelSpeed) const
{
    leftWheelSpeed =
        std::clamp(
            leftWheelSpeed,
            -m_config.maxReverseWheelSpeed,
            m_config.maxWheelSpeed);

    rightWheelSpeed =
        std::clamp(
            rightWheelSpeed,
            -m_config.maxReverseWheelSpeed,
            m_config.maxWheelSpeed);
}

//====================================================
// Wheel deadzone
//====================================================

float MotionPlanner::applyWheelDeadzone(
    float speed) const
{
    if (
        fabs(speed) <
        m_config.wheelDeadzone)
    {
        return 0.0f;
    }

    return speed;
}

//====================================================
// Predictive braking
//====================================================

void MotionPlanner::applyPredictiveBraking(
    MotionCommand &motionCommand,
    const NavigationDecision &navigationDecision) const
{
    if (
        navigationDecision.emergencyOverride)
    {
        motionCommand.brakingActive = true;

        motionCommand.targetDeceleration =
            m_config.emergencyBrakeDecelerationPerSec;
    }
}

//====================================================
// Emergency braking
//====================================================

void MotionPlanner::applyEmergencyBraking(
    MotionCommand &motionCommand)
{
    motionCommand.leftWheelSpeed = 0.0f;

    motionCommand.rightWheelSpeed = 0.0f;

    motionCommand.brakingActive = true;

    motionCommand.emergencyBrakingActive = true;

    motionCommand.targetDeceleration =
        m_config.emergencyBrakeDecelerationPerSec;
}

//====================================================
// Motion persistence
//====================================================

bool MotionPlanner::shouldPersistMotion(
    MotionState state,
    uint32_t currentTimestampMs) const
{
    if (
        !m_config.enableMotionPersistence)
    {
        return false;
    }

    switch (state)
    {
    case MotionState::Turning:
    case MotionState::Escaping:
    case MotionState::EmergencyBraking:

        return true;

    default:

        return false;
    }
}

//====================================================
// Motion memory update
//====================================================

void MotionPlanner::updateMotionMemory(
    const MotionCommand &motionCommand,
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // State transition
    //-----------------------------------------

    if (
        motionCommand.state !=
        m_memory.currentState)
    {
        m_memory.previousState =
            m_memory.currentState;

        m_memory.currentState =
            motionCommand.state;

        m_memory.lastStateChangeTimestampMs =
            currentTimestampMs;

        m_memory.stateEntryTimestampMs =
            currentTimestampMs;

        m_memory.cooldownUntilTimestampMs =
            currentTimestampMs +
            m_config.motionCooldownMs;

        m_memory.stableFrameCount = 0;
    }
    else
    {
        if (
            m_memory.stableFrameCount <
            UINT32_MAX)
        {
            m_memory.stableFrameCount++;
        }
    }

    //-----------------------------------------
    // Stable state
    //-----------------------------------------

    if (
        m_memory.stableFrameCount >=
        m_config.minimumStableFrames)
    {
        m_memory.stableState =
            motionCommand.state;
    }

    //-----------------------------------------
    // Wheel history
    //-----------------------------------------

    m_memory.previousLeftWheelSpeed =
        m_memory.currentLeftWheelSpeed;

    m_memory.previousRightWheelSpeed =
        m_memory.currentRightWheelSpeed;

    m_memory.currentLeftWheelSpeed =
        motionCommand.leftWheelSpeed;

    m_memory.currentRightWheelSpeed =
        motionCommand.rightWheelSpeed;

    //-----------------------------------------
    // Speed tracking
    //-----------------------------------------

    m_memory.currentLinearSpeed =
        (motionCommand.leftWheelSpeed +
         motionCommand.rightWheelSpeed) *
        0.5f;

    m_memory.currentAngularSpeedDegPerSec =
        motionCommand.targetAngularSpeedDegPerSec;

    //-----------------------------------------
    // Steering memory
    //-----------------------------------------

    m_memory.previousSteeringCurvature =
        motionCommand.steeringCurvature;

    //-----------------------------------------
    // Braking
    //-----------------------------------------

    m_memory.brakingActive =
        motionCommand.brakingActive;

    //-----------------------------------------
    // Emergency tracking
    //-----------------------------------------

    if (
        motionCommand.emergencyBrakingActive)
    {
        if (
            m_memory.consecutiveEmergencyBrakingCount <
            UINT32_MAX)
        {
            m_memory.consecutiveEmergencyBrakingCount++;
        }
    }
    else
    {
        m_memory.consecutiveEmergencyBrakingCount = 0;
    }

    //-----------------------------------------
    // Escape tracking
    //-----------------------------------------

    if (
        motionCommand.escapeManeuverActive)
    {
        if (
            m_memory.consecutiveEscapeManeuvers <
            UINT32_MAX)
        {
            m_memory.consecutiveEscapeManeuvers++;
        }
    }
    else
    {
        m_memory.consecutiveEscapeManeuvers = 0;
    }

    //-----------------------------------------
    // Persistence
    //-----------------------------------------

    m_memory.persistentMotionActive =
        motionCommand.stabilityControlActive;

    m_memory.stabilityControlActive =
        motionCommand.stabilityControlActive;

    //-----------------------------------------
    // Update timestamp
    //-----------------------------------------

    m_memory.lastUpdateTimestampMs =
        currentTimestampMs;
}