//====================================================
// File: MotionPlanner.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "navigation/include/NavigationDecision.hpp"

#include "MotionCommand.hpp"
#include "MotionMemory.hpp"
#include "MotionPlannerConfig.hpp"

//====================================================
// MotionPlanner
//====================================================
//
// Physical locomotion strategy engine
// that converts navigation intent
// into executable motion targets.
//
//----------------------------------------------------
// INPUT
//----------------------------------------------------
//
//      NavigationDecision
//
//----------------------------------------------------
// OUTPUT
//----------------------------------------------------
//
//      MotionCommand
//
//----------------------------------------------------
// RESPONSIBILITIES
//----------------------------------------------------
//
// 1. Motion strategy generation
//      - smooth curve
//      - pivot turn
//      - reverse escape
//      - emergency braking
//
// 2. Steering generation
//      - steering curvature
//      - differential drive planning
//
// 3. Speed profiling
//      - acceleration ramps
//      - deceleration ramps
//      - jerk-safe transitions
//
// 4. Motion smoothing
//      - anti-jitter
//      - anti-twitch
//      - wheel stabilization
//
// 5. Differential drive planning
//      - left wheel target
//      - right wheel target
//
// 6. Motion safety constraints
//      - wheel saturation
//      - acceleration limits
//      - steering limits
//
// 7. Emergency motion handling
//      - emergency braking
//      - stabilization
//
// 8. Motion persistence
//      - stable locomotion transitions
//
// 9. NavigationDecision → MotionCommand
//
//----------------------------------------------------
// THIS LAYER DOES NOT
//----------------------------------------------------
//
//      - Control GPIO
//      - Generate PWM
//      - Drive motors directly
//      - Interpret sensors
//      - Perform path planning
//
//====================================================

class MotionPlanner
{
public:
    // Constructor
    explicit MotionPlanner(
        const MotionPlannerConfig &config);

    //================================================
    // Main locomotion planning pipeline
    // Converts:
    //      NavigationDecision
    // Into:
    //      MotionCommand
    MotionCommand process(
        const NavigationDecision &navigationDecision,
        uint32_t currentTimestampMs);

private:
    //================================================
    // Motion state determination
    // Converts navigation behavior
    // into locomotion state.
    MotionState determineMotionState(
        const NavigationDecision &navigationDecision) const;

    //================================================
    // Motion transition validation
    // Prevents:
    //      - unstable locomotion
    //      - rapid switching
    //      - wheel twitching

    bool canTransitionToState(
        MotionState newState,
        uint32_t currentTimestampMs) const;

    //================================================
    // Emergency braking detection
    // Determines whether immediate
    // emergency braking is required.
    bool shouldTriggerEmergencyBraking(
        const NavigationDecision &navigationDecision) const;

    //================================================
    // Linear speed planning
    // Converts:
    //      targetSpeedPercent
    //
    // Into:
    //      stable locomotion speed
    float calculateTargetLinearSpeed(
        const NavigationDecision &navigationDecision) const;

    //================================================
    // Angular speed planning
    // Converts:
    //      turn angle
    //
    // Into:
    //      angular turning velocity
    float calculateTargetAngularSpeed(
        const NavigationDecision &navigationDecision) const;

    //================================================
    // Steering curvature planning
    // Converts:
    //      steering intent
    // Into:
    //      locomotion curvature
    float calculateSteeringCurvature(
        const NavigationDecision &navigationDecision) const;

    //================================================
    // Differential drive generation
    // Converts:
    //      linear speed
    //      angular speed
    //      steering curvature
    // Into:
    //      left/right wheel targets
    void generateDifferentialDriveTargets(
        float linearSpeed,
        float angularSpeed,
        float steeringCurvature,
        float &leftWheelTarget,
        float &rightWheelTarget) const;

    //================================================
    // Wheel speed smoothing
    // Prevents:
    //      - wheel twitching
    //      - violent transitions
    //      - unstable locomotion
    float applyWheelSpeedSmoothing(
        float targetSpeed,
        float currentSpeed) const;

    //================================================
    // Steering smoothing
    // Prevents:
    //      - steering jitter
    //      - unstable curvature
    float applySteeringSmoothing(
        float targetCurvature,
        float currentCurvature) const;

    //================================================
    // Acceleration limiting
    // Prevents:
    //      - violent acceleration
    //      - motor stress
    float applyAccelerationLimit(
        float targetSpeed,
        float currentSpeed,
        float deltaTimeSec) const;

    //================================================
    // Deceleration limiting
    // Prevents:
    //      - unstable braking
    //      - wheel locking
    float applyDecelerationLimit(
        float targetSpeed,
        float currentSpeed,
        float deltaTimeSec) const;

    //================================================
    // Wheel saturation protection
    // Ensures:
    //      wheel speeds remain safe
    void applyWheelSaturation(
        float &leftWheelSpeed,
        float &rightWheelSpeed) const;

    //================================================
    // Predictive braking
    // Applies:
    //      inertia-aware braking
    //      smooth stopping behavior
    void applyPredictiveBraking(
        MotionCommand &motionCommand,
        const NavigationDecision &navigationDecision) const;

    //================================================
    // Emergency braking handling
    // Generates:
    //      immediate stabilization
    void applyEmergencyBraking(
        MotionCommand &motionCommand);

    //================================================
    // Motion persistence evaluation
    // Prevents:
    //      unstable locomotion transitions
    bool shouldPersistMotion(
        MotionState state,
        uint32_t currentTimestampMs) const;

    //================================================
    // Motion memory update
    // Updates:
    //      - wheel history
    //      - motion transitions
    //      - steering persistence
    //      - braking state
    void updateMotionMemory(
        const MotionCommand &motionCommand,
        uint32_t currentTimestampMs);

    // Emergency brake persistence
    bool isEmergencyBrakeLocked(
        uint32_t currentTimestampMs) const;

    // Steering rate limiting
    float applySteeringRateLimit(
        float target,
        float current,
        float deltaTimeSec) const;

    // Unsafe direction protection
    bool isUnsafeDirectionChange(
        float targetSpeed,
        float currentSpeed) const;

    // Wheel deadzone
    float applyWheelDeadzone(
        float speed) const;

private:
    //================================================
    // Configuration
    //================================================
    MotionPlannerConfig m_config;

    //================================================
    // Persistent locomotion memory
    //================================================
    MotionMemory m_memory;
};