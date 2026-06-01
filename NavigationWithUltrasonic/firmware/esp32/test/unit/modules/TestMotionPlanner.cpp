#include "framework/TestFramework.hpp"

#include "motion/include/MotionPlanner.hpp"

#include <cmath>

TEST_CASE(MotionPlanner_forwardDecisionGeneratesForwardMotion)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 10.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Forward;
    decision.action = NavigationAction::MoveForward;
    decision.targetSpeedPercent = 0.7f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_EQ(
        command.state,
        MotionState::Cruising);

    EXPECT_TRUE(
        command.leftWheelSpeed > 0.0f);

    EXPECT_TRUE(
        command.rightWheelSpeed > 0.0f);
}

TEST_CASE(MotionPlanner_emergencyDecisionGeneratesBraking)
{
    MotionPlanner planner(
        MotionPlannerConfig{});

    NavigationDecision decision;
    decision.state = NavigationState::EmergencyStop;
    decision.action = NavigationAction::Stop;
    decision.emergencyOverride = true;
    decision.targetSpeedPercent = 0.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_EQ(
        command.state,
        MotionState::EmergencyBraking);

    EXPECT_TRUE(
        command.emergencyBrakingActive);

    EXPECT_TRUE(
        command.brakingActive);
}

TEST_CASE(MotionPlanner_avoidanceDecisionGeneratesTurningMotion)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.steeringRateLimitPerSec = 100.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Avoiding;
    decision.action = NavigationAction::CurveLeft;
    decision.targetSpeedPercent = 0.4f;
    decision.desiredTurnAngleDeg = -35.0f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_EQ(
        command.state,
        MotionState::Turning);

    EXPECT_TRUE(
        command.leftWheelSpeed !=
        command.rightWheelSpeed);

    EXPECT_TRUE(
        command.steeringCurvature < 0.0f);
}

TEST_CASE(MotionPlanner_escapeDecisionGeneratesReverseMotion)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 10.0f;
    config.maxDecelerationPerSec = 10.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::EscapeMode;
    decision.action = NavigationAction::MoveBackward;
    decision.targetSpeedPercent = 0.5f;
    decision.escapeBehaviorActive = true;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_EQ(
        command.state,
        MotionState::Escaping);

    EXPECT_TRUE(
        command.reverseMotionActive);

    EXPECT_TRUE(
        command.leftWheelSpeed < 0.0f);

    EXPECT_TRUE(
        command.rightWheelSpeed < 0.0f);
}

TEST_CASE(MotionPlanner_clampsNavigationConfidence)
{
    MotionPlanner planner(
        MotionPlannerConfig{});

    NavigationDecision decision;
    decision.state = NavigationState::Forward;
    decision.targetSpeedPercent = 0.5f;
    decision.navigationConfidence = 5.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_NEAR(
        command.motionConfidence,
        1.0f,
        0.001f);
}

TEST_CASE(MotionPlanner_emergencyBrakePersistsForMinimumDuration)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.minimumEmergencyBrakeDurationMs = 300;

    MotionPlanner planner(config);

    NavigationDecision emergency;
    emergency.state = NavigationState::EmergencyStop;
    emergency.emergencyOverride = true;

    planner.process(
        emergency,
        100);

    NavigationDecision forward;
    forward.state = NavigationState::Forward;
    forward.action = NavigationAction::MoveForward;
    forward.targetSpeedPercent = 0.5f;
    forward.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            forward,
            200);

    EXPECT_EQ(
        command.state,
        MotionState::EmergencyBraking);
}

TEST_CASE(MotionPlanner_avoidLeftProducesRightWheelBias)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.steeringRateLimitPerSec = 100.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Avoiding;
    decision.action = NavigationAction::CurveLeft;
    decision.turnDirection = TurnDirection::Left;
    decision.targetSpeedPercent = 0.6f;
    decision.desiredTurnAngleDeg = -35.0f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_TRUE(
        command.leftWheelSpeed >
        command.rightWheelSpeed);
}

TEST_CASE(MotionPlanner_reverseEscapeClampsToMaxReverseSpeed)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.maxReverseWheelSpeed = 0.3f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::EscapeMode;
    decision.action = NavigationAction::MoveBackward;
    decision.targetSpeedPercent = 1.0f;
    decision.escapeBehaviorActive = true;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_TRUE(
        command.leftWheelSpeed >=
        -config.maxReverseWheelSpeed);

    EXPECT_TRUE(
        command.rightWheelSpeed >=
        -config.maxReverseWheelSpeed);
}

TEST_CASE(MotionPlanner_motionCooldownPreventsRapidStateChange)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 500;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision forward;
    forward.state = NavigationState::Forward;
    forward.action = NavigationAction::MoveForward;
    forward.targetSpeedPercent = 0.5f;
    forward.navigationConfidence = 1.0f;

    planner.process(
        forward,
        600);

    NavigationDecision avoiding;
    avoiding.state = NavigationState::Avoiding;
    avoiding.action = NavigationAction::CurveLeft;
    avoiding.targetSpeedPercent = 0.4f;
    avoiding.desiredTurnAngleDeg = -35.0f;
    avoiding.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            avoiding,
            700);

    EXPECT_EQ(
        command.state,
        MotionState::Cruising);
}

TEST_CASE(MotionPlanner_minimumMotionDurationPreventsRapidStateChange)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 500;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision forward;
    forward.state = NavigationState::Forward;
    forward.action = NavigationAction::MoveForward;
    forward.targetSpeedPercent = 0.5f;
    forward.navigationConfidence = 1.0f;

    planner.process(
        forward,
        600);

    NavigationDecision avoiding;
    avoiding.state = NavigationState::Avoiding;
    avoiding.action = NavigationAction::CurveLeft;
    avoiding.targetSpeedPercent = 0.4f;
    avoiding.desiredTurnAngleDeg = -35.0f;
    avoiding.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            avoiding,
            700);

    EXPECT_EQ(
        command.state,
        MotionState::Cruising);
}

TEST_CASE(MotionPlanner_escapeTimeoutFallsBackToTurning)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.escapeManeuverDurationMs = 100;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision escape;
    escape.state = NavigationState::EscapeMode;
    escape.action = NavigationAction::MoveBackward;
    escape.targetSpeedPercent = 0.5f;
    escape.escapeBehaviorActive = true;
    escape.navigationConfidence = 1.0f;

    planner.process(
        escape,
        100);

    const MotionCommand command =
        planner.process(
            escape,
            250);

    EXPECT_EQ(
        command.state,
        MotionState::Turning);
}

TEST_CASE(MotionPlanner_unsafeDirectionChangesZeroEachWheel)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.enableMotionSmoothing = false;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision forward;
    forward.state = NavigationState::Forward;
    forward.action = NavigationAction::MoveForward;
    forward.targetSpeedPercent = 0.5f;
    forward.navigationConfidence = 1.0f;

    planner.process(
        forward,
        100);

    NavigationDecision reverse;
    reverse.state = NavigationState::EscapeMode;
    reverse.action = NavigationAction::MoveBackward;
    reverse.targetSpeedPercent = 0.5f;
    reverse.escapeBehaviorActive = true;
    reverse.navigationConfidence = 1.0f;

    const MotionCommand reverseCommand =
        planner.process(
            reverse,
            200);

    EXPECT_NEAR(
        reverseCommand.leftWheelSpeed,
        0.0f,
        0.001f);

    EXPECT_NEAR(
        reverseCommand.rightWheelSpeed,
        0.0f,
        0.001f);

    planner.process(
        reverse,
        400);

    const MotionCommand forwardCommand =
        planner.process(
            forward,
            500);

    EXPECT_NEAR(
        forwardCommand.leftWheelSpeed,
        0.0f,
        0.001f);

    EXPECT_NEAR(
        forwardCommand.rightWheelSpeed,
        0.0f,
        0.001f);
}

TEST_CASE(MotionPlanner_nanAndInfinityWheelTargetsAreSanitized)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = INFINITY;
    config.maxDecelerationPerSec = INFINITY;
    config.enableMotionSmoothing = false;
    config.wheelDeadzone = 0.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Forward;
    decision.action = NavigationAction::MoveForward;
    decision.targetSpeedPercent = INFINITY;
    decision.desiredTurnAngleDeg = 0.0f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_NEAR(
        command.leftWheelSpeed,
        0.0f,
        0.001f);

    EXPECT_NEAR(
        command.rightWheelSpeed,
        0.0f,
        0.001f);
}

TEST_CASE(MotionPlanner_blockedDecisionProducesBrakingState)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Blocked;
    decision.action = NavigationAction::Stop;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_EQ(
        command.state,
        MotionState::Braking);
}

TEST_CASE(MotionPlanner_pivotTurnUsesOppositeWheelDirections)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.steeringRateLimitPerSec = 100.0f;
    config.enableMotionSmoothing = false;
    config.wheelDeadzone = 0.0f;
    config.pivotTurnThresholdDeg = 45.0f;
    config.pivotTurnSpeed = 0.4f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Avoiding;
    decision.action = NavigationAction::CurveRight;
    decision.targetSpeedPercent = 0.2f;
    decision.desiredTurnAngleDeg = 90.0f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_EQ(
        command.state,
        MotionState::Turning);

    EXPECT_NEAR(
        command.leftWheelSpeed,
        -0.4f,
        0.001f);

    EXPECT_NEAR(
        command.rightWheelSpeed,
        0.4f,
        0.001f);
}

TEST_CASE(MotionPlanner_normalizesDifferentialWheelTargets)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.maxAccelerationPerSec = 100.0f;
    config.maxDecelerationPerSec = 100.0f;
    config.steeringRateLimitPerSec = 100.0f;
    config.enableMotionSmoothing = false;
    config.wheelDeadzone = 0.0f;
    config.pivotTurnThresholdDeg = 180.0f;
    config.wheelNormalizationLimit = 1.0f;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Avoiding;
    decision.action = NavigationAction::CurveRight;
    decision.targetSpeedPercent = 1.0f;
    decision.desiredTurnAngleDeg = 90.0f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_NEAR(
        command.leftWheelSpeed,
        0.0f,
        0.001f);

    EXPECT_NEAR(
        command.rightWheelSpeed,
        1.0f,
        0.001f);
}

TEST_CASE(MotionPlanner_persistenceCanBeDisabled)
{
    MotionPlannerConfig config;
    config.minimumMotionDurationMs = 0;
    config.motionCooldownMs = 0;
    config.enableMotionPersistence = false;

    MotionPlanner planner(config);

    NavigationDecision decision;
    decision.state = NavigationState::Avoiding;
    decision.action = NavigationAction::CurveLeft;
    decision.desiredTurnAngleDeg = -30.0f;
    decision.navigationConfidence = 1.0f;

    const MotionCommand command =
        planner.process(
            decision,
            100);

    EXPECT_FALSE(
        command.stabilityControlActive);
}
