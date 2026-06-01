#include "framework/TestFramework.hpp"

#include "navigation/include/NavigationManager.hpp"

namespace
{
    ObstacleAnalysis makeAnalysis(
        DangerLevel dangerLevel)
    {
        ObstacleAnalysis analysis;
        analysis.dangerLevel = dangerLevel;
        analysis.confidence = 1.0f;
        analysis.sensorHealthy = true;
        analysis.isStable = true;
        analysis.obstacleDetected =
            dangerLevel != DangerLevel::Safe;
        analysis.emergencyDetected =
            dangerLevel == DangerLevel::Emergency;

        return analysis;
    }
}

TEST_CASE(NavigationManager_safeEnvironmentMovesForward)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Safe),
            100);

    EXPECT_EQ(
        decision.state,
        NavigationState::Forward);

    EXPECT_EQ(
        decision.action,
        NavigationAction::MoveForward);
}

TEST_CASE(NavigationManager_emergencyStopsImmediately)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Emergency),
            100);

    EXPECT_EQ(
        decision.state,
        NavigationState::EmergencyStop);

    EXPECT_EQ(
        decision.action,
        NavigationAction::Stop);

    EXPECT_TRUE(
        decision.emergencyOverride);
}

TEST_CASE(NavigationManager_cautionUsesCautiousForwardProfile)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Caution),
            100);

    EXPECT_EQ(
        decision.state,
        NavigationState::CautiousForward);

    EXPECT_EQ(
        decision.action,
        NavigationAction::MoveForward);

    EXPECT_TRUE(
        decision.targetSpeedPercent <
        config.normalSpeedPercent);
}

TEST_CASE(NavigationManager_avoidChoosesCurveAndAvoidanceFlag)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.avoidanceCooldownMs = 0;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Avoid),
            100);

    EXPECT_EQ(
        decision.state,
        NavigationState::Avoiding);

    EXPECT_TRUE(
        decision.action == NavigationAction::CurveLeft ||
        decision.action == NavigationAction::CurveRight);

    EXPECT_TRUE(
        decision.obstacleAvoidanceActive);

    EXPECT_TRUE(
        decision.desiredTurnAngleDeg != 0.0f);
}

TEST_CASE(NavigationManager_unknownEnvironmentMovesCautiously)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Unknown),
            100);

    EXPECT_EQ(
        decision.state,
        NavigationState::CautiousForward);

    EXPECT_EQ(
        decision.action,
        NavigationAction::MoveForward);
}

TEST_CASE(NavigationManager_blockedEnvironmentEntersEscapeMode)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.enableRandomEscapeDirection = false;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Blocked),
            100);

    EXPECT_EQ(
        decision.state,
        NavigationState::EscapeMode);

    EXPECT_EQ(
        decision.action,
        NavigationAction::MoveBackward);

    EXPECT_TRUE(
        decision.escapeBehaviorActive);
}

TEST_CASE(NavigationManager_lowConfidenceIsAdaptedDown)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.lowConfidenceThreshold = 0.5f;
    config.enableConfidenceAdaptation = true;

    NavigationManager manager(config);

    ObstacleAnalysis analysis =
        makeAnalysis(DangerLevel::Safe);

    analysis.confidence = 0.4f;

    const NavigationDecision decision =
        manager.process(
            analysis,
            100);

    EXPECT_NEAR(
        decision.navigationConfidence,
        0.28f,
        0.001f);
}

TEST_CASE(NavigationManager_avoidanceDirectionAlternates)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.avoidanceCooldownMs = 0;

    NavigationManager manager(config);

    const NavigationDecision first =
        manager.process(
            makeAnalysis(DangerLevel::Avoid),
            100);

    const NavigationDecision second =
        manager.process(
            makeAnalysis(DangerLevel::Avoid),
            120);

    EXPECT_TRUE(
        first.turnDirection != TurnDirection::None);

    EXPECT_TRUE(
        second.turnDirection != TurnDirection::None);

    EXPECT_TRUE(
        first.turnDirection != second.turnDirection);
}

TEST_CASE(NavigationManager_blockedStateEscalatesAfterRepeatedEscapes)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.blockedStateThreshold = 2;
    config.enableRandomEscapeDirection = false;

    NavigationManager manager(config);

    manager.process(
        makeAnalysis(DangerLevel::Blocked),
        100);

    manager.process(
        makeAnalysis(DangerLevel::Blocked),
        120);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Blocked),
            140);

    EXPECT_EQ(
        decision.state,
        NavigationState::Blocked);

    EXPECT_EQ(
        decision.action,
        NavigationAction::Stop);
}

TEST_CASE(NavigationManager_avoidanceCooldownPreventsRapidExit)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.avoidanceCooldownMs = 500;

    NavigationManager manager(config);

    manager.process(
        makeAnalysis(DangerLevel::Avoid),
        100);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Safe),
            200);

    EXPECT_EQ(
        decision.state,
        NavigationState::Avoiding);

    EXPECT_TRUE(
        decision.persistentBehavior);
}

TEST_CASE(NavigationManager_minimumStateDurationPreventsRapidChange)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 500;
    config.avoidanceCooldownMs = 0;

    NavigationManager manager(config);

    manager.process(
        makeAnalysis(DangerLevel::Safe),
        600);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Avoid),
            700);

    EXPECT_EQ(
        decision.state,
        NavigationState::Forward);
}

TEST_CASE(NavigationManager_persistenceCanBeDisabled)
{
    NavigationManagerConfig config;
    config.minimumStateDurationMs = 0;
    config.enableBehaviorPersistence = false;

    NavigationManager manager(config);

    const NavigationDecision decision =
        manager.process(
            makeAnalysis(DangerLevel::Avoid),
            100);

    EXPECT_FALSE(
        decision.persistentBehavior);
}
