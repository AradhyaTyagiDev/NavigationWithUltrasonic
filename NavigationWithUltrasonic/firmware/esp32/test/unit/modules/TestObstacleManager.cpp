#include "framework/TestFramework.hpp"

#include "obstacle/include/ObstacleManager.hpp"

namespace
{
    FilteredSensorData makeSensorData(
        float distanceCm)
    {
        FilteredSensorData data;
        data.filteredDistanceCm = distanceCm;
        data.rawDistanceCm = distanceCm;
        data.confidence = 1.0f;
        data.isStable = true;
        data.obstacleDetected = true;
        data.timestampMs = 100;

        return data;
    }
}

TEST_CASE(ObstacleManager_classifiesEmergencyDistance)
{
    ObstacleManagerConfig config;
    config.enableStabilityValidation = true;
    config.enableConfidenceGating = true;

    ObstacleManager manager(config);

    const ObstacleAnalysis analysis =
        manager.process(
            makeSensorData(10.0f),
            100);

    EXPECT_TRUE(
        analysis.obstacleDetected);

    EXPECT_TRUE(
        analysis.emergencyDetected);

    EXPECT_EQ(
        analysis.dangerLevel,
        DangerLevel::Emergency);
}

TEST_CASE(ObstacleManager_unhealthySensorBecomesUnknown)
{
    ObstacleManager manager(
        ObstacleManagerConfig{});

    FilteredSensorData data =
        makeSensorData(10.0f);

    data.timeoutOccurred = true;
    data.consecutiveTimeouts = 20;

    const ObstacleAnalysis analysis =
        manager.process(
            data,
            100);

    EXPECT_EQ(
        analysis.dangerLevel,
        DangerLevel::Unknown);

    EXPECT_FALSE(
        analysis.obstacleDetected);
}

TEST_CASE(ObstacleManager_classifiesCautionAvoidAndSafe)
{
    ObstacleManagerConfig config;
    config.enableStabilityValidation = true;
    config.enableConfidenceGating = true;

    ObstacleManager manager(config);

    EXPECT_EQ(
        manager.process(
                   makeSensorData(150.0f),
                   100)
            .dangerLevel,
        DangerLevel::Safe);

    EXPECT_EQ(
        manager.process(
                   makeSensorData(80.0f),
                   120)
            .dangerLevel,
        DangerLevel::Caution);

    EXPECT_EQ(
        manager.process(
                   makeSensorData(40.0f),
                   140)
            .dangerLevel,
        DangerLevel::Avoid);
}

TEST_CASE(ObstacleManager_confidenceGatingRejectsWeakObstacle)
{
    ObstacleManagerConfig config;
    config.minimumConfidence = 0.8f;
    config.enableConfidenceGating = true;

    ObstacleManager manager(config);

    FilteredSensorData data =
        makeSensorData(10.0f);

    data.confidence = 0.2f;

    const ObstacleAnalysis analysis =
        manager.process(
            data,
            100);

    EXPECT_FALSE(
        analysis.obstacleDetected);

    EXPECT_EQ(
        analysis.dangerLevel,
        DangerLevel::Safe);
}

TEST_CASE(ObstacleManager_stabilityValidationRejectsUnstableObstacle)
{
    ObstacleManagerConfig config;
    config.enableStabilityValidation = true;

    ObstacleManager manager(config);

    FilteredSensorData data =
        makeSensorData(10.0f);

    data.isStable = false;

    const ObstacleAnalysis analysis =
        manager.process(
            data,
            100);

    EXPECT_FALSE(
        analysis.obstacleDetected);

    EXPECT_EQ(
        analysis.dangerLevel,
        DangerLevel::Safe);
}

TEST_CASE(ObstacleManager_hysteresisKeepsAvoidUntilExitMargin)
{
    ObstacleManagerConfig config;
    config.avoidDistanceCm = 50.0f;
    config.hysteresisCm = 5.0f;

    ObstacleManager manager(config);

    manager.process(
        makeSensorData(45.0f),
        100);

    const ObstacleAnalysis stillAvoid =
        manager.process(
            makeSensorData(54.0f),
            120);

    EXPECT_EQ(
        stillAvoid.dangerLevel,
        DangerLevel::Avoid);
}

TEST_CASE(ObstacleManager_fastApproachTriggersEmergencyOverride)
{
    ObstacleManagerConfig config;
    config.enableEmergencyOverride = true;
    config.emergencyVelocityThresholdCmPerSec = -150.0f;

    ObstacleManager manager(config);

    FilteredSensorData data =
        makeSensorData(80.0f);

    data.velocityCmPerSec = -200.0f;

    const ObstacleAnalysis analysis =
        manager.process(
            data,
            100);

    EXPECT_EQ(
        analysis.dangerLevel,
        DangerLevel::Emergency);

    EXPECT_TRUE(
        analysis.emergencyDetected);
}

TEST_CASE(ObstacleManager_remembersRecentlyLostObstacle)
{
    ObstacleManagerConfig config;
    config.obstacleMemoryMs = 500;
    config.maximumLostFrames = 3;
    config.enableObstacleMemory = true;

    ObstacleManager manager(config);

    manager.process(
        makeSensorData(40.0f),
        100);

    FilteredSensorData lostData =
        makeSensorData(150.0f);

    lostData.obstacleDetected = false;

    const ObstacleAnalysis analysis =
        manager.process(
            lostData,
            200);

    EXPECT_TRUE(
        analysis.obstacleRemembered);
}
