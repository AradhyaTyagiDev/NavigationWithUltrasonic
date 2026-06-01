#include "framework/TestFramework.hpp"

#include "filter/include/UltrasonicFilter.hpp"

TEST_CASE(UltrasonicFilter_validPulseProducesDistance)
{
    UltrasonicFilterConfig config;
    config.requiredStableFrames = 1;
    config.obstacleThresholdCm = 30.0f;

    UltrasonicFilter filter(config);

    const FilteredSensorData data =
        filter.process(
            5800,
            20);

    EXPECT_NEAR(
        data.rawDistanceCm,
        100.0f,
        0.1f);

    EXPECT_FALSE(
        data.timeoutOccurred);

    EXPECT_TRUE(
        data.confidence > 0.0f);
}

TEST_CASE(UltrasonicFilter_invalidPulseMarksTimeout)
{
    UltrasonicFilter filter(
        UltrasonicFilterConfig{});

    const FilteredSensorData data =
        filter.process(
            0,
            20);

    EXPECT_TRUE(
        data.timeoutOccurred);

    EXPECT_EQ(
        data.consecutiveTimeouts,
        1U);
}

TEST_CASE(UltrasonicFilter_repeatedInvalidPulsesIncrementTimeoutCounter)
{
    UltrasonicFilter filter(
        UltrasonicFilterConfig{});

    filter.process(
        0,
        20);

    const FilteredSensorData data =
        filter.process(
            0,
            40);

    EXPECT_TRUE(
        data.timeoutOccurred);

    EXPECT_EQ(
        data.consecutiveTimeouts,
        2U);
}

TEST_CASE(UltrasonicFilter_validPulseAfterTimeoutClearsTimeoutCounter)
{
    UltrasonicFilterConfig config;
    config.largeJumpThresholdCm = 1000.0f;
    config.maxRealisticVelocityCmPerSec = 10000.0f;

    UltrasonicFilter filter(config);

    filter.process(
        0,
        20);

    const FilteredSensorData data =
        filter.process(
            5800,
            40);

    EXPECT_FALSE(
        data.timeoutOccurred);

    EXPECT_EQ(
        data.consecutiveTimeouts,
        0U);
}

TEST_CASE(UltrasonicFilter_repeatedNearReadingsBecomeStableObstacle)
{
    UltrasonicFilterConfig config;
    config.requiredStableFrames = 2;
    config.obstacleThresholdCm = 30.0f;
    config.largeJumpThresholdCm = 1000.0f;
    config.maxRealisticVelocityCmPerSec = 10000.0f;
    config.deadZoneCm = 0.0f;
    config.confidenceDecayAlpha = 1.0f;

    UltrasonicFilter filter(config);

    filter.process(
        580,
        20);

    filter.process(
        580,
        40);

    const FilteredSensorData data =
        filter.process(
            580,
            60);

    EXPECT_TRUE(
        data.isStable);

    EXPECT_TRUE(
        data.obstacleDetected);
}

TEST_CASE(UltrasonicFilter_unrealisticVelocityReducesConfidenceAndVelocity)
{
    UltrasonicFilterConfig config;
    config.largeJumpThresholdCm = 1000.0f;
    config.deadZoneCm = 0.0f;
    config.maxRealisticVelocityCmPerSec = 10.0f;
    config.confidenceDecayAlpha = 1.0f;

    UltrasonicFilter filter(config);

    filter.process(
        5800,
        20);

    const FilteredSensorData data =
        filter.process(
            11600,
            21);

    EXPECT_NEAR(
        data.velocityCmPerSec,
        0.0f,
        0.001f);

    EXPECT_TRUE(
        data.confidence < 1.0f);
}

TEST_CASE(UltrasonicFilter_outlierIsRejectedFromFilteredOutput)
{
    UltrasonicFilterConfig config;
    config.largeJumpThresholdCm = 50.0f;
    config.deadZoneCm = 0.0f;

    UltrasonicFilter filter(config);

    filter.process(
        23200,
        20);

    const FilteredSensorData data =
        filter.process(
            5800,
            40);

    EXPECT_NEAR(
        data.rawDistanceCm,
        100.0f,
        0.1f);

    EXPECT_TRUE(
        data.filteredDistanceCm > 300.0f);
}

TEST_CASE(UltrasonicFilter_confidenceDecaysAfterInvalidMeasurement)
{
    UltrasonicFilterConfig config;
    config.confidenceDecayAlpha = 0.5f;
    config.largeJumpThresholdCm = 1000.0f;

    UltrasonicFilter filter(config);

    const FilteredSensorData valid =
        filter.process(
            5800,
            20);

    const FilteredSensorData invalid =
        filter.process(
            0,
            40);

    EXPECT_TRUE(
        invalid.confidence <
        valid.confidence);

    EXPECT_TRUE(
        invalid.confidence > 0.0f);
}

TEST_CASE(UltrasonicFilter_velocityIsCalculatedForValidMovement)
{
    UltrasonicFilterConfig config;
    config.largeJumpThresholdCm = 1000.0f;
    config.maxRealisticVelocityCmPerSec = 10000.0f;
    config.deadZoneCm = 0.0f;

    UltrasonicFilter filter(config);

    filter.process(
        5800,
        100);

    const FilteredSensorData data =
        filter.process(
            11600,
            200);

    EXPECT_TRUE(
        data.velocityCmPerSec > 0.0f);
}
