#include "framework/TestFramework.hpp"
#include "IntegrationHarness.hpp"

#include <cstddef>

namespace
{
    void runTrace(
        IntegrationHarness &harness,
        const float *distancesCm,
        size_t count)
    {
        for (size_t index = 0; index < count; ++index)
        {
            harness.stepMs(
                20,
                true,
                distancesCm[index]);
        }
    }

    void expectMotorCommandsBounded(
        const IntegrationHarness &harness)
    {
        EXPECT_TRUE(
            harness.motorDriver
                .lastLeftCommand
                .normalizedSpeed >= 0.0f);

        EXPECT_TRUE(
            harness.motorDriver
                .lastLeftCommand
                .normalizedSpeed <= 1.0f);

        EXPECT_TRUE(
            harness.motorDriver
                .lastRightCommand
                .normalizedSpeed >= 0.0f);

        EXPECT_TRUE(
            harness.motorDriver
                .lastRightCommand
                .normalizedSpeed <= 1.0f);
    }

    void expectNoUnexpectedRuntimeFailure(
        const IntegrationHarness &harness)
    {
        EXPECT_FALSE(
            harness.robotController.hasFault());

        EXPECT_FALSE(
            harness.robotController.isEmergencyActive());

        EXPECT_TRUE(
            harness.robotController
                .getSystemHealth()
                .systemHealthy);
    }

    IntegrationConfig e2eConfig()
    {
        IntegrationConfig config =
            productionLikeIntegrationConfig();

        config.robot.sensorTimeoutMs = 100;
        config.robot.maximumPipelineDurationUs = 10000;
        config.motor.motorCommandTimeoutMs = 120;

        return config;
    }
}

TEST_CASE(E2E_LongClearPathRuntimeSoakRemainsHealthy)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    for (uint32_t cycle = 0; cycle < 500; ++cycle)
    {
        harness.stepMs(
            20,
            true,
            180.0f);
    }

    const RuntimeStatistics &statistics =
        harness.robotController
            .getMemory()
            .runtimeStatistics;

    EXPECT_EQ(
        statistics.totalControlLoops,
        500ULL);

    EXPECT_EQ(
        statistics.successfulControlLoops,
        500ULL);

    EXPECT_EQ(
        statistics.sensorUpdates,
        500ULL);

    EXPECT_EQ(
        statistics.motorExecutions,
        500ULL);

    EXPECT_EQ(
        statistics.emergencyStops,
        0ULL);

    EXPECT_EQ(
        statistics.totalFaults,
        0ULL);

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .systemHealthy);

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_EQ(
        harness.motorDriver.lastLeftCommand.direction,
        MotorDirection::Forward);

    EXPECT_EQ(
        harness.motorDriver.lastRightCommand.direction,
        MotorDirection::Forward);
}

TEST_CASE(E2E_MixedMissionTraceAvoidsObstacleAndEndsHealthy)
{
    IntegrationConfig config =
        e2eConfig();

    config.obstacle.enableEmergencyOverride = false;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    const float trace[] = {
        180.0f, 160.0f, 140.0f, 110.0f, 90.0f,
        70.0f, 55.0f, 45.0f, 48.0f, 52.0f,
        70.0f, 100.0f, 140.0f, 180.0f, 180.0f};

    runTrace(
        harness,
        trace,
        sizeof(trace) / sizeof(trace[0]));

    EXPECT_FALSE(
        harness.robotController.hasFault());

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .systemHealthy);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .navigationDecisions >=
        sizeof(trace) / sizeof(trace[0]));

    EXPECT_TRUE(
        harness.motorDriver.dualCommandCount > 0U);
}

TEST_CASE(E2E_CollisionEmergencyDominatesUntilExplicitClear)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    const float dangerTrace[] = {
        180.0f,
        120.0f,
        70.0f,
        35.0f,
        10.0f,
        10.0f};

    runTrace(
        harness,
        dangerTrace,
        sizeof(dangerTrace) / sizeof(dangerTrace[0]));

    EXPECT_TRUE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.motorController.isEmergencyStopActive());

    EXPECT_TRUE(
        harness.motorDriver.emergencyStopActive);

    const uint64_t emergencyCount =
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .emergencyStops;

    harness.stepMs(
        100,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController.isEmergencyActive());

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .emergencyStops,
        emergencyCount);

    harness.robotController.clearEmergency();

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_FALSE(
        harness.robotController.hasFault());
}

TEST_CASE(E2E_SensorDropoutThenRecoveryPreservesRuntime)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        100,
        true,
        180.0f);

    harness.stepMs(
        300,
        false,
        180.0f);

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .sensorTimeouts > 0ULL);

    harness.stepMs(
        60,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);

    EXPECT_FALSE(
        harness.robotController.hasFault());

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());
}

TEST_CASE(E2E_BurstyMotorBusyCyclesAreMissedButRecover)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    for (uint32_t burst = 0; burst < 5; ++burst)
    {
        harness.motorMutex.setForceBusy(true);

        harness.stepMs(
            20,
            true,
            180.0f);

        harness.motorMutex.setForceBusy(false);

        harness.stepMs(
            40,
            true,
            180.0f);
    }

    const RuntimeStatistics &statistics =
        harness.robotController
            .getMemory()
            .runtimeStatistics;

    EXPECT_EQ(
        statistics.missedControlCycles,
        5ULL);

    EXPECT_TRUE(
        statistics.motorExecutions > 0ULL);

    EXPECT_FALSE(
        harness.robotController.hasFault());

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());
}

TEST_CASE(E2E_MotorFaultAutomaticallyRecoversToPaused)
{
    IntegrationConfig config =
        e2eConfig();

    config.robot.enableAutomaticFaultRecovery = true;
    config.robot.faultRecoveryCooldownMs = 100;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        40,
        true,
        180.0f);

    harness.motorDriver.fault = true;

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController.hasFault());

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Fault);

    harness.motorDriver.fault = false;

    harness.stepMs(
        200,
        true,
        180.0f);

    EXPECT_FALSE(
        harness.robotController.hasFault());

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Paused);
}

TEST_CASE(E2E_TimingViolationIsRecordedDuringSlowMotorBoundary)
{
    IntegrationHarness harness(
        e2eConfig());

    harness.motorDriver.timerToAdvance =
        &harness.timer;

    harness.motorDriver.advanceOnDualCommandUs =
        20000;

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        80,
        true,
        180.0f);

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .timingHealthy);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .timingViolations > 0ULL);
}

TEST_CASE(E2E_ApproachWallFrom200To10TriggersEmergency)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    const float approachTrace[] = {
        200.0f, 180.0f, 160.0f, 140.0f, 120.0f,
        100.0f, 80.0f, 60.0f, 40.0f, 25.0f,
        15.0f, 10.0f};

    runTrace(
        harness,
        approachTrace,
        sizeof(approachTrace) / sizeof(approachTrace[0]));

    EXPECT_TRUE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.motorController.isEmergencyStopActive());

    EXPECT_TRUE(
        harness.motorDriver.emergencyStopActive);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .emergencyStops > 0ULL);
}

TEST_CASE(E2E_SuddenObstacleTriggersEmergencyImmediately)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        100,
        true,
        200.0f);

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    harness.stepMs(
        20,
        true,
        10.0f);

    EXPECT_TRUE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.motorDriver.emergencyStopActive);
}

TEST_CASE(E2E_SensorDropoutFor500msRecoversCleanly)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        100,
        true,
        180.0f);

    harness.stepMs(
        500,
        false,
        180.0f);

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .sensorTimeouts > 0ULL);

    harness.stepMs(
        100,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);

    EXPECT_FALSE(
        harness.robotController.hasFault());
}

TEST_CASE(E2E_ObstacleFlickerAroundThresholdDoesNotFaultOrEmergency)
{
    IntegrationConfig config =
        e2eConfig();

    config.obstacle.enableEmergencyOverride = false;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    const float flickerTrace[] = {
        55.0f, 49.0f, 51.0f, 48.0f, 52.0f,
        47.0f, 53.0f, 46.0f, 54.0f, 60.0f,
        45.0f, 65.0f, 50.0f, 70.0f};

    runTrace(
        harness,
        flickerTrace,
        sizeof(flickerTrace) / sizeof(flickerTrace[0]));

    expectNoUnexpectedRuntimeFailure(
        harness);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .navigationDecisions >=
        sizeof(flickerTrace) / sizeof(flickerTrace[0]));

    expectMotorCommandsBounded(
        harness);
}

TEST_CASE(E2E_TenMinuteClearPathSoakMeetsRuntimeInvariants)
{
    IntegrationHarness harness(
        e2eConfig());

    EXPECT_TRUE(
        harness.start());

    constexpr uint32_t tenMinutesAt50Hz = 30000;

    for (uint32_t cycle = 0;
         cycle < tenMinutesAt50Hz;
         ++cycle)
    {
        harness.stepMs(
            20,
            true,
            180.0f);
    }

    const RuntimeStatistics &statistics =
        harness.robotController
            .getMemory()
            .runtimeStatistics;

    EXPECT_EQ(
        statistics.totalControlLoops,
        static_cast<uint64_t>(tenMinutesAt50Hz));

    EXPECT_EQ(
        statistics.successfulControlLoops,
        static_cast<uint64_t>(tenMinutesAt50Hz));

    EXPECT_EQ(
        statistics.missedControlCycles,
        0ULL);

    EXPECT_EQ(
        statistics.totalFaults,
        0ULL);

    EXPECT_EQ(
        statistics.emergencyStops,
        0ULL);

    EXPECT_EQ(
        statistics.timingViolations,
        0ULL);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .pipelineTiming
            .averageLoopDurationUs <
        harness.robotConfig.maximumPipelineDurationUs);

    expectNoUnexpectedRuntimeFailure(
        harness);

    expectMotorCommandsBounded(
        harness);
}

TEST_CASE(E2E_AverageLoopDurationStaysUnderBudget)
{
    IntegrationHarness harness(
        e2eConfig());

    harness.motorDriver.timerToAdvance =
        &harness.timer;

    harness.motorDriver.advanceOnDualCommandUs =
        1000;

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        200,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .pipelineTiming
            .averageLoopDurationUs <
        harness.robotConfig.maximumPipelineDurationUs);

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .timingViolations,
        0ULL);
}

TEST_CASE(E2E_ReplaysSimulatedDistanceTraceAndKeepsStateSane)
{
    IntegrationConfig config =
        e2eConfig();

    config.obstacle.enableEmergencyOverride = false;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    const float simulatedTrace[] = {
        200.0f, 195.0f, 190.0f, 175.0f, 150.0f,
        125.0f, 100.0f, 80.0f, 60.0f, 52.0f,
        48.0f, 44.0f, 48.0f, 55.0f, 75.0f,
        110.0f, 150.0f, 190.0f};

    RobotState previousState =
        harness.robotController.getCurrentState();

    for (size_t index = 0;
         index < sizeof(simulatedTrace) / sizeof(simulatedTrace[0]);
         ++index)
    {
        harness.stepMs(
            20,
            true,
            simulatedTrace[index]);

        const RobotState currentState =
            harness.robotController.getCurrentState();

        EXPECT_FALSE(
            previousState == RobotState::Fault &&
            currentState == RobotState::Active);

        previousState = currentState;
    }

    expectNoUnexpectedRuntimeFailure(
        harness);

    expectMotorCommandsBounded(
        harness);
}
