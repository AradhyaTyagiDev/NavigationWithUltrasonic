#include "framework/TestFramework.hpp"
#include "integration/IntegrationHarness.hpp"

TEST_CASE(Integration_ClearPathProducesForwardMotorCommands)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Active);

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.motorDriver.dualCommandCount > 0U);

    EXPECT_EQ(
        harness.motorDriver.lastLeftCommand.direction,
        MotorDirection::Forward);

    EXPECT_EQ(
        harness.motorDriver.lastRightCommand.direction,
        MotorDirection::Forward);
}

TEST_CASE(Integration_ApproachingWallTransitionsToEmergency)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    EXPECT_TRUE(
        harness.start());

    const float distances[] = {
        180.0f,
        120.0f,
        80.0f,
        45.0f,
        15.0f};

    harness.runDistanceSequence(
        distances,
        sizeof(distances) / sizeof(distances[0]));

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

TEST_CASE(Integration_AvoidDistanceProducesDifferentialTurnCommand)
{
    IntegrationConfig config =
        productionLikeIntegrationConfig();

    config.obstacle.enableEmergencyOverride = false;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    for (uint32_t index = 0; index < 4; ++index)
    {
        harness.stepMs(
            20,
            true,
            45.0f);
    }

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.motorDriver.dualCommandCount > 0U);

    EXPECT_TRUE(
        harness.motorDriver.lastLeftCommand.normalizedSpeed !=
        harness.motorDriver.lastRightCommand.normalizedSpeed);
}

TEST_CASE(Integration_SensorDropoutSetsUnhealthyWithoutEmergency)
{
    IntegrationConfig config =
        productionLikeIntegrationConfig();

    config.robot.sensorTimeoutMs = 100;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        20,
        true,
        180.0f);

    harness.stepMs(
        200,
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
}

TEST_CASE(Integration_MotorFaultPropagatesToRobotFault)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    EXPECT_TRUE(
        harness.start());

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

    EXPECT_TRUE(
        harness.motorController.hasFault());
}

TEST_CASE(Integration_MotorBusyRecordsMissedCycleAndRecovers)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    EXPECT_TRUE(
        harness.start());

    harness.motorMutex.setForceBusy(true);

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .missedControlCycles,
        1ULL);

    harness.motorMutex.setForceBusy(false);

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .motorExecutions > 0ULL);
}

TEST_CASE(Integration_LongClearPathSoakKeepsSystemHealthy)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    EXPECT_TRUE(
        harness.start());

    for (uint32_t index = 0; index < 100; ++index)
    {
        harness.stepMs(
            20,
            true,
            180.0f);
    }

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .systemHealthy);

    EXPECT_FALSE(
        harness.robotController.hasFault());

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .successfulControlLoops,
        100ULL);
}

TEST_CASE(Integration_ObstacleFlickerAroundAvoidThresholdDoesNotFault)
{
    IntegrationConfig config =
        productionLikeIntegrationConfig();

    config.obstacle.enableEmergencyOverride = false;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    const float distances[] = {
        55.0f,
        48.0f,
        53.0f,
        47.0f,
        60.0f,
        45.0f,
        70.0f};

    harness.runDistanceSequence(
        distances,
        sizeof(distances) / sizeof(distances[0]));

    EXPECT_FALSE(
        harness.robotController.hasFault());

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .navigationDecisions > 0ULL);
}

TEST_CASE(Integration_EmergencyCanBeClearedAndRobotCanResume)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    EXPECT_TRUE(
        harness.start());

    for (uint32_t index = 0; index < 4; ++index)
    {
        harness.stepMs(
            20,
            true,
            10.0f);
    }

    EXPECT_TRUE(
        harness.robotController.isEmergencyActive());

    harness.robotController.clearEmergency();

    EXPECT_FALSE(
        harness.robotController.isEmergencyActive());

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_FALSE(
        harness.robotController.hasFault());
}

TEST_CASE(Integration_SensorDropoutRecoveryRestoresSensorHealth)
{
    IntegrationConfig config =
        productionLikeIntegrationConfig();

    config.robot.sensorTimeoutMs = 100;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        200,
        false,
        180.0f);

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);
}

TEST_CASE(Integration_AutomaticFaultRecoveryReturnsToPaused)
{
    IntegrationConfig config =
        productionLikeIntegrationConfig();

    config.robot.enableAutomaticFaultRecovery = true;
    config.robot.faultRecoveryCooldownMs = 100;

    IntegrationHarness harness(config);

    EXPECT_TRUE(
        harness.start());

    harness.motorDriver.fault = true;

    harness.stepMs(
        20,
        true,
        180.0f);

    EXPECT_TRUE(
        harness.robotController.hasFault());

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

TEST_CASE(Integration_ControllerTimingViolationMarksTimingUnhealthy)
{
    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    harness.motorDriver.timerToAdvance =
        &harness.timer;

    harness.motorDriver.advanceOnDualCommandUs =
        20000;

    EXPECT_TRUE(
        harness.start());

    harness.stepMs(
        60,
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
