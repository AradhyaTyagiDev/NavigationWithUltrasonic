#include "framework/TestFramework.hpp"
#include "fakes/Fakes.hpp"

#include "filter/include/UltrasonicFilter.hpp"
#include "motion/include/MotionPlanner.hpp"
#include "motor/controller/include/MotorController.hpp"
#include "navigation/include/NavigationManager.hpp"
#include "obstacle/include/ObstacleManager.hpp"
#include "robot/include/RobotController.hpp"

namespace
{
    struct RobotHarness
    {
        FakeUltrasonicSensor sensor;
        FakeMotorDriver driver;
        FakeMutex robotMutex;
        FakeMutex motorMutex;
        FakeLogger logger;
        FakeTimer timer;

        UltrasonicFilter filter{UltrasonicFilterConfig{}};
        ObstacleManager obstacleManager{ObstacleManagerConfig{}};
        NavigationManager navigationManager{NavigationManagerConfig{}};
        MotionPlanner motionPlanner{MotionPlannerConfig{}};
        MotorController motorController{
            driver,
            motorMutex,
            logger,
            timer,
            MotorControllerConfig{}};
        RobotControllerConfig robotConfig;
        RobotController robotController{
            sensor,
            filter,
            obstacleManager,
            navigationManager,
            motionPlanner,
            motorController,
            robotMutex,
            logger,
            timer,
            robotConfig};
    };
}

TEST_CASE(RobotController_initializesAndStartsDependencies)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    EXPECT_TRUE(
        harness.sensor.initialized);

    EXPECT_TRUE(
        harness.sensor.running);

    EXPECT_TRUE(
        harness.robotController.isRunning());
}

TEST_CASE(RobotController_sensorTimeoutMarksSensorUnhealthy)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.timer.setMs(300);

    harness.robotController.update();

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);
}

TEST_CASE(RobotController_motorBusyDoesNotBlockPipeline)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.motorMutex.setForceBusy(true);

    harness.timer.setMs(20);

    harness.robotController.update();

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .missedControlCycles,
        1ULL);
}

TEST_CASE(RobotController_startFailureActivatesFault)
{
    RobotHarness harness;
    harness.sensor.startResult = false;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_FALSE(
        harness.robotController.start());

    EXPECT_TRUE(
        harness.robotController.hasFault());
}

TEST_CASE(RobotController_freshSensorDataClearsTimeoutHealth)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.timer.setMs(300);

    harness.robotController.update();

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);

    harness.sensor.publishPulse(
        5800,
        320);

    harness.timer.setMs(320);

    harness.robotController.update();

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .sensorHealthy);
}

TEST_CASE(RobotController_normalUpdateIncrementsLoopStatistics)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.sensor.publishPulse(
        5800,
        20);

    harness.timer.setMs(20);

    harness.robotController.update();

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .totalControlLoops,
        1ULL);

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .successfulControlLoops,
        1ULL);
}

TEST_CASE(RobotController_emergencyStopStopsMotorController)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.robotController.emergencyStop();

    EXPECT_TRUE(
        harness.robotController.isEmergencyActive());

    EXPECT_TRUE(
        harness.motorController.isEmergencyStopActive());

    EXPECT_TRUE(
        harness.driver.emergencyStopActive);
}

TEST_CASE(RobotController_resetClearsRuntimeState)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.timer.setMs(300);

    harness.robotController.update();

    harness.robotController.reset();

    EXPECT_FALSE(
        harness.robotController.isRunning());

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .totalControlLoops,
        0ULL);
}

TEST_CASE(RobotController_emergencyObstacleTriggersEmergencyState)
{
    FakeUltrasonicSensor sensor;
    FakeMotorDriver driver;
    FakeMutex robotMutex;
    FakeMutex motorMutex;
    FakeLogger logger;
    FakeTimer timer;

    UltrasonicFilterConfig filterConfig;
    filterConfig.requiredStableFrames = 1;
    filterConfig.obstacleThresholdCm = 30.0f;
    filterConfig.largeJumpThresholdCm = 1000.0f;
    filterConfig.maxRealisticVelocityCmPerSec = 10000.0f;
    filterConfig.deadZoneCm = 0.0f;
    filterConfig.confidenceDecayAlpha = 1.0f;

    ObstacleManagerConfig obstacleConfig;
    obstacleConfig.emergencyDistanceCm = 120.0f;
    obstacleConfig.enableConfidenceGating = false;
    obstacleConfig.enableStabilityValidation = false;

    NavigationManagerConfig navigationConfig;
    navigationConfig.minimumStateDurationMs = 0;

    MotionPlannerConfig motionConfig;
    motionConfig.minimumMotionDurationMs = 0;
    motionConfig.motionCooldownMs = 0;

    UltrasonicFilter filter(filterConfig);
    ObstacleManager obstacleManager(obstacleConfig);
    NavigationManager navigationManager(navigationConfig);
    MotionPlanner motionPlanner(motionConfig);

    MotorController motorController{
        driver,
        motorMutex,
        logger,
        timer,
        MotorControllerConfig{}};

    RobotControllerConfig robotConfig;

    RobotController robotController{
        sensor,
        filter,
        obstacleManager,
        navigationManager,
        motionPlanner,
        motorController,
        robotMutex,
        logger,
        timer,
        robotConfig};

    EXPECT_TRUE(
        robotController.initialize());

    EXPECT_TRUE(
        robotController.start());

    sensor.publishPulse(
        580,
        20);

    timer.setMs(20);

    robotController.update();

    EXPECT_TRUE(
        robotController.isEmergencyActive());

    EXPECT_TRUE(
        motorController.isEmergencyStopActive());
}

TEST_CASE(RobotController_pipelineExecutionUpdatesAllStageStatistics)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.sensor.publishPulse(
        5800,
        20);

    harness.timer.setMs(20);

    harness.robotController.update();

    const RuntimeStatistics &statistics =
        harness.robotController
            .getMemory()
            .runtimeStatistics;

    EXPECT_EQ(
        statistics.sensorUpdates,
        1ULL);

    EXPECT_EQ(
        statistics.obstacleAnalyses,
        1ULL);

    EXPECT_EQ(
        statistics.navigationDecisions,
        1ULL);

    EXPECT_EQ(
        statistics.motionPlans,
        1ULL);

    EXPECT_EQ(
        statistics.motorExecutions,
        1ULL);
}

TEST_CASE(RobotController_automaticFaultRecoveryClearsFaultAfterCooldown)
{
    FakeUltrasonicSensor sensor;
    FakeMotorDriver driver;
    FakeMutex robotMutex;
    FakeMutex motorMutex;
    FakeLogger logger;
    FakeTimer timer;

    UltrasonicFilter filter{UltrasonicFilterConfig{}};
    ObstacleManager obstacleManager{ObstacleManagerConfig{}};
    NavigationManager navigationManager{NavigationManagerConfig{}};
    MotionPlanner motionPlanner{MotionPlannerConfig{}};
    MotorController motorController{
        driver,
        motorMutex,
        logger,
        timer,
        MotorControllerConfig{}};

    RobotControllerConfig robotConfig;
    robotConfig.enableAutomaticFaultRecovery = true;
    robotConfig.faultRecoveryCooldownMs = 100;

    RobotController robotController{
        sensor,
        filter,
        obstacleManager,
        navigationManager,
        motionPlanner,
        motorController,
        robotMutex,
        logger,
        timer,
        robotConfig};

    EXPECT_TRUE(
        robotController.initialize());

    EXPECT_TRUE(
        robotController.start());

    driver.fault = true;
    sensor.publishPulse(
        5800,
        20);
    timer.setMs(20);

    robotController.update();

    EXPECT_TRUE(
        robotController.hasFault());

    driver.fault = false;
    sensor.publishPulse(
        5800,
        200);
    timer.setMs(200);

    robotController.update();

    EXPECT_FALSE(
        robotController.hasFault());

    EXPECT_EQ(
        robotController.getCurrentState(),
        RobotState::Paused);
}

TEST_CASE(RobotController_faultStateRejectsInvalidActiveTransition)
{
    RobotHarness harness;
    harness.sensor.startResult = false;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_FALSE(
        harness.robotController.start());

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Fault);

    harness.robotController.clearEmergency();

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Fault);
}

TEST_CASE(RobotController_pipelineTimingViolationIsDetected)
{
    RobotHarness harness;
    harness.driver.timerToAdvance = &harness.timer;
    harness.driver.advanceOnDualCommandUs = 20000;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.sensor.publishPulse(
        5800,
        20);
    harness.timer.setMs(20);

    harness.robotController.update();

    harness.sensor.publishPulse(
        5800,
        40);
    harness.timer.setMs(40);

    harness.robotController.update();

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .timingHealthy);

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .timingViolations,
        1ULL);
}

TEST_CASE(RobotController_runtimeTimingStatisticsTrackAverageAndWorstCase)
{
    RobotHarness harness;
    harness.driver.timerToAdvance = &harness.timer;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.driver.advanceOnDualCommandUs = 1000;
    harness.sensor.publishPulse(
        5800,
        20);
    harness.timer.setMs(20);

    harness.robotController.update();

    harness.driver.advanceOnDualCommandUs = 3000;
    harness.sensor.publishPulse(
        5800,
        40);
    harness.timer.setMs(40);

    harness.robotController.update();

    const PipelineTiming &timing =
        harness.robotController
            .getMemory()
            .pipelineTiming;

    EXPECT_TRUE(
        timing.averageLoopDurationUs > 0U);

    EXPECT_TRUE(
        timing.worstCaseLoopDurationUs >=
        timing.averageLoopDurationUs);
}

TEST_CASE(RobotController_initializeIsIdempotent)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.sensor.initialized);
}

TEST_CASE(RobotController_sensorInitializeFailureActivatesFault)
{
    RobotHarness harness;
    harness.sensor.initializeResult = false;

    EXPECT_FALSE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.hasFault());

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Fault);
}

TEST_CASE(RobotController_motorInitializeFailureActivatesFault)
{
    RobotHarness harness;
    harness.driver.initializeResult = false;

    EXPECT_FALSE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.hasFault());

    EXPECT_EQ(
        harness.robotController.getCurrentState(),
        RobotState::Fault);
}

TEST_CASE(RobotController_startBeforeInitializeReturnsFalse)
{
    RobotHarness harness;

    EXPECT_FALSE(
        harness.robotController.start());

    EXPECT_FALSE(
        harness.robotController.isRunning());
}

TEST_CASE(RobotController_startIsIdempotent)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    EXPECT_TRUE(
        harness.robotController.start());

    EXPECT_TRUE(
        harness.robotController.isRunning());
}

TEST_CASE(RobotController_stopWhenNotRunningIsNoop)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    harness.robotController.stop();

    EXPECT_FALSE(
        harness.robotController.isRunning());
}

TEST_CASE(RobotController_updateWhenStoppedDoesNotRunPipeline)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    harness.sensor.publishPulse(
        5800,
        20);
    harness.timer.setMs(20);

    harness.robotController.update();

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .totalControlLoops,
        0ULL);
}

TEST_CASE(RobotController_updateWhenMutexBusyDoesNotRunPipeline)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.robotMutex.setForceBusy(true);

    harness.sensor.publishPulse(
        5800,
        20);
    harness.timer.setMs(20);

    harness.robotController.update();

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .totalControlLoops,
        0ULL);
}

TEST_CASE(RobotController_behaviorModeCanBeChanged)
{
    RobotHarness harness;

    EXPECT_TRUE(
        harness.robotController.initialize());

    harness.robotController.setBehaviorMode(
        RobotBehaviorMode::Conservative);

    EXPECT_EQ(
        harness.robotController.getBehaviorMode(),
        RobotBehaviorMode::Conservative);
}

TEST_CASE(RobotController_healthReflectsTimingAndEmergencyState)
{
    RobotHarness harness;
    harness.driver.timerToAdvance = &harness.timer;
    harness.driver.advanceOnDualCommandUs = 20000;

    EXPECT_TRUE(
        harness.robotController.initialize());

    EXPECT_TRUE(
        harness.robotController.start());

    harness.sensor.publishPulse(
        5800,
        20);
    harness.timer.setMs(20);

    harness.robotController.update();

    harness.driver.advanceOnDualCommandUs = 0;
    harness.sensor.publishPulse(
        5800,
        60);
    harness.timer.setMs(60);

    harness.robotController.update();

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .timingHealthy);

    EXPECT_FALSE(
        harness.robotController
            .getSystemHealth()
            .systemHealthy);

    EXPECT_EQ(
        harness.robotController
            .getMemory()
            .runtimeStatistics
            .lastStatisticsUpdateTimestampMs,
        60U);

    harness.robotController.emergencyStop();

    EXPECT_TRUE(
        harness.robotController
            .getSystemHealth()
            .emergencyActive);
}
