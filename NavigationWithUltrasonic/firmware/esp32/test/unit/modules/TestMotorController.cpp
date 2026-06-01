#include "framework/TestFramework.hpp"
#include "fakes/Fakes.hpp"

#include "motor/controller/include/MotorController.hpp"

namespace
{
    MotionCommand forwardCommand()
    {
        MotionCommand command;
        command.leftWheelSpeed = 0.5f;
        command.rightWheelSpeed = 0.5f;
        command.motionConfidence = 1.0f;

        return command;
    }
}

TEST_CASE(MotorController_executeMotionSubmitsDualDriverCommand)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_EQ(
        driver.dualCommandCount,
        1U);

    EXPECT_EQ(
        driver.lastLeftCommand.direction,
        MotorDirection::Forward);

    EXPECT_EQ(
        driver.lastRightCommand.direction,
        MotorDirection::Forward);
}

TEST_CASE(MotorController_tryExecuteMotionReturnsFalseWhenBusy)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    mutex.setForceBusy(true);

    EXPECT_FALSE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_EQ(
        driver.dualCommandCount,
        0U);
}

TEST_CASE(MotorController_invalidMotionCommandTriggersFault)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command =
        forwardCommand();

    command.leftWheelSpeed = 2.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_TRUE(
        controller.hasFault());

    EXPECT_TRUE(
        driver.emergencyStopActive);
}

TEST_CASE(MotorController_emergencyStopBlocksFurtherMotion)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    controller.emergencyStop();

    EXPECT_TRUE(
        controller.isEmergencyStopActive());

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_EQ(
        driver.dualCommandCount,
        0U);
}

TEST_CASE(MotorController_clearEmergencyAllowsMotionAgain)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    controller.emergencyStop();
    controller.clearEmergencyStop();

    EXPECT_FALSE(
        controller.isEmergencyStopActive());

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_EQ(
        driver.dualCommandCount,
        1U);
}

TEST_CASE(MotorController_updateDelegatesToDriver)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.controlFrequencyHz = 50;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    controller.update(
        100);

    EXPECT_EQ(
        driver.updateCount,
        1U);

    EXPECT_EQ(
        driver.lastUpdateTimestampMs,
        100U);
}

TEST_CASE(MotorController_driverFaultDuringMotionTriggersFault)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    driver.fault = true;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_TRUE(
        controller.hasFault());
}

TEST_CASE(MotorController_reverseTransitionProtectionZerosInstantReverse)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableSafeReverseTransition = true;
    config.maximumAccelerationPercentPerSec = 100.0f;
    config.maximumDecelerationPercentPerSec = 100.0f;
    config.enableMotionSmoothing = false;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    timer.setMs(100);

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    MotionCommand reverse;
    reverse.leftWheelSpeed = -0.5f;
    reverse.rightWheelSpeed = -0.5f;
    reverse.motionConfidence = 1.0f;

    timer.setMs(120);

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            reverse));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.0f,
        0.001f);

    EXPECT_NEAR(
        driver.lastRightCommand.normalizedSpeed,
        0.0f,
        0.001f);
}

TEST_CASE(MotorController_accelerationLimitCapsSpeedIncrease)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableStartupBoost = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.0f;
    config.maximumAccelerationPercentPerSec = 1.0f;
    config.maximumDecelerationPercentPerSec = 10.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    timer.setMs(100);

    MotionCommand command;
    command.leftWheelSpeed = 1.0f;
    command.rightWheelSpeed = 1.0f;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.1f,
        0.001f);
}

TEST_CASE(MotorController_decelerationLimitCapsSpeedDecrease)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableStartupBoost = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.0f;
    config.maximumAccelerationPercentPerSec = 10.0f;
    config.maximumDecelerationPercentPerSec = 1.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    timer.setMs(100);

    MotionCommand fast;
    fast.leftWheelSpeed = 1.0f;
    fast.rightWheelSpeed = 1.0f;
    fast.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            fast));

    timer.setMs(200);

    MotionCommand stop;
    stop.leftWheelSpeed = 0.0f;
    stop.rightWheelSpeed = 0.0f;
    stop.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            stop));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.9f,
        0.001f);
}

TEST_CASE(MotorController_startupBoostRaisesSmallInitialCommand)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableStartupBoost = true;
    config.enableMotionSmoothing = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.2f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    timer.setMs(100);

    MotionCommand command;
    command.leftWheelSpeed = 0.05f;
    command.rightWheelSpeed = 0.05f;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.2f,
        0.001f);
}

TEST_CASE(MotorController_deadzoneCompensationRaisesSmallCommand)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableStartupBoost = false;
    config.enableMotionSmoothing = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.2f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    timer.setMs(100);

    MotionCommand command;
    command.leftWheelSpeed = 0.05f;
    command.rightWheelSpeed = 0.05f;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.2f,
        0.001f);
}

TEST_CASE(MotorController_coordinatedBrakingActivatesBrakeOnStop)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableCoordinatedBraking = true;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command;
    command.leftWheelSpeed = 0.0f;
    command.rightWheelSpeed = 0.0f;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_EQ(
        driver.lastLeftCommand.brakeMode,
        BrakeMode::Active);

    EXPECT_EQ(
        driver.lastRightCommand.brakeMode,
        BrakeMode::Active);
}

TEST_CASE(MotorController_initializeFailureReturnsFalse)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    driver.initializeResult = false;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_FALSE(
        controller.initialize());

    EXPECT_FALSE(
        driver.ready);
}

TEST_CASE(MotorController_executeMotionBlockingWrapperSubmitsCommand)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    controller.executeMotion(
        forwardCommand());

    EXPECT_EQ(
        driver.dualCommandCount,
        1U);
}

TEST_CASE(MotorController_updateReturnsEarlyWhenCalledTooSoon)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.controlFrequencyHz = 50;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    controller.update(
        1);

    EXPECT_EQ(
        driver.updateCount,
        0U);
}

TEST_CASE(MotorController_motionTimeoutStopsDriver)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.motorCommandTimeoutMs = 50;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    timer.setMs(100);

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    controller.update(
        200);

    EXPECT_TRUE(
        driver.stopped);
}

TEST_CASE(MotorController_gettersExposeCurrentStateAndMemory)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    EXPECT_EQ(
        controller.getCurrentState(),
        MotorState::Idle);

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    const MotorControllerMemory memory =
        controller.getMemory();

    EXPECT_EQ(
        memory.executedCommandCount,
        1ULL);
}

TEST_CASE(MotorController_driverNotReadyTriggersFault)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    driver.ready = false;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_TRUE(
        controller.hasFault());
}

TEST_CASE(MotorController_negativeConfidenceTriggersFault)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        MotorControllerConfig{});

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command =
        forwardCommand();

    command.motionConfidence = -0.1f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_TRUE(
        controller.hasFault());
}

TEST_CASE(MotorController_wheelSynchronizationAveragesLargeDifference)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableWheelSynchronization = true;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command;
    command.leftWheelSpeed = 1.0f;
    command.rightWheelSpeed = 0.0f;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.5f,
        0.001f);

    EXPECT_NEAR(
        driver.lastRightCommand.normalizedSpeed,
        0.5f,
        0.001f);
}

TEST_CASE(MotorController_startupBoostDisabledKeepsSmallCommand)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableStartupBoost = false;
    config.enableMotionSmoothing = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command;
    command.leftWheelSpeed = 0.05f;
    command.rightWheelSpeed = 0.05f;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.05f,
        0.001f);
}

TEST_CASE(MotorController_safeReverseDisabledAllowsInstantReverse)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableSafeReverseTransition = false;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    MotionCommand reverse;
    reverse.leftWheelSpeed = -0.5f;
    reverse.rightWheelSpeed = -0.5f;
    reverse.reverseMotionActive = true;
    reverse.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            reverse));

    EXPECT_EQ(
        driver.lastLeftCommand.direction,
        MotorDirection::Reverse);

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.5f,
        0.001f);
}

TEST_CASE(MotorController_reverseToForwardProtectionZerosInstantForward)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableSafeReverseTransition = true;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand reverse;
    reverse.leftWheelSpeed = -0.5f;
    reverse.rightWheelSpeed = -0.5f;
    reverse.reverseMotionActive = true;
    reverse.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            reverse));

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            forwardCommand()));

    EXPECT_NEAR(
        driver.lastLeftCommand.normalizedSpeed,
        0.0f,
        0.001f);

    EXPECT_NEAR(
        driver.lastRightCommand.normalizedSpeed,
        0.0f,
        0.001f);
}

TEST_CASE(MotorController_brakingCommandDeterminesBrakingState)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command;
    command.leftWheelSpeed = 0.0f;
    command.rightWheelSpeed = 0.0f;
    command.brakingActive = true;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_EQ(
        controller.getCurrentState(),
        MotorState::Braking);
}

TEST_CASE(MotorController_reverseCommandDeterminesReverseState)
{
    FakeMotorDriver driver;
    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;

    MotorControllerConfig config;
    config.enableSafeReverseTransition = false;
    config.enableMotionSmoothing = false;
    config.enableStartupBoost = false;
    config.enableWheelSynchronization = false;
    config.minimumEffectiveSpeedPercent = 0.0f;

    MotorController controller(
        driver,
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        controller.initialize());

    MotionCommand command;
    command.leftWheelSpeed = -0.4f;
    command.rightWheelSpeed = -0.4f;
    command.reverseMotionActive = true;
    command.motionConfidence = 1.0f;

    EXPECT_TRUE(
        controller.tryExecuteMotion(
            command));

    EXPECT_EQ(
        controller.getCurrentState(),
        MotorState::Reverse);
}
