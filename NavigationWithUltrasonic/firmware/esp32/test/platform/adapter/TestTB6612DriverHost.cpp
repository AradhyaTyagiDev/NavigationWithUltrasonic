#include "framework/TestFramework.hpp"
#include "fakes/Fakes.hpp"
#include "fake_espidf/FakeEspIdf.hpp"

#include "motor/TB6612FNG/include/TB6612Driver.hpp"

namespace
{
    TB6612DriverConfig driverConfig()
    {
        TB6612DriverConfig config;
        config.leftMotorIN1Pin = 2;
        config.leftMotorIN2Pin = 3;
        config.leftMotorPWMPin = 4;
        config.rightMotorIN1Pin = 5;
        config.rightMotorIN2Pin = 6;
        config.rightMotorPWMPin = 7;
        config.standbyPin = 8;
        config.maximumPWMDuty = 1000;
        config.minimumEffectivePWMDuty = 0;
        config.enableDeadzoneCompensation = false;
        config.enableStartupBoost = false;
        config.enablePWMRamping = false;
        config.enableSafeReverseSequence = false;
        config.enableSafeDirectionTransition = false;

        return config;
    }

    MotorDriverCommand command(
        MotorChannel channel,
        MotorDirection direction,
        float speed)
    {
        MotorDriverCommand result;
        result.channel = channel;
        result.direction = direction;
        result.normalizedSpeed = speed;
        result.enabled = true;

        return result;
    }
}

TEST_CASE(TB6612Host_initializeConfiguresGpioPwmAndSafeState)
{
    fake_espidf::reset();

    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    TB6612DriverConfig config =
        driverConfig();

    TB6612Driver driver(
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        driver.initialize());

    EXPECT_TRUE(
        driver.isReady());

    EXPECT_EQ(
        fake_espidf::state().gpioConfigs.size(),
        1U);

    EXPECT_EQ(
        fake_espidf::state().ledcTimerConfigs.size(),
        1U);

    EXPECT_EQ(
        fake_espidf::state().ledcChannelConfigs.size(),
        2U);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.standbyPin],
        0);

    EXPECT_EQ(
        fake_espidf::state().ledcDuty[config.leftPWMChannel],
        0U);

    EXPECT_EQ(
        fake_espidf::state().ledcDuty[config.rightPWMChannel],
        0U);
}

TEST_CASE(TB6612Host_forwardCommandSetsDirectionStandbyAndPwm)
{
    fake_espidf::reset();

    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    TB6612DriverConfig config =
        driverConfig();

    TB6612Driver driver(
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        driver.initialize());

    timer.setMs(100);

    driver.executeCommand(
        command(
            MotorChannel::Left,
            MotorDirection::Forward,
            0.5f));

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.standbyPin],
        1);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.leftMotorIN1Pin],
        1);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.leftMotorIN2Pin],
        0);

    EXPECT_EQ(
        fake_espidf::state().ledcDuty[config.leftPWMChannel],
        500U);

    EXPECT_EQ(
        driver.getStatus().currentLeftPWM,
        500U);
}

TEST_CASE(TB6612Host_motorInversionSwapsDirectionPins)
{
    fake_espidf::reset();

    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    TB6612DriverConfig config =
        driverConfig();
    config.invertLeftMotor = true;

    TB6612Driver driver(
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        driver.initialize());

    driver.executeCommand(
        command(
            MotorChannel::Left,
            MotorDirection::Forward,
            0.3f));

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.leftMotorIN1Pin],
        0);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.leftMotorIN2Pin],
        1);
}

TEST_CASE(TB6612Host_brakeAndEmergencyStopForceSafeOutputs)
{
    fake_espidf::reset();

    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    TB6612DriverConfig config =
        driverConfig();

    TB6612Driver driver(
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        driver.initialize());

    MotorDriverCommand brake =
        command(
            MotorChannel::Right,
            MotorDirection::Forward,
            0.6f);
    brake.brakeMode = BrakeMode::Active;

    driver.executeCommand(
        brake);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.rightMotorIN1Pin],
        1);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.rightMotorIN2Pin],
        1);

    EXPECT_EQ(
        fake_espidf::state().ledcDuty[config.rightPWMChannel],
        0U);

    driver.emergencyStop();

    EXPECT_TRUE(
        driver.getStatus().emergencyStopActive);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.standbyPin],
        0);
}

TEST_CASE(TB6612Host_watchdogTimeoutEntersEmergencyStop)
{
    fake_espidf::reset();

    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    TB6612DriverConfig config =
        driverConfig();
    config.commandTimeoutMs = 50;

    TB6612Driver driver(
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        driver.initialize());

    timer.setMs(100);

    driver.executeCommand(
        command(
            MotorChannel::Left,
            MotorDirection::Forward,
            0.4f));

    timer.setMs(200);

    driver.update(200);

    EXPECT_TRUE(
        driver.getStatus().emergencyStopActive);

    EXPECT_EQ(
        driver.getDriverState(),
        MotorDriverState::EmergencyStopped);
}

TEST_CASE(TB6612Host_immediateReverseIsRejectedWhenProtectionEnabled)
{
    fake_espidf::reset();

    FakeMutex mutex;
    FakeLogger logger;
    FakeTimer timer;
    TB6612DriverConfig config =
        driverConfig();
    config.enableSafeDirectionTransition = true;

    TB6612Driver driver(
        mutex,
        logger,
        timer,
        config);

    EXPECT_TRUE(
        driver.initialize());

    timer.setMs(100);
    driver.executeCommand(
        command(
            MotorChannel::Left,
            MotorDirection::Forward,
            0.5f));

    timer.setMs(120);
    driver.executeCommand(
        command(
            MotorChannel::Left,
            MotorDirection::Reverse,
            0.5f));

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.leftMotorIN1Pin],
        1);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[config.leftMotorIN2Pin],
        0);

    EXPECT_EQ(
        driver.getStatus().currentLeftPWM,
        500U);
}
