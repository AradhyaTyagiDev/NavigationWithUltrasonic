#include "framework/TestFramework.hpp"
#include "fake_espidf/FakeEspIdf.hpp"

#include "sensor/ultrasonic/include/UltrasonicSensor.hpp"

namespace
{
    UltrasonicSensor::Config sensorConfig()
    {
        UltrasonicSensor::Config config;
        config.trigPin = 10;
        config.echoPin = 11;
        config.sensorFrequencyHz = 50;
        config.taskPriority = 6;
        config.taskCore = 0;
        config.queueSize = 1;

        return config;
    }
}

TEST_CASE(UltrasonicHost_initializeConfiguresQueueGpioAndRmt)
{
    fake_espidf::reset();

    UltrasonicSensor sensor(
        sensorConfig());

    EXPECT_TRUE(
        sensor.initialize());

    EXPECT_TRUE(
        sensor.isHealthy());

    EXPECT_EQ(
        fake_espidf::state().queues.size(),
        1U);

    EXPECT_EQ(
        fake_espidf::state().gpioLevels[sensorConfig().trigPin],
        0);

    EXPECT_EQ(
        fake_espidf::state().rmtChannels.size(),
        1U);

    EXPECT_TRUE(
        fake_espidf::state().rmtChannels.front()->enabled);
}

TEST_CASE(UltrasonicHost_startCreatesSensorTask)
{
    fake_espidf::reset();

    UltrasonicSensor sensor(
        sensorConfig());

    EXPECT_TRUE(
        sensor.initialize());

    EXPECT_TRUE(
        sensor.start());

    EXPECT_TRUE(
        sensor.isRunning());

    EXPECT_EQ(
        fake_espidf::state().tasks.size(),
        1U);

    EXPECT_EQ(
        fake_espidf::state().tasks.front()->name,
        std::string("UltrasonicTask"));

    EXPECT_EQ(
        fake_espidf::state().tasks.front()->priority,
        6U);
}

TEST_CASE(UltrasonicHost_rmtCallbackPublishesLatestPulse)
{
    fake_espidf::reset();
    fake_espidf::setEspTimerUs(123000);

    UltrasonicSensor sensor(
        sensorConfig());

    EXPECT_TRUE(
        sensor.initialize());

    fake_espidf::emitRmtRx(
        fake_current_rmt_channel(),
        5800);

    UltrasonicSensorData data;

    EXPECT_TRUE(
        sensor.fetchLatestData(data));

    EXPECT_EQ(
        data.pulseWidthUs,
        5800U);

    EXPECT_EQ(
        data.timestampMs,
        123U);
}

TEST_CASE(UltrasonicHost_zeroPulseIsIgnored)
{
    fake_espidf::reset();

    UltrasonicSensor sensor(
        sensorConfig());

    EXPECT_TRUE(
        sensor.initialize());

    fake_espidf::emitRmtRx(
        fake_current_rmt_channel(),
        0);

    UltrasonicSensorData data;

    EXPECT_FALSE(
        sensor.fetchLatestData(data));
}

TEST_CASE(UltrasonicHost_shutdownDeletesTaskQueueAndRmt)
{
    fake_espidf::reset();

    UltrasonicSensor sensor(
        sensorConfig());

    EXPECT_TRUE(
        sensor.initialize());

    EXPECT_TRUE(
        sensor.start());

    sensor.shutdown();

    EXPECT_FALSE(
        sensor.isRunning());

    EXPECT_FALSE(
        sensor.isHealthy());

    EXPECT_EQ(
        fake_espidf::state().tasks.size(),
        0U);

    EXPECT_EQ(
        fake_espidf::state().queues.size(),
        0U);

    EXPECT_EQ(
        fake_espidf::state().rmtChannels.size(),
        0U);
}
