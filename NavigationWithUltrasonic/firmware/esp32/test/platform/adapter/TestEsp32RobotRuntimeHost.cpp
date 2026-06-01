#include "framework/TestFramework.hpp"
#include "fakes/Fakes.hpp"
#include "fake_espidf/FakeEspIdf.hpp"
#include "IntegrationHarness.hpp"

#include "runtime/include/Esp32RobotRuntime.hpp"

TEST_CASE(Esp32RuntimeHost_startCreatesControllerDriverAndTelemetryTasks)
{
    fake_espidf::reset();

    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    RobotTaskConfig taskConfig;
    taskConfig.controllerTaskPriority = 8;
    taskConfig.driverTaskPriority = 7;
    taskConfig.telemetryTaskPriority = 2;
    taskConfig.controllerCore = 1;
    taskConfig.driverCore = 1;
    taskConfig.telemetryCore = 0;
    taskConfig.controllerHz = 50;
    taskConfig.driverHz = 200;
    taskConfig.telemetryHz = 5;

    Esp32RobotRuntime runtime(
        harness.robotController,
        harness.motorController,
        harness.timer,
        harness.logger,
        taskConfig);

    EXPECT_TRUE(
        runtime.start());

    EXPECT_TRUE(
        runtime.isRunning());

    EXPECT_TRUE(
        harness.robotController.isRunning());

    EXPECT_EQ(
        fake_espidf::state().tasks.size(),
        3U);

    EXPECT_EQ(
        fake_espidf::state().tasks[0]->name,
        std::string("RobotControllerTask"));

    EXPECT_EQ(
        fake_espidf::state().tasks[0]->priority,
        8U);

    EXPECT_EQ(
        fake_espidf::state().tasks[0]->core,
        1);

    EXPECT_EQ(
        fake_espidf::state().tasks[1]->name,
        std::string("MotorDriverTask"));

    EXPECT_EQ(
        fake_espidf::state().tasks[1]->priority,
        7U);

    EXPECT_EQ(
        fake_espidf::state().tasks[2]->name,
        std::string("TelemetryTask"));

    runtime.stop();

    EXPECT_FALSE(
        runtime.isRunning());

    EXPECT_FALSE(
        harness.robotController.isRunning());

    EXPECT_EQ(
        fake_espidf::state().tasks.size(),
        0U);
}

TEST_CASE(Esp32RuntimeHost_telemetryTaskCanBeDisabled)
{
    fake_espidf::reset();

    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    RobotTaskConfig taskConfig;
    taskConfig.telemetryHz = 0;

    Esp32RobotRuntime runtime(
        harness.robotController,
        harness.motorController,
        harness.timer,
        harness.logger,
        taskConfig);

    EXPECT_TRUE(
        runtime.start());

    EXPECT_EQ(
        fake_espidf::state().tasks.size(),
        2U);
}

TEST_CASE(Esp32RuntimeHost_taskCreationFailureStopsRobot)
{
    fake_espidf::reset();
    fake_espidf::state().failTaskCreateAtCall = 2;

    IntegrationHarness harness(
        productionLikeIntegrationConfig());

    RobotTaskConfig taskConfig;

    Esp32RobotRuntime runtime(
        harness.robotController,
        harness.motorController,
        harness.timer,
        harness.logger,
        taskConfig);

    EXPECT_FALSE(
        runtime.start());

    EXPECT_FALSE(
        runtime.isRunning());

    EXPECT_FALSE(
        harness.robotController.isRunning());

    EXPECT_EQ(
        fake_espidf::state().tasks.size(),
        0U);
}
