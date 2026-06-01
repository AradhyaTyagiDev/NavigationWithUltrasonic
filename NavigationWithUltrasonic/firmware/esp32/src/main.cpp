#include "filter/include/UltrasonicFilter.hpp"
#include "filter/include/UltrasonicFilterConfig.hpp"
#include "interfaces/include/logging/LoggerExtensions.hpp"
#include "motion/include/MotionPlanner.hpp"
#include "motion/include/MotionPlannerConfig.hpp"
#include "motor/TB6612FNG/include/TB6612Driver.hpp"
#include "motor/TB6612FNG/include/TB6612DriverConfig.hpp"
#include "motor/controller/include/MotorController.hpp"
#include "motor/controller/include/MotorControllerConfig.hpp"
#include "navigation/include/NavigationManager.hpp"
#include "navigation/include/NavigationManagerConfig.hpp"
#include "obstacle/include/ObstacleManager.hpp"
#include "obstacle/include/ObstacleManagerConfig.hpp"
#include "robot/include/RobotController.hpp"
#include "robot/include/RobotControllerConfig.hpp"
#include "runtime/include/Esp32RobotRuntime.hpp"
#include "sensor/ultrasonic/include/UltrasonicSensor.hpp"

#include "logging/ESPLogger.hpp"
#include "synchronization/include/FreeRTOSMutex.hpp"
#include "timing/ESPTimer.hpp"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef ROBOT_ULTRASONIC_TRIG_PIN
#define ROBOT_ULTRASONIC_TRIG_PIN 25
#endif

#ifndef ROBOT_ULTRASONIC_ECHO_PIN
#define ROBOT_ULTRASONIC_ECHO_PIN 32
#endif

#ifndef ROBOT_LEFT_IN1_PIN
#define ROBOT_LEFT_IN1_PIN 16
#endif

#ifndef ROBOT_LEFT_IN2_PIN
#define ROBOT_LEFT_IN2_PIN 17
#endif

#ifndef ROBOT_LEFT_PWM_PIN
#define ROBOT_LEFT_PWM_PIN 18
#endif

#ifndef ROBOT_RIGHT_IN1_PIN
#define ROBOT_RIGHT_IN1_PIN 19
#endif

#ifndef ROBOT_RIGHT_IN2_PIN
#define ROBOT_RIGHT_IN2_PIN 21
#endif

#ifndef ROBOT_RIGHT_PWM_PIN
#define ROBOT_RIGHT_PWM_PIN 22
#endif

#ifndef ROBOT_MOTOR_STANDBY_PIN
#define ROBOT_MOTOR_STANDBY_PIN 23
#endif

namespace
{
    constexpr const char *TAG = "Main";

    gpio_num_t gpioFromPin(
        int pin)
    {
        return static_cast<gpio_num_t>(
            pin);
    }

    RobotControllerConfig makeRobotControllerConfig()
    {
        RobotControllerConfig config;

        config.controlLoopHz = 50;
        config.controlLoopIntervalMs = 20;

        config.taskConfig.sensorTaskPriority = 6;
        config.taskConfig.controllerTaskPriority = 8;
        config.taskConfig.driverTaskPriority = 7;
        config.taskConfig.telemetryTaskPriority = 2;

        config.taskConfig.sensorCore = 0;
        config.taskConfig.controllerCore = 1;
        config.taskConfig.driverCore = 1;
        config.taskConfig.telemetryCore = 0;

        config.taskConfig.sensorHz = 50;
        config.taskConfig.controllerHz = 50;
        config.taskConfig.driverHz = 200;
        config.taskConfig.telemetryHz = 5;

        config.taskConfig.sensorTaskStackSize = 4096;
        config.taskConfig.controllerTaskStackSize = 8192;
        config.taskConfig.driverTaskStackSize = 4096;
        config.taskConfig.telemetryTaskStackSize = 4096;

        return config;
    }

    UltrasonicSensor::Config makeUltrasonicConfig(
        const RobotTaskConfig &taskConfig)
    {
        UltrasonicSensor::Config config;

        config.trigPin =
            gpioFromPin(
                ROBOT_ULTRASONIC_TRIG_PIN);

        config.echoPin =
            gpioFromPin(
                ROBOT_ULTRASONIC_ECHO_PIN);

        config.taskStackSize =
            taskConfig.sensorTaskStackSize;

        config.taskPriority =
            taskConfig.sensorTaskPriority;

        config.taskCore =
            taskConfig.sensorCore;

        config.sensorFrequencyHz =
            taskConfig.sensorHz;

        config.queueSize = 1;

        return config;
    }

    TB6612DriverConfig makeMotorDriverConfig()
    {
        TB6612DriverConfig config;

        config.leftMotorIN1Pin =
            gpioFromPin(
                ROBOT_LEFT_IN1_PIN);

        config.leftMotorIN2Pin =
            gpioFromPin(
                ROBOT_LEFT_IN2_PIN);

        config.leftMotorPWMPin =
            gpioFromPin(
                ROBOT_LEFT_PWM_PIN);

        config.rightMotorIN1Pin =
            gpioFromPin(
                ROBOT_RIGHT_IN1_PIN);

        config.rightMotorIN2Pin =
            gpioFromPin(
                ROBOT_RIGHT_IN2_PIN);

        config.rightMotorPWMPin =
            gpioFromPin(
                ROBOT_RIGHT_PWM_PIN);

        config.standbyPin =
            gpioFromPin(
                ROBOT_MOTOR_STANDBY_PIN);

        return config;
    }
}

extern "C" void app_main()
{
    static ESPLogger logger(LogLevel::Info);
    static ESPTimer timer;

    static FreeRTOSMutex robotMutex;
    static FreeRTOSMutex motorControllerMutex;
    static FreeRTOSMutex motorDriverMutex;

    static const RobotControllerConfig robotConfig =
        makeRobotControllerConfig();

    static const UltrasonicSensor::Config ultrasonicConfig =
        makeUltrasonicConfig(
            robotConfig.taskConfig);

    static const TB6612DriverConfig motorDriverConfig =
        makeMotorDriverConfig();

    static const UltrasonicFilterConfig ultrasonicFilterConfig;
    static const ObstacleManagerConfig obstacleManagerConfig;
    static const NavigationManagerConfig navigationManagerConfig;
    static const MotionPlannerConfig motionPlannerConfig;
    static const MotorControllerConfig motorControllerConfig;

    static UltrasonicSensor ultrasonicSensor(
        ultrasonicConfig);

    static UltrasonicFilter ultrasonicFilter(
        ultrasonicFilterConfig);

    static ObstacleManager obstacleManager(
        obstacleManagerConfig);

    static NavigationManager navigationManager(
        navigationManagerConfig);

    static MotionPlanner motionPlanner(
        motionPlannerConfig);

    static TB6612Driver motorDriver(
        motorDriverMutex,
        logger,
        timer,
        motorDriverConfig);

    static MotorController motorController(
        motorDriver,
        motorControllerMutex,
        logger,
        timer,
        motorControllerConfig);

    static RobotController robotController(
        ultrasonicSensor,
        ultrasonicFilter,
        obstacleManager,
        navigationManager,
        motionPlanner,
        motorController,
        robotMutex,
        logger,
        timer,
        robotConfig);

    static Esp32RobotRuntime runtime(
        robotController,
        motorController,
        timer,
        logger,
        robotConfig.taskConfig);

    if (!runtime.start())
    {
        Logger::critical(
            logger,
            TAG,
            "Robot runtime failed to start");

        while (true)
        {
            vTaskDelay(
                pdMS_TO_TICKS(1000));
        }
    }

    vTaskDelete(nullptr);
}
