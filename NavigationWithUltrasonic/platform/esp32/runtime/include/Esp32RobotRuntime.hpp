#pragma once

#include "interfaces/include/logging/ILogger.hpp"
#include "interfaces/include/runtime/IRobotRuntime.hpp"
#include "interfaces/include/timing/ITimer.hpp"

#include "motor/controller/include/MotorController.hpp"
#include "robot/include/RobotController.hpp"
#include "robot/include/RobotTaskConfig.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <stdint.h>

class Esp32RobotRuntime final : public IRobotRuntime
{
public:
    Esp32RobotRuntime(
        RobotController &robotController,
        MotorController &motorController,
        ITimer &timer,
        ILogger &logger,
        const RobotTaskConfig &taskConfig);

    ~Esp32RobotRuntime() override;

    bool initialize() override;

    bool start() override;

    void stop() override;

    void shutdown() override;

    bool isRunning() const override;

private:
    static void controllerTaskEntry(void *context);

    static void motorDriverTaskEntry(void *context);

    static void telemetryTaskEntry(void *context);

    void controllerTaskLoop();

    void motorDriverTaskLoop();

    void telemetryTaskLoop();

    bool createTasks();

    void destroyTasks();

    void waitForTasksToExit(uint32_t timeoutMs);

    static void deleteTask(TaskHandle_t &taskHandle);

    static TickType_t periodFromHz(uint32_t frequencyHz);

private:
    RobotController &m_robotController;

    MotorController &m_motorController;

    ITimer &m_timer;

    ILogger &m_logger;

    RobotTaskConfig m_taskConfig;

    TaskHandle_t m_controllerTaskHandle = nullptr;

    TaskHandle_t m_motorDriverTaskHandle = nullptr;

    TaskHandle_t m_telemetryTaskHandle = nullptr;

    std::atomic<bool> m_running{false};

    bool m_initialized = false;
};
