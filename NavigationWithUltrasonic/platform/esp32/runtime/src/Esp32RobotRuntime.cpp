#include "runtime/include/Esp32RobotRuntime.hpp"

#include "interfaces/include/logging/LoggerExtensions.hpp"

namespace
{
    constexpr const char *TAG = "Esp32RobotRuntime";
}

Esp32RobotRuntime::Esp32RobotRuntime(
    RobotController &robotController,
    MotorController &motorController,
    ITimer &timer,
    ILogger &logger,
    const RobotTaskConfig &taskConfig)
    : m_robotController(robotController),
      m_motorController(motorController),
      m_timer(timer),
      m_logger(logger),
      m_taskConfig(taskConfig)
{
}

Esp32RobotRuntime::~Esp32RobotRuntime()
{
    shutdown();
}

bool Esp32RobotRuntime::initialize()
{
    if (m_initialized)
    {
        return true;
    }

    if (!m_robotController.initialize())
    {
        Logger::error(
            m_logger,
            TAG,
            "RobotController initialization failed");

        return false;
    }

    m_initialized = true;

    return true;
}

bool Esp32RobotRuntime::start()
{
    if (!initialize())
    {
        return false;
    }

    if (m_running.load())
    {
        return true;
    }

    if (!m_robotController.start())
    {
        Logger::error(
            m_logger,
            TAG,
            "RobotController start failed");

        return false;
    }

    m_running.store(true);

    if (!createTasks())
    {
        Logger::error(
            m_logger,
            TAG,
            "Runtime task creation failed");

        m_running.store(false);
        destroyTasks();
        m_robotController.stop();

        return false;
    }

    Logger::info(
        m_logger,
        TAG,
        "ESP32 robot runtime started");

    return true;
}

void Esp32RobotRuntime::stop()
{
    if (!m_running.exchange(false))
    {
        return;
    }

    waitForTasksToExit(250);

    destroyTasks();

    m_robotController.stop();

    Logger::info(
        m_logger,
        TAG,
        "ESP32 robot runtime stopped");
}

void Esp32RobotRuntime::shutdown()
{
    stop();

    if (!m_initialized)
    {
        return;
    }

    m_robotController.shutdown();

    m_initialized = false;
}

bool Esp32RobotRuntime::isRunning() const
{
    return m_running.load();
}

bool Esp32RobotRuntime::createTasks()
{
    BaseType_t result =
        xTaskCreatePinnedToCore(
            controllerTaskEntry,
            "RobotControllerTask",
            m_taskConfig.controllerTaskStackSize,
            this,
            m_taskConfig.controllerTaskPriority,
            &m_controllerTaskHandle,
            m_taskConfig.controllerCore);

    if (result != pdPASS)
    {
        return false;
    }

    result =
        xTaskCreatePinnedToCore(
            motorDriverTaskEntry,
            "MotorDriverTask",
            m_taskConfig.driverTaskStackSize,
            this,
            m_taskConfig.driverTaskPriority,
            &m_motorDriverTaskHandle,
            m_taskConfig.driverCore);

    if (result != pdPASS)
    {
        return false;
    }

    if (m_taskConfig.telemetryHz > 0)
    {
        result =
            xTaskCreatePinnedToCore(
                telemetryTaskEntry,
                "TelemetryTask",
                m_taskConfig.telemetryTaskStackSize,
                this,
                m_taskConfig.telemetryTaskPriority,
                &m_telemetryTaskHandle,
                m_taskConfig.telemetryCore);

        if (result != pdPASS)
        {
            return false;
        }
    }

    return true;
}

void Esp32RobotRuntime::destroyTasks()
{
    deleteTask(m_controllerTaskHandle);
    deleteTask(m_motorDriverTaskHandle);
    deleteTask(m_telemetryTaskHandle);
}

void Esp32RobotRuntime::waitForTasksToExit(
    uint32_t timeoutMs)
{
    TaskHandle_t currentTask =
        xTaskGetCurrentTaskHandle();

    const TickType_t startTick =
        xTaskGetTickCount();

    const TickType_t timeoutTicks =
        pdMS_TO_TICKS(
            timeoutMs);

    while (
        ((m_controllerTaskHandle != nullptr &&
          m_controllerTaskHandle != currentTask) ||
         (m_motorDriverTaskHandle != nullptr &&
          m_motorDriverTaskHandle != currentTask) ||
         (m_telemetryTaskHandle != nullptr &&
          m_telemetryTaskHandle != currentTask)) &&
        ((xTaskGetTickCount() - startTick) <
         timeoutTicks))
    {
        vTaskDelay(
            pdMS_TO_TICKS(1));
    }
}

void Esp32RobotRuntime::deleteTask(
    TaskHandle_t &taskHandle)
{
    if (taskHandle == nullptr)
    {
        return;
    }

    if (taskHandle == xTaskGetCurrentTaskHandle())
    {
        return;
    }

    TaskHandle_t taskToDelete =
        taskHandle;

    taskHandle = nullptr;

    vTaskDelete(taskToDelete);
}

void Esp32RobotRuntime::controllerTaskEntry(
    void *context)
{
    static_cast<Esp32RobotRuntime *>(context)
        ->controllerTaskLoop();
}

void Esp32RobotRuntime::motorDriverTaskEntry(
    void *context)
{
    static_cast<Esp32RobotRuntime *>(context)
        ->motorDriverTaskLoop();
}

void Esp32RobotRuntime::telemetryTaskEntry(
    void *context)
{
    static_cast<Esp32RobotRuntime *>(context)
        ->telemetryTaskLoop();
}

void Esp32RobotRuntime::controllerTaskLoop()
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    const TickType_t period =
        periodFromHz(
            m_taskConfig.controllerHz);

    while (m_running.load())
    {
        m_robotController.update();

        vTaskDelayUntil(
            &previousWakeTime,
            period);
    }

    m_controllerTaskHandle = nullptr;

    vTaskDelete(nullptr);
}

void Esp32RobotRuntime::motorDriverTaskLoop()
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    const TickType_t period =
        periodFromHz(
            m_taskConfig.driverHz);

    while (m_running.load())
    {
        m_motorController.update(
            Timer::milliseconds(
                m_timer));

        vTaskDelayUntil(
            &previousWakeTime,
            period);
    }

    m_motorDriverTaskHandle = nullptr;

    vTaskDelete(nullptr);
}

void Esp32RobotRuntime::telemetryTaskLoop()
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    const TickType_t period =
        periodFromHz(
            m_taskConfig.telemetryHz);

    while (m_running.load())
    {
        vTaskDelayUntil(
            &previousWakeTime,
            period);
    }

    m_telemetryTaskHandle = nullptr;

    vTaskDelete(nullptr);
}

TickType_t Esp32RobotRuntime::periodFromHz(
    uint32_t frequencyHz)
{
    if (frequencyHz == 0)
    {
        return pdMS_TO_TICKS(1000);
    }

    const uint32_t periodMs =
        1000U / frequencyHz;

    return pdMS_TO_TICKS(
        periodMs == 0 ? 1U : periodMs);
}
