//====================================================
// File: RobotController.cpp
//====================================================

#include "robot/include/RobotController.hpp"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG =
    "RobotController";

//====================================================
// Constructor
//====================================================

RobotController::RobotController(
    IUltrasonicSensor &ultrasonicSensor,
    UltrasonicFilter &ultrasonicFilter,
    ObstacleManager &obstacleManager,
    NavigationManager &navigationManager,
    MotionPlanner &motionPlanner,
    MotorController &motorController,
    const RobotControllerConfig &config)
    : m_ultrasonicSensor(
          ultrasonicSensor),

      m_ultrasonicFilter(
          ultrasonicFilter),

      m_obstacleManager(
          obstacleManager),

      m_navigationManager(
          navigationManager),

      m_motionPlanner(
          motionPlanner),

      m_motorController(
          motorController),

      m_config(config)
{
}

//====================================================
// Destructor
//====================================================

RobotController::~RobotController()
{
    shutdown();
}

//====================================================
// Initialization
//====================================================

bool RobotController::initialize()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Already initialized
    //-----------------------------------------

    if (m_initialized)
    {
        return true;
    }

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        RobotState::Initializing);

    //-----------------------------------------
    // Create mutex
    //-----------------------------------------

    if (m_controllerMutex == nullptr)
    {
        m_controllerMutex =
            xSemaphoreCreateMutex();

        if (m_controllerMutex == nullptr)
        {
            ESP_LOGE(
                TAG,
                "Failed to create mutex");

            return false;
        }
    }

    //-----------------------------------------
    // Initialize sensor
    //-----------------------------------------

    if (!m_ultrasonicSensor.initialize())
    {
        handleFault(
            "Ultrasonic sensor init failed");

        return false;
    }

    //-----------------------------------------
    // Initialize motor controller
    //-----------------------------------------

    if (!m_motorController.initialize())
    {
        handleFault(
            "Motor controller init failed");

        return false;
    }

    //-----------------------------------------
    // Reset runtime memory
    //-----------------------------------------

    m_memory = {};

    //-----------------------------------------
    // Runtime timestamps
    //-----------------------------------------

    const uint32_t timestampMs =
        getCurrentTimestampMs();

    m_memory.runtimeStartTimestampMs =
        timestampMs;

    m_memory.lastHealthyRuntimeTimestampMs =
        timestampMs;

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    m_initialized = true;

    m_memory.initialized = true;

    transitionToState(
        RobotState::Active);

    ESP_LOGI(
        TAG,
        "RobotController initialized");

    return true;
}

//====================================================
// Shutdown
//====================================================

void RobotController::shutdown()
{
    //-----------------------------------------
    // Stop runtime. NEVER LOCK AROUND CALLS THAT INTERNALLY LOCK
    //-----------------------------------------
    stop();

    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Shutdown motor controller
    //-----------------------------------------

    m_motorController.shutdown();

    //-----------------------------------------
    // Shutdown sensor
    //-----------------------------------------

    m_ultrasonicSensor.shutdown();

    //-----------------------------------------
    // Destroy tasks
    //-----------------------------------------

    destroyTasks();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    m_initialized = false;

    m_running = false;

    transitionToState(
        RobotState::Shutdown);

    ESP_LOGI(
        TAG,
        "RobotController shutdown");
}

//====================================================
// Start runtime
//====================================================

bool RobotController::start()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Must be initialized
    //-----------------------------------------

    if (!m_initialized)
    {
        return false;
    }

    //-----------------------------------------
    // Already running
    //-----------------------------------------

    if (m_running)
    {
        return true;
    }

    //-----------------------------------------
    // Create RTOS tasks
    //-----------------------------------------

    if (!createTasks())
    {
        handleFault(
            "Task creation failed");

        return false;
    }

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    m_running = true;

    m_memory.runtimeActive = true;

    transitionToState(
        RobotState::Active);

    ESP_LOGI(
        TAG,
        "RobotController started");

    return true;
}

//====================================================
// Stop runtime
//====================================================

void RobotController::stop()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Already stopped
    //-----------------------------------------

    if (!m_running)
    {
        return;
    }

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    m_running = false;

    m_memory.runtimeActive = false;

    //-----------------------------------------
    // Stop locomotion
    //-----------------------------------------

    m_motorController.stop();

    //-----------------------------------------
    // Destroy tasks
    //-----------------------------------------

    destroyTasks();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        RobotState::Paused);

    ESP_LOGI(
        TAG,
        "RobotController stopped");
}

//====================================================
// Runtime update
//====================================================

void RobotController::update()
{
    const uint32_t currentTimestampMs =
        getCurrentTimestampMs();

    executePipeline(
        currentTimestampMs);
}

//====================================================
// Main robotics pipeline
//====================================================

void RobotController::executePipeline(
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Begin timing
    //-----------------------------------------

    beginPipelineTiming(
        currentTimestampMs);

    //-----------------------------------------
    // Runtime statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .totalControlLoops++;

    //-----------------------------------------
    // Sensor stage
    //-----------------------------------------

    executeSensorStage(
        currentTimestampMs);

    //-----------------------------------------
    // Filter stage
    //-----------------------------------------

    executeFilterStage(
        currentTimestampMs);

    //-----------------------------------------
    // Obstacle stage
    //-----------------------------------------

    executeObstacleStage(
        currentTimestampMs);

    //-----------------------------------------
    // Navigation stage
    //-----------------------------------------

    executeNavigationStage(
        currentTimestampMs);

    //-----------------------------------------
    // Motion stage
    //-----------------------------------------

    executeMotionStage(
        currentTimestampMs);

    //-----------------------------------------
    // Motor stage
    //-----------------------------------------

    executeMotorStage(
        currentTimestampMs);

    //-----------------------------------------
    // Monitoring stage
    //-----------------------------------------

    executeMonitoringStage(
        currentTimestampMs);

    //-----------------------------------------
    // Runtime memory
    //-----------------------------------------

    updateRuntimeMemory(
        currentTimestampMs);

    //-----------------------------------------
    // End timing
    //-----------------------------------------

    endPipelineTiming(
        currentTimestampMs);

    //-----------------------------------------
    // Success statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .successfulControlLoops++;
}

//====================================================
// Sensor stage
//====================================================
void RobotController::executeSensorStage(uint32_t currentTimestampMs)
{
    const uint64_t startUs = esp_timer_get_time();

    // Fetch latest sensor frame
    UltrasonicSensorData latestData;

    const bool received = m_ultrasonicSensor.fetchLatestData(latestData);

    // Fresh data available
    if (received)
    {
        m_latestSensorData = latestData;

        m_memory.sensorDataAvailable = true;

        m_memory.runtimeStatistics.sensorUpdates++;

        // Healthy timestamp
        m_memory.lastHealthyRuntimeTimestampMs = currentTimestampMs;
    }
    else
    {
        // No fresh perception frame
        m_memory.sensorDataAvailable = false;

        // Sensor timeout supervision
        const uint32_t elapsedMs = currentTimestampMs - m_latestSensorData.timestampMs;

        if (elapsedMs > m_config.sensorTimeoutMs)
        {
            m_memory.runtimeFlags.sensorTimeoutActive = true;

            m_memory.systemHealth.sensorHealthy = false;

            m_memory.runtimeStatistics.sensorTimeouts++;
        }
    }

    // Stage timing
    m_memory.pipelineTiming.sensorStageDurationUs = static_cast<uint32_t>(esp_timer_get_time() - startUs);
}

//====================================================
// Filter stage
//====================================================
void RobotController::executeFilterStage(uint32_t currentTimestampMs)
{
    const uint64_t startUs = esp_timer_get_time();

    // Sensor data available
    if (!m_memory.sensorDataAvailable)
    {
        return;
    }

    // Filter perception
    m_latestFilteredData =
        m_ultrasonicFilter.process(
            m_latestSensorData.pulseWidthUs,
            m_latestSensorData.timestampMs);

    // Timing
    m_memory.pipelineTiming
        .filterStageDurationUs =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            startUs);

    (void)currentTimestampMs;
}

//====================================================
// Obstacle stage
//====================================================
void RobotController::executeObstacleStage(
    uint32_t currentTimestampMs)
{
    const uint64_t startUs =
        esp_timer_get_time();

    // Validate filtered perception
    if (!m_memory.sensorDataAvailable)
    {
        return;
    }

    // Environment interpretation
    m_latestObstacleAnalysis =
        m_obstacleManager.process(
            m_latestFilteredData,
            currentTimestampMs);

    // Statistics
    m_memory.runtimeStatistics
        .obstacleAnalyses++;

    // Timing
    m_memory.pipelineTiming
        .obstacleStageDurationUs =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            startUs);
}

//====================================================
// Navigation stage
//====================================================

void RobotController::executeNavigationStage(
    uint32_t currentTimestampMs)
{
    const uint64_t startUs =
        esp_timer_get_time();

    //-----------------------------------------
    // Navigation decision
    //-----------------------------------------

    m_latestNavigationDecision =
        m_navigationManager.process(
            m_latestObstacleAnalysis,
            currentTimestampMs);

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .navigationDecisions++;

    //-----------------------------------------
    // Timing
    //-----------------------------------------

    m_memory.pipelineTiming
        .navigationStageDurationUs =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            startUs);
}

//====================================================
// Motion stage
//====================================================

void RobotController::executeMotionStage(
    uint32_t currentTimestampMs)
{
    const uint64_t startUs =
        esp_timer_get_time();

    //-----------------------------------------
    // Motion planning
    //-----------------------------------------

    m_latestMotionCommand =
        m_motionPlanner.process(
            m_latestNavigationDecision,
            currentTimestampMs);

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .motionPlans++;

    //-----------------------------------------
    // Timing
    //-----------------------------------------

    m_memory.pipelineTiming
        .motionStageDurationUs =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            startUs);
}

//====================================================
// Motor stage
//====================================================

void RobotController::executeMotorStage(
    uint32_t currentTimestampMs)
{
    if (m_memory.runtimeFlags.emergencyActive)
    {
        return;
    }

    const uint64_t startUs =
        esp_timer_get_time();

    //-----------------------------------------
    // Execute locomotion
    //-----------------------------------------

    m_motorController.executeMotion(
        m_latestMotionCommand);

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .motorExecutions++;

    //-----------------------------------------
    // Timing
    //-----------------------------------------

    m_memory.pipelineTiming
        .motorStageDurationUs =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            startUs);

    (void)currentTimestampMs;
}

//====================================================
// Monitoring stage
//====================================================

void RobotController::executeMonitoringStage(
    uint32_t currentTimestampMs)
{
    performHealthMonitoring(
        currentTimestampMs);

    performTimingSupervision(
        currentTimestampMs);

    performEmergencySupervision(
        currentTimestampMs);

    performFaultMonitoring(
        currentTimestampMs);
}

//====================================================
// Health monitoring
//====================================================

void RobotController::performHealthMonitoring(
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Sensor health
    //-----------------------------------------

    m_memory.systemHealth
        .sensorHealthy =
        !m_latestFilteredData
             .timeoutOccurred;

    //-----------------------------------------
    // Motor health
    //-----------------------------------------

    m_memory.systemHealth
        .motorHealthy =
        !m_motorController.hasFault();

    //-----------------------------------------
    // Driver health
    //-----------------------------------------

    m_memory.systemHealth
        .driverHealthy =
        !m_motorController.hasFault();

    //-----------------------------------------
    // Global health
    //-----------------------------------------

    m_memory.systemHealth
        .systemHealthy =
        (m_memory.systemHealth
             .sensorHealthy &&
         m_memory.systemHealth
             .motorHealthy &&
         m_memory.systemHealth
             .driverHealthy);

    //-----------------------------------------
    // Healthy timestamp
    //-----------------------------------------

    if (
        m_memory.systemHealth
            .systemHealthy)
    {
        m_memory
            .lastHealthyRuntimeTimestampMs =
            currentTimestampMs;
    }
}

//====================================================
// Timing supervision
//====================================================

void RobotController::performTimingSupervision(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    const uint32_t loopDurationUs =
        m_memory.pipelineTiming
            .controlLoopDurationUs;

    //-----------------------------------------
    // Timing violation
    //-----------------------------------------

    if (
        hasPipelineTimingViolation(
            loopDurationUs))
    {
        m_memory.runtimeStatistics
            .timingViolations++;

        m_memory.systemHealth
            .timingHealthy = false;
    }
    else
    {
        m_memory.systemHealth
            .timingHealthy = true;
    }
}

//====================================================
// Emergency supervision
//====================================================

void RobotController::performEmergencySupervision(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    //-----------------------------------------
    // Obstacle emergency
    //-----------------------------------------

    if (
        m_latestObstacleAnalysis
            .emergencyDetected)
    {
        triggerEmergency(
            EmergencySeverity::Critical,
            EmergencySource::Obstacle,
            EmergencyCode::
                ObstacleCollisionRisk);
    }
}

//====================================================
// Fault monitoring
//====================================================

void RobotController::performFaultMonitoring(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    //-----------------------------------------
    // Motor controller fault
    //-----------------------------------------

    if (m_motorController.hasFault())
    {
        handleFault(
            "Motor controller fault");
    }
}

//====================================================
// Emergency stop
//====================================================

void RobotController::emergencyStop()
{
    ScopedControllerLock lock(this);

    triggerEmergency(
        EmergencySeverity::Critical,
        EmergencySource::Runtime,
        EmergencyCode::
            EmergencyStopTriggered);
}

//====================================================
// Trigger emergency
//====================================================

void RobotController::triggerEmergency(
    EmergencySeverity severity,
    EmergencySource source,
    EmergencyCode code)
{
    if (m_memory.emergencyState.emergencyActive)
    {
        return;
    }

    //-----------------------------------------
    // Emergency state
    //-----------------------------------------

    m_memory.emergencyState
        .emergencyActive = true;

    m_memory.emergencyState
        .severity = severity;

    m_memory.emergencyState
        .source = source;

    m_memory.emergencyState
        .code = code;

    m_memory.emergencyState
        .emergencyStopTriggered =
        true;

    m_memory.emergencyState
        .lastEmergencyTimestampMs =
        getCurrentTimestampMs();

    //-----------------------------------------
    // Runtime flags
    //-----------------------------------------

    m_memory.runtimeFlags
        .emergencyActive = true;

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .emergencyStops++;

    //-----------------------------------------
    // Stop locomotion
    //-----------------------------------------

    m_motorController
        .emergencyStop();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        RobotState::Emergency);

    ESP_LOGE(
        TAG,
        "Emergency triggered");
}

//====================================================
// Clear emergency
//====================================================

void RobotController::clearEmergency()
{
    ScopedControllerLock lock(this);

    clearEmergencyInternal();
}

//====================================================
// Internal emergency clear
//====================================================

void RobotController::clearEmergencyInternal()
{
    //-----------------------------------------
    // Clear runtime emergency
    //-----------------------------------------

    m_memory.emergencyState =
        {};

    //-----------------------------------------
    // Runtime flags
    //-----------------------------------------

    m_memory.runtimeFlags
        .emergencyActive = false;

    //-----------------------------------------
    // Clear motor controller emergency
    //-----------------------------------------

    m_motorController
        .clearEmergencyStop();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        RobotState::Active);

    ESP_LOGI(
        TAG,
        "Emergency cleared");
}

//====================================================
// State transition
//====================================================

void RobotController::transitionToState(
    RobotState newState)
{
    //-----------------------------------------
    // Validate transition
    //-----------------------------------------

    if (
        !validateStateTransition(
            m_memory.currentState,
            newState))
    {
        return;
    }

    //-----------------------------------------
    // Previous state
    //-----------------------------------------

    m_memory.previousState =
        m_memory.currentState;

    //-----------------------------------------
    // Current state
    //-----------------------------------------

    m_memory.currentState =
        newState;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    m_memory.lastStateTransitionTimestampMs =
        getCurrentTimestampMs();
}

//====================================================
// Validate state transition
//====================================================

bool RobotController::validateStateTransition(
    RobotState currentState,
    RobotState newState) const
{
    //-----------------------------------------
    // Fault protection
    //-----------------------------------------

    if (
        currentState ==
            RobotState::Fault &&
        newState !=
            RobotState::Shutdown)
    {
        return false;
    }

    return true;
}

//====================================================
// Fault handling
//====================================================

void RobotController::handleFault(
    const char *reason)
{
    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        RobotState::Fault);

    //-----------------------------------------
    // System health
    //-----------------------------------------

    m_memory.systemHealth
        .faultActive = true;

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.runtimeStatistics
        .totalFaults++;

    //-----------------------------------------
    // Stop locomotion
    //-----------------------------------------

    m_motorController
        .emergencyStop();

    ESP_LOGE(
        TAG,
        "RobotController fault: %s",
        reason);
}

//====================================================
// Begin timing
//====================================================

void RobotController::beginPipelineTiming(
    uint32_t currentTimestampMs)
{
    m_pipelineStartTimestampUs =
        static_cast<uint32_t>(
            esp_timer_get_time());

    m_memory.pipelineTiming
        .lastUpdateTimestampMs =
        currentTimestampMs;
}

//====================================================
// End timing
//====================================================

void RobotController::endPipelineTiming(
    uint32_t currentTimestampMs)
{
    const uint32_t endUs =
        static_cast<uint32_t>(
            esp_timer_get_time());

    const uint32_t durationUs =
        endUs -
        m_pipelineStartTimestampUs;

    //-----------------------------------------
    // Timing
    //-----------------------------------------

    m_memory.pipelineTiming
        .controlLoopDurationUs =
        durationUs;

    //-----------------------------------------
    // Worst case
    //-----------------------------------------

    if (
        durationUs >
        m_memory.pipelineTiming
            .worstCaseLoopDurationUs)
    {
        m_memory.pipelineTiming
            .worstCaseLoopDurationUs =
            durationUs;
    }

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    updatePipelineStatistics(
        durationUs);

    (void)currentTimestampMs;
}

//====================================================
// Update pipeline statistics
//====================================================

void RobotController::updatePipelineStatistics(
    uint32_t pipelineDurationUs)
{
    auto &timing =
        m_memory.pipelineTiming;

    //-----------------------------------------
    // Running average
    //-----------------------------------------

    timing.averageLoopDurationUs =
        ((
             timing.averageLoopDurationUs *
             (m_pipelineExecutionCounter - 1)) +
         pipelineDurationUs) /
        m_pipelineExecutionCounter;
}

//====================================================
// Timing violation
//====================================================

bool RobotController::hasPipelineTimingViolation(
    uint32_t pipelineDurationUs) const
{
    return (
        pipelineDurationUs >
        m_config
            .maximumPipelineDurationUs);
}

//====================================================
// Create RTOS tasks
//====================================================
bool RobotController::createTasks()
{
    //-----------------------------------------
    // Controller task
    //-----------------------------------------

    BaseType_t result =
        xTaskCreatePinnedToCore(
            controllerTaskEntry,
            "RobotControllerTask",
            m_config.taskConfig
                .controllerTaskStackSize,
            this,
            m_config.taskConfig
                .controllerTaskPriority,
            &m_controllerTaskHandle,
            m_config.taskConfig
                .controllerCore);

    //-----------------------------------------
    // Task creation failed
    //-----------------------------------------

    if (result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create controller task");

        destroyTasks();

        return false;
    }

    //-----------------------------------------
    // Driver task
    //-----------------------------------------

    result =
        xTaskCreatePinnedToCore(
            driverTaskEntry,
            "MotorDriverTask",
            m_config.taskConfig
                .driverTaskStackSize,
            this,
            m_config.taskConfig
                .driverTaskPriority,
            &m_driverTaskHandle,
            m_config.taskConfig
                .driverCore);

    //-----------------------------------------
    // Task creation failed
    //-----------------------------------------

    if (result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create driver task");

        destroyTasks();

        return false;
    }

    //-----------------------------------------
    // Telemetry task
    //-----------------------------------------

    result =
        xTaskCreatePinnedToCore(
            telemetryTaskEntry,
            "TelemetryTask",
            m_config.taskConfig
                .telemetryTaskStackSize,
            this,
            m_config.taskConfig
                .telemetryTaskPriority,
            &m_telemetryTaskHandle,
            m_config.taskConfig
                .telemetryCore);

    //-----------------------------------------
    // Task creation failed
    //-----------------------------------------

    if (result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create telemetry task");

        destroyTasks();

        return false;
    }

    //-----------------------------------------
    // Success
    //-----------------------------------------

    ESP_LOGI(
        TAG,
        "All RTOS tasks created successfully");

    return true;
}

//====================================================
// Destroy tasks
//====================================================

void RobotController::destroyTasks()
{
    if (m_controllerTaskHandle != nullptr)
    {
        vTaskDelete(
            m_controllerTaskHandle);

        m_controllerTaskHandle =
            nullptr;
    }

    if (m_driverTaskHandle != nullptr)
    {
        vTaskDelete(
            m_driverTaskHandle);

        m_driverTaskHandle =
            nullptr;
    }

    if (m_telemetryTaskHandle != nullptr)
    {
        vTaskDelete(
            m_telemetryTaskHandle);

        m_telemetryTaskHandle =
            nullptr;
    }
}

//====================================================
// Controller task entry
//====================================================

void RobotController::controllerTaskEntry(
    void *context)
{
    auto *controller =
        static_cast<RobotController *>(
            context);

    controller->controllerTaskLoop();
}

//====================================================
// Driver task entry
//====================================================

void RobotController::driverTaskEntry(
    void *context)
{
    auto *controller =
        static_cast<RobotController *>(
            context);

    controller->driverTaskLoop();
}

//====================================================
// Telemetry task entry
//====================================================

void RobotController::telemetryTaskEntry(
    void *context)
{
    auto *controller =
        static_cast<RobotController *>(
            context);

    controller->telemetryTaskLoop();
}

//====================================================
// Controller task loop
//====================================================
void RobotController::controllerTaskLoop()
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    const TickType_t frequency =
        pdMS_TO_TICKS(
            1000 /
            m_config.taskConfig
                .controllerHz);

    while (m_running)
    {
        update();

        vTaskDelayUntil(
            &previousWakeTime,
            frequency);
    }
}

//====================================================
// Driver task loop
//====================================================

void RobotController::driverTaskLoop()
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    const TickType_t frequency =
        pdMS_TO_TICKS(
            1000 /
            m_config.taskConfig
                .driverHz);

    while (m_running)
    {
        m_motorController.update(
            getCurrentTimestampMs());

        vTaskDelayUntil(
            &previousWakeTime,
            frequency);
    }
}

//====================================================
// Telemetry task loop
//====================================================

void RobotController::telemetryTaskLoop()
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    const TickType_t frequency =
        pdMS_TO_TICKS(
            1000 /
            m_config.taskConfig
                .telemetryHz);

    while (m_running)
    {
        //-----------------------------------------
        // Future telemetry
        //-----------------------------------------

        vTaskDelayUntil(
            &previousWakeTime,
            frequency);
    }
}

//====================================================
// Lock
//====================================================

void RobotController::lockController()
{
    if (m_controllerMutex != nullptr)
    {
        xSemaphoreTake(
            m_controllerMutex,
            portMAX_DELAY);
    }
}

//====================================================
// Unlock
//====================================================

void RobotController::unlockController()
{
    if (m_controllerMutex != nullptr)
    {
        xSemaphoreGive(
            m_controllerMutex);
    }
}

//====================================================
// Runtime memory
//====================================================

const RobotControllerMemory &
RobotController::getMemory() const
{
    return m_memory;
}

//====================================================
// System health
//====================================================

const SystemHealth &
RobotController::getSystemHealth() const
{
    return m_memory.systemHealth;
}

//====================================================
// Runtime state
//====================================================

RobotState
RobotController::getCurrentState() const
{
    return m_memory.currentState;
}

//====================================================
// Runtime active
//====================================================

bool RobotController::isRunning() const
{
    return m_running;
}

//====================================================
// Emergency active
//====================================================

bool RobotController::isEmergencyActive() const
{
    return m_memory
        .emergencyState
        .emergencyActive;
}

//====================================================
// Fault active
//====================================================

bool RobotController::hasFault() const
{
    return m_memory
        .systemHealth
        .faultActive;
}

//====================================================
// Behavior mode
//====================================================

RobotBehaviorMode
RobotController::getBehaviorMode() const
{
    return m_memory.behaviorMode;
}

//====================================================
// Set behavior mode
//====================================================

void RobotController::setBehaviorMode(
    RobotBehaviorMode mode)
{
    m_memory.behaviorMode =
        mode;
}

//====================================================
// Runtime memory update
//====================================================

void RobotController::updateRuntimeMemory(
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Runtime timestamp
    //-----------------------------------------

    m_memory.lastRuntimeUpdateTimestampMs =
        currentTimestampMs;

    //-----------------------------------------
    // Pipeline timestamp
    //-----------------------------------------

    m_memory.lastPipelineTimestampMs =
        currentTimestampMs;

    //-----------------------------------------
    // Runtime duration
    //-----------------------------------------

    m_memory.totalRuntimeDurationMs =
        currentTimestampMs -
        m_memory.runtimeStartTimestampMs;

    //-----------------------------------------
    // Pipeline execution counter
    //-----------------------------------------

    m_pipelineExecutionCounter++;
}

//====================================================
// Timestamp utility
//====================================================

uint32_t RobotController::getCurrentTimestampMs() const
{
    return static_cast<uint32_t>(
        esp_timer_get_time() / 1000ULL);
}