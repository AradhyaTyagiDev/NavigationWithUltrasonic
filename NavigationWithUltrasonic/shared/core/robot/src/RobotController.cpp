
#include "robot/include/RobotController.hpp"

#include "interfaces/include/logging/LoggerExtensions.hpp"
#include "interfaces/include/synchronization/LockGuard.hpp"

static const char *TAG =
    "RobotController";

RobotController::RobotController(
    IUltrasonicSensor &ultrasonicSensor,
    UltrasonicFilter &ultrasonicFilter,
    ObstacleManager &obstacleManager,
    NavigationManager &navigationManager,
    MotionPlanner &motionPlanner,
    MotorController &motorController,
    IMutex &mutex,
    ILogger &logger,
    ITimer &timer,
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

      m_mutex(
          mutex),

      m_logger(
          logger),

      m_timer(
          timer),

      m_config(config)
{
}

RobotController::~RobotController()
{
    shutdown();
}

bool RobotController::initialize()
{
    LockGuard lock(m_mutex);

    if (m_initialized)
    {
        return true;
    }

    transitionToState(
        RobotState::Initializing);

    if (!m_ultrasonicSensor.initialize())
    {
        handleFault(
            "Ultrasonic sensor init failed");

        return false;
    }

    if (!m_motorController.initialize())
    {
        handleFault(
            "Motor controller init failed");

        return false;
    }

    m_memory = {};

    m_memory.behaviorMode =
        m_config.behaviorMode;

    const uint32_t timestampMs =
        getCurrentTimestampMs();

    m_memory.runtimeStartTimestampMs =
        timestampMs;

    m_memory.lastHealthyRuntimeTimestampMs =
        timestampMs;

    m_initialized = true;

    m_memory.initialized = true;

    transitionToState(
        RobotState::Active);

    Logger::info(
        m_logger,
        TAG,
        "RobotController initialized");

    return true;
}

void RobotController::shutdown()
{
    stop();

    LockGuard lock(m_mutex);

    m_motorController.shutdown();

    m_ultrasonicSensor.shutdown();

    m_initialized = false;

    m_running = false;

    transitionToState(
        RobotState::Shutdown);

    Logger::info(
        m_logger,
        TAG,
        "RobotController shutdown");
}

bool RobotController::start()
{
    LockGuard lock(m_mutex);

    if (!m_initialized)
    {
        return false;
    }

    if (m_running)
    {
        return true;
    }

    if (!m_ultrasonicSensor.start())
    {
        handleFault(
            "Ultrasonic sensor start failed");

        return false;
    }

    m_running = true;

    m_memory.runtimeActive = true;

    transitionToState(
        RobotState::Active);

    Logger::info(
        m_logger,
        TAG,
        "RobotController started");

    return true;
}

void RobotController::stop()
{
    LockGuard lock(m_mutex);

    if (!m_running)
    {
        return;
    }

    m_running = false;

    m_memory.runtimeActive = false;

    m_motorController.stop();

    transitionToState(
        RobotState::Paused);

    Logger::info(
        m_logger,
        TAG,
        "RobotController stopped");
}

void RobotController::update()
{
    if (!m_mutex.tryLock())
    {
        return;
    }

    if (!m_initialized || !m_running)
    {
        m_mutex.unlock();

        return;
    }

    const uint32_t currentTimestampMs =
        getCurrentTimestampMs();

    executePipeline(
        currentTimestampMs);

    m_mutex.unlock();
}

void RobotController::executePipeline(
    uint32_t currentTimestampMs)
{

    beginPipelineTiming(
        currentTimestampMs);

    m_memory.runtimeStatistics
        .totalControlLoops++;

    executeSensorStage(
        currentTimestampMs);

    executeFilterStage(
        currentTimestampMs);

    executeObstacleStage(
        currentTimestampMs);

    executeNavigationStage(
        currentTimestampMs);

    executeMotionStage(
        currentTimestampMs);

    executeMotorStage(
        currentTimestampMs);

    executeMonitoringStage(
        currentTimestampMs);

    updateRuntimeMemory(
        currentTimestampMs);

    endPipelineTiming(
        currentTimestampMs);

    m_memory.runtimeStatistics
        .successfulControlLoops++;
}

void RobotController::executeSensorStage(uint32_t currentTimestampMs)
{
    const uint32_t startUs = getCurrentTimestampUs();

    UltrasonicSensorData latestData;

    const bool received = m_ultrasonicSensor.fetchLatestData(latestData);

    if (received)
    {
        m_latestSensorData = latestData;

        m_memory.sensorDataAvailable = true;

        m_memory.runtimeStatistics.sensorUpdates++;

        m_memory.runtimeFlags.sensorTimeoutActive = false;

        m_memory.lastHealthyRuntimeTimestampMs = currentTimestampMs;
    }
    else
    {
        m_memory.sensorDataAvailable = false;

        const uint32_t elapsedMs = currentTimestampMs - m_latestSensorData.timestampMs;

        if (elapsedMs > m_config.sensorTimeoutMs)
        {
            m_memory.runtimeFlags.sensorTimeoutActive = true;

            m_memory.systemHealth.sensorHealthy = false;

            m_memory.runtimeStatistics.sensorTimeouts++;
        }
    }

    m_memory.pipelineTiming.sensorStageDurationUs =
        getCurrentTimestampUs() -
        startUs;
}

void RobotController::executeFilterStage(uint32_t currentTimestampMs)
{
    const uint32_t startUs = getCurrentTimestampUs();

    if (!m_memory.sensorDataAvailable)
    {
        return;
    }

    m_latestFilteredData =
        m_ultrasonicFilter.process(
            m_latestSensorData.pulseWidthUs,
            m_latestSensorData.timestampMs);

    m_memory.pipelineTiming
        .filterStageDurationUs =
        getCurrentTimestampUs() -
        startUs;

    (void)currentTimestampMs;
}

void RobotController::executeObstacleStage(
    uint32_t currentTimestampMs)
{
    const uint64_t startUs =
        getCurrentTimestampUs();

    if (!m_memory.sensorDataAvailable)
    {
        return;
    }

    m_latestObstacleAnalysis =
        m_obstacleManager.process(
            m_latestFilteredData,
            currentTimestampMs);

    m_memory.runtimeStatistics
        .obstacleAnalyses++;

    m_memory.pipelineTiming
        .obstacleStageDurationUs =
        getCurrentTimestampUs() -
        startUs;
}

void RobotController::executeNavigationStage(
    uint32_t currentTimestampMs)
{
    const uint64_t startUs =
        getCurrentTimestampUs();

    m_latestNavigationDecision =
        m_navigationManager.process(
            m_latestObstacleAnalysis,
            currentTimestampMs);

    m_memory.runtimeStatistics
        .navigationDecisions++;

    m_memory.pipelineTiming
        .navigationStageDurationUs =
        getCurrentTimestampUs() -
        startUs;
}

void RobotController::executeMotionStage(
    uint32_t currentTimestampMs)
{
    const uint64_t startUs =
        getCurrentTimestampUs();

    m_latestMotionCommand =
        m_motionPlanner.process(
            m_latestNavigationDecision,
            currentTimestampMs);

    m_memory.runtimeStatistics
        .motionPlans++;

    m_memory.pipelineTiming
        .motionStageDurationUs =
        getCurrentTimestampUs() -
        startUs;
}

void RobotController::executeMotorStage(
    uint32_t currentTimestampMs)
{
    if (m_memory.runtimeFlags.emergencyActive)
    {
        return;
    }

    const uint64_t startUs =
        getCurrentTimestampUs();

    const bool submitted =
        m_motorController.tryExecuteMotion(
            m_latestMotionCommand);

    if (!submitted)
    {
        m_memory.runtimeStatistics
            .missedControlCycles++;

        m_memory.consecutiveMissedCycles++;

        m_memory.pipelineTiming
            .motorStageDurationUs =
            getCurrentTimestampUs() -
            startUs;

        return;
    }

    m_memory.runtimeStatistics
        .motorExecutions++;

    m_memory.consecutiveMissedCycles = 0;

    m_memory.pipelineTiming
        .motorStageDurationUs =
        getCurrentTimestampUs() -
        startUs;

    (void)currentTimestampMs;
}

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

    updateSystemHealth();

    updateRuntimeStatistics();
}

void RobotController::performHealthMonitoring(
    uint32_t currentTimestampMs)
{

    m_memory.systemHealth
        .sensorHealthy =
        !m_memory.runtimeFlags.sensorTimeoutActive &&
        !m_latestFilteredData.timeoutOccurred;

    m_memory.systemHealth
        .motorHealthy =
        !m_motorController.hasFault();

    m_memory.systemHealth
        .driverHealthy =
        !m_motorController.hasFault();

    m_memory.systemHealth
        .systemHealthy =
        (m_memory.systemHealth
             .sensorHealthy &&
         m_memory.systemHealth
             .motorHealthy &&
         m_memory.systemHealth
             .driverHealthy);

    if (
        m_memory.systemHealth
            .systemHealthy)
    {
        m_memory
            .lastHealthyRuntimeTimestampMs =
            currentTimestampMs;
    }
}

void RobotController::performTimingSupervision(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    const uint32_t loopDurationUs =
        m_memory.pipelineTiming
            .controlLoopDurationUs;

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

void RobotController::performEmergencySupervision(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    if (!shouldTriggerEmergency())
    {
        return;
    }

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
    else if (m_motorController.isEmergencyStopActive())
    {
        triggerEmergency(
            EmergencySeverity::Critical,
            EmergencySource::Runtime,
            EmergencyCode::
                EmergencyStopTriggered);
    }
}

void RobotController::performFaultMonitoring(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    if (m_memory.systemHealth.faultActive)
    {
        recoverFromFault();
    }

    if (m_motorController.hasFault())
    {
        handleFault(
            "Motor controller fault");
    }
}

void RobotController::emergencyStop()
{
    LockGuard lock(m_mutex);

    triggerEmergency(
        EmergencySeverity::Critical,
        EmergencySource::Runtime,
        EmergencyCode::
            EmergencyStopTriggered);
}

void RobotController::triggerEmergency(
    EmergencySeverity severity,
    EmergencySource source,
    EmergencyCode code)
{
    if (m_memory.emergencyState.emergencyActive)
    {
        return;
    }

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

    m_memory.runtimeFlags
        .emergencyActive = true;

    m_memory.systemHealth
        .emergencyActive = true;

    m_memory.systemHealth
        .systemHealthy = false;

    m_memory.runtimeStatistics
        .emergencyStops++;

    m_motorController
        .emergencyStop();

    transitionToState(
        RobotState::Emergency);

    Logger::error(
        m_logger,
        TAG,
        "Emergency triggered");
}

void RobotController::clearEmergency()
{
    LockGuard lock(m_mutex);

    clearEmergencyInternal();
}

void RobotController::clearEmergencyInternal()
{

    m_memory.emergencyState =
        {};

    m_memory.runtimeFlags
        .emergencyActive = false;

    m_memory.systemHealth
        .emergencyActive = false;

    m_motorController
        .clearEmergencyStop();

    transitionToState(
        RobotState::Active);

    Logger::info(
        m_logger,
        TAG,
        "Emergency cleared");
}

void RobotController::transitionToState(
    RobotState newState)
{

    if (
        !validateStateTransition(
            m_memory.currentState,
            newState))
    {
        return;
    }

    m_memory.previousState =
        m_memory.currentState;

    m_memory.currentState =
        newState;

    m_memory.lastStateTransitionTimestampMs =
        getCurrentTimestampMs();
}

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
            RobotState::Shutdown &&
        newState !=
            RobotState::Paused)
    {
        return false;
    }

    return true;
}

void RobotController::handleFault(
    const char *reason)
{
    if (m_memory.systemHealth.faultActive)
    {
        return;
    }

    transitionToState(
        RobotState::Fault);

    m_memory.systemHealth
        .faultActive = true;

    m_memory.systemHealth
        .consecutiveFaultCount++;

    m_memory.systemHealth
        .lastFaultTimestampMs =
        getCurrentTimestampMs();

    m_memory.runtimeStatistics
        .totalFaults++;

    m_motorController
        .emergencyStop();

    Logger::error(
        m_logger,
        TAG,
        "RobotController fault: %s",
        reason);
}

void RobotController::beginPipelineTiming(
    uint32_t currentTimestampMs)
{
    m_pipelineStartTimestampUs =
        getCurrentTimestampUs();

    m_memory.pipelineTiming
        .lastUpdateTimestampMs =
        currentTimestampMs;
}

void RobotController::endPipelineTiming(
    uint32_t currentTimestampMs)
{
    const uint32_t endUs =
        getCurrentTimestampUs();

    const uint32_t durationUs =
        endUs -
        m_pipelineStartTimestampUs;

    m_memory.pipelineTiming
        .controlLoopDurationUs =
        durationUs;

    if (
        durationUs >
        m_memory.pipelineTiming
            .worstCaseLoopDurationUs)
    {
        m_memory.pipelineTiming
            .worstCaseLoopDurationUs =
            durationUs;
    }

    updatePipelineStatistics(
        durationUs);

    (void)currentTimestampMs;
}

void RobotController::updatePipelineStatistics(
    uint32_t pipelineDurationUs)
{
    auto &timing =
        m_memory.pipelineTiming;

    timing.averageLoopDurationUs =
        ((
             timing.averageLoopDurationUs *
             (m_pipelineExecutionCounter - 1)) +
         pipelineDurationUs) /
        m_pipelineExecutionCounter;
}

bool RobotController::hasPipelineTimingViolation(
    uint32_t pipelineDurationUs) const
{
    return (
        pipelineDurationUs >
        m_config
            .maximumPipelineDurationUs);
}

void RobotController::reset()
{
    LockGuard lock(m_mutex);

    m_motorController.reset();

    m_memory = {};

    m_latestSensorData = {};
    m_latestFilteredData = {};
    m_latestObstacleAnalysis = {};
    m_latestNavigationDecision = {};
    m_latestMotionCommand = {};

    m_running = false;
    m_pipelineExecutionCounter = 0;
    m_pipelineStartTimestampUs = 0;

    const uint32_t timestampMs =
        getCurrentTimestampMs();

    m_memory.initialized =
        m_initialized;

    m_memory.behaviorMode =
        m_config.behaviorMode;

    m_memory.runtimeStartTimestampMs =
        timestampMs;

    m_memory.lastHealthyRuntimeTimestampMs =
        timestampMs;

    transitionToState(
        m_initialized
            ? RobotState::Paused
            : RobotState::Booting);

    Logger::info(
        m_logger,
        TAG,
        "RobotController reset");
}

bool RobotController::shouldTriggerEmergency() const
{
    return (
        m_latestObstacleAnalysis.emergencyDetected ||
        m_motorController.isEmergencyStopActive());
}

void RobotController::recoverFromFault()
{
    if (!m_config.enableAutomaticFaultRecovery)
    {
        return;
    }

    const uint32_t currentTimestampMs =
        getCurrentTimestampMs();

    const uint32_t elapsedMs =
        currentTimestampMs -
        m_memory.systemHealth.lastFaultTimestampMs;

    if (elapsedMs < m_config.faultRecoveryCooldownMs)
    {
        return;
    }

    m_memory.systemHealth.faultActive =
        false;

    m_memory.systemHealth.consecutiveFaultCount =
        0;

    m_motorController.reset();

    transitionToState(
        RobotState::Paused);
}

bool RobotController::validateRuntimeHealth() const
{
    return (
        validateSubsystemHealth() &&
        validateTimingHealth() &&
        validateEmergencyState());
}

bool RobotController::validateSubsystemHealth() const
{
    return (
        m_memory.systemHealth.sensorHealthy &&
        !m_motorController.hasFault());
}

bool RobotController::validateTimingHealth() const
{
    return !hasPipelineTimingViolation(
        m_memory.pipelineTiming
            .controlLoopDurationUs);
}

bool RobotController::validateEmergencyState() const
{
    return !m_memory.emergencyState
                .emergencyActive;
}

const RobotControllerMemory &
RobotController::getMemory() const
{
    return m_memory;
}

const SystemHealth &
RobotController::getSystemHealth() const
{
    return m_memory.systemHealth;
}

RobotState
RobotController::getCurrentState() const
{
    return m_memory.currentState;
}

bool RobotController::isRunning() const
{
    return m_running;
}

bool RobotController::isEmergencyActive() const
{
    return m_memory
        .emergencyState
        .emergencyActive;
}

bool RobotController::hasFault() const
{
    return m_memory
        .systemHealth
        .faultActive;
}

RobotBehaviorMode
RobotController::getBehaviorMode() const
{
    return m_memory.behaviorMode;
}

void RobotController::setBehaviorMode(
    RobotBehaviorMode mode)
{
    m_memory.behaviorMode =
        mode;
}

void RobotController::updateRuntimeMemory(
    uint32_t currentTimestampMs)
{

    m_memory.lastRuntimeUpdateTimestampMs =
        currentTimestampMs;

    m_memory.lastPipelineTimestampMs =
        currentTimestampMs;

    m_memory.totalRuntimeDurationMs =
        currentTimestampMs -
        m_memory.runtimeStartTimestampMs;

    m_pipelineExecutionCounter++;
}

void RobotController::updateRuntimeStatistics()
{
    m_memory.runtimeStatistics
        .lastStatisticsUpdateTimestampMs =
        getCurrentTimestampMs();
}

void RobotController::updateSystemHealth()
{
    m_memory.systemHealth
        .sensorHealthy =
        m_ultrasonicSensor.isHealthy() &&
        !m_memory.runtimeFlags.sensorTimeoutActive &&
        !m_latestFilteredData.timeoutOccurred;

    m_memory.systemHealth
        .motorHealthy =
        !m_motorController.hasFault();

    m_memory.systemHealth
        .driverHealthy =
        m_memory.systemHealth
            .motorHealthy;

    m_memory.systemHealth
        .emergencyActive =
        m_memory.emergencyState
            .emergencyActive;

    m_memory.systemHealth
        .timingHealthy =
        validateTimingHealth();

    m_memory.systemHealth
        .systemHealthy =
        validateRuntimeHealth();
}

uint32_t RobotController::getCurrentTimestampMs() const
{
    return Timer::milliseconds(
        m_timer);
}

uint32_t RobotController::getCurrentTimestampUs() const
{
    return static_cast<uint32_t>(
        m_timer.getTimestampUs());
}
