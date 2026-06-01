//====================================================
// File: RobotController.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "filter/include/UltrasonicFilter.hpp"
#include "obstacle/include/ObstacleAnalysis.hpp"
#include "obstacle/include/ObstacleManager.hpp"
#include "navigation/include/NavigationDecision.hpp"
#include "navigation/include/NavigationManager.hpp"
#include "motion/include/MotionCommand.hpp"
#include "motion/include/MotionPlanner.hpp"

#include "interfaces/include/sensor/IUltrasonicSensor.hpp"
#include "interfaces/include/logging/ILogger.hpp"
#include "interfaces/include/synchronization/IMutex.hpp"
#include "interfaces/include/timing/ITimer.hpp"

#include "motor/controller/include/MotorController.hpp"

#include "EmergencyState.hpp"
#include "PipelineTiming.hpp"
#include "RobotBehaviorMode.hpp"
#include "RobotControllerConfig.hpp"
#include "RobotControllerMemory.hpp"
#include "RobotRuntimeFlags.hpp"
#include "RobotState.hpp"
#include "RuntimeStatistics.hpp"
#include "SystemHealth.hpp"

//====================================================
// RobotController
//====================================================
//
// Top-level robotics orchestration engine.
//
// RESPONSIBILITIES
//
//  - Central Runtime Orchestrator
//  - Pipeline Coordinator
//  - Deterministic Execution Engine
//  - System State Management
//  - Fault Propagation
//  - Emergency Coordination
//  - Health Monitoring
//  - Runtime Timing Management
//  - Future Expansion Point
//
// ROBOTICS PIPELINE
//
//  Read sensors
//      ↓
//  Filter perception
//      ↓
//  Interpret environment
//      ↓
//  Decide navigation
//      ↓
//  Plan locomotion
//      ↓
//  Execute motors
//      ↓
//  Monitor runtime
//
// IMPORTANT
//
//  RobotController MUST:
//      - remain deterministic
//      - avoid blocking
//      - avoid heavy allocations
//      - avoid slow IO
//
//====================================================

class RobotController final
{
public:
    RobotController(
        IUltrasonicSensor &ultrasonicSensor,
        UltrasonicFilter &ultrasonicFilter,
        ObstacleManager &obstacleManager,
        NavigationManager &navigationManager,
        MotionPlanner &motionPlanner,
        MotorController &motorController,
        IMutex &mutex,
        ILogger &logger,
        ITimer &timer,
        const RobotControllerConfig &config);

    // Destructor
    ~RobotController();

    bool initialize();

    void shutdown();

    // Start runtime
    bool start();

    // Stop runtime
    void stop();

    // Runtime update
    void update();

    // Emergency stop
    void emergencyStop();

    // Clear emergency state
    void clearEmergency();

    // Reset runtime
    void reset();

    // Runtime state
    RobotState getCurrentState() const;

    // Runtime health
    const SystemHealth &getSystemHealth() const;

    // Runtime memory
    const RobotControllerMemory &getMemory() const;

    // Runtime behavior mode
    RobotBehaviorMode getBehaviorMode() const;

    // Set runtime behavior mode
    void setBehaviorMode(RobotBehaviorMode mode);

    // Runtime active
    bool isRunning() const;

    // Emergency active
    bool isEmergencyActive() const;

    // Fault active
    bool hasFault() const;

private:
    // Main deterministic robotics pipeline
    void executePipeline(uint32_t currentTimestampMs);

    //================================================
    // Pipeline stages
    //================================================

    void executeSensorStage(uint32_t currentTimestampMs);

    void executeFilterStage(uint32_t currentTimestampMs);

    void executeObstacleStage(uint32_t currentTimestampMs);

    void executeNavigationStage(uint32_t currentTimestampMs);

    void executeMotionStage(uint32_t currentTimestampMs);

    void executeMotorStage(uint32_t currentTimestampMs);

    void executeMonitoringStage(uint32_t currentTimestampMs);

    //================================================
    // Runtime supervision
    //================================================

    void performHealthMonitoring(uint32_t currentTimestampMs);

    void performTimingSupervision(uint32_t currentTimestampMs);

    void performEmergencySupervision(uint32_t currentTimestampMs);

    void performFaultMonitoring(uint32_t currentTimestampMs);

    //================================================
    // State management
    //================================================

    void transitionToState(RobotState newState);

    bool validateStateTransition(RobotState currentState, RobotState newState) const;

    //================================================
    // Emergency handling
    //================================================

    void triggerEmergency(EmergencySeverity severity, EmergencySource source, EmergencyCode code);

    void clearEmergencyInternal();

    bool shouldTriggerEmergency() const;

    //================================================
    // Fault handling
    //================================================

    void handleFault(const char *reason);

    void recoverFromFault();

    //================================================
    // Runtime timing
    //================================================

    void beginPipelineTiming(
        uint32_t currentTimestampMs);

    void endPipelineTiming(uint32_t currentTimestampMs);

    void updatePipelineStatistics(uint32_t pipelineDurationUs);

    bool hasPipelineTimingViolation(uint32_t pipelineDurationUs) const;

    //================================================
    // Runtime validation
    //================================================

    bool validateRuntimeHealth() const;

    bool validateSubsystemHealth() const;

    bool validateTimingHealth() const;

    bool validateEmergencyState() const;

    //================================================
    // Runtime memory update
    //================================================

    void updateRuntimeMemory(uint32_t currentTimestampMs);

    void updateRuntimeStatistics();

    void updateSystemHealth();

    //================================================
    // Timestamp utility
    //================================================

    uint32_t getCurrentTimestampMs() const;

    uint32_t getCurrentTimestampUs() const;

private:
    // Sensor layer
    IUltrasonicSensor &m_ultrasonicSensor;

    // Filter layer
    UltrasonicFilter &m_ultrasonicFilter;

    // Environment layer
    ObstacleManager &m_obstacleManager;

    // Navigation layer
    NavigationManager &m_navigationManager;

    // Motion layer
    MotionPlanner &m_motionPlanner;

    // Locomotion layer
    MotorController &m_motorController;

    // Platform services
    IMutex &m_mutex;
    ILogger &m_logger;
    ITimer &m_timer;

    // Configuration
    RobotControllerConfig m_config;

    // Runtime memory
    RobotControllerMemory m_memory;

    //================================================
    // Pipeline runtime data
    //================================================
    UltrasonicSensorData m_latestSensorData;
    FilteredSensorData m_latestFilteredData;
    ObstacleAnalysis m_latestObstacleAnalysis;
    NavigationDecision m_latestNavigationDecision;
    MotionCommand m_latestMotionCommand;

    //================================================
    // Runtime state
    //================================================
    bool m_running = false;
    bool m_initialized = false;

    //================================================
    // Pipeline timing
    //================================================
    uint32_t m_pipelineStartTimestampUs = 0;

    //================================================
    // Runtime sequence tracking
    //================================================
    uint64_t m_pipelineExecutionCounter = 0;
};
