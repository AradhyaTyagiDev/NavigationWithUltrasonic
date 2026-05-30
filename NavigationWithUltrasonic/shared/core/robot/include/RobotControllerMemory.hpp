//====================================================
// File: RobotControllerMemory.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "EmergencyState.hpp"
#include "PipelineTiming.hpp"
#include "RobotBehaviorMode.hpp"
#include "RobotRuntimeFlags.hpp"
#include "RobotState.hpp"
#include "RuntimeStatistics.hpp"
#include "SystemHealth.hpp"

//====================================================
// RobotControllerMemory
//====================================================
//
// Global robotics runtime memory.
//
// Tracks:
//      robot runtime state
//      pipeline execution
//      emergency runtime
//      fault runtime
//      timing state
//      runtime statistics
//
//====================================================

struct RobotControllerMemory
{
    // Current robot runtime state
    RobotState currentState = RobotState::Booting;

    // Previous robot state
    RobotState previousState = RobotState::Booting;

    // Current behavior mode
    RobotBehaviorMode behaviorMode = RobotBehaviorMode::Balanced;

    // Runtime flags
    RobotRuntimeFlags runtimeFlags;

    // Global system health
    SystemHealth systemHealth;

    // Emergency runtime state
    EmergencyState emergencyState;

    // Runtime statistics
    RuntimeStatistics runtimeStatistics;

    // Pipeline timing
    PipelineTiming pipelineTiming;

    // Runtime active
    bool runtimeActive = false;

    // Robot initialized
    bool initialized = false;

    // Sensor data available
    bool sensorDataAvailable = false;

    // Pipeline execution active
    bool pipelineExecutionActive = false;

    // Last successful pipeline execution
    uint32_t lastPipelineTimestampMs = 0;

    // Last runtime update
    uint32_t lastRuntimeUpdateTimestampMs = 0;

    // Last state transition
    uint32_t lastStateTransitionTimestampMs = 0;

    // Last healthy runtime timestamp
    uint32_t lastHealthyRuntimeTimestampMs = 0;

    // Total runtime duration
    uint64_t totalRuntimeDurationMs = 0;

    // Runtime start timestamp
    uint32_t runtimeStartTimestampMs = 0;

    // Consecutive pipeline failures
    uint32_t consecutivePipelineFailures = 0;

    // Consecutive missed cycles
    uint32_t consecutiveMissedCycles = 0;
};