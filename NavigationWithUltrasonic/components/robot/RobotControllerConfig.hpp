//====================================================
// File: RobotControllerConfig.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "RobotBehaviorMode.hpp"
#include "RobotTaskConfig.hpp"

//====================================================
// RobotControllerConfig
//====================================================
//
// Top-level robotics runtime configuration.
//
// Controls:
//      runtime behavior
//      deterministic execution
//      safety thresholds
//      watchdog timing
//      orchestration behavior
//
//====================================================

struct RobotControllerConfig
{
    // Default robot behavior mode
    RobotBehaviorMode behaviorMode = RobotBehaviorMode::Balanced;
    // RTOS task configuration
    RobotTaskConfig taskConfig;
    // Main control loop frequency
    uint32_t controlLoopHz = 50;
    // Main control loop interval
    uint32_t controlLoopIntervalMs = 20;
    // Runtime watchdog timeout
    uint32_t runtimeWatchdogTimeoutMs = 1000;
    // Sensor timeout threshold
    uint32_t sensorTimeoutMs = 250;
    // Motion timeout threshold
    uint32_t motionTimeoutMs = 250;
    // Emergency recovery cooldown
    uint32_t emergencyRecoveryCooldownMs = 1000;
    // Fault recovery cooldown
    uint32_t faultRecoveryCooldownMs = 2000;
    // Maximum allowed pipeline duration
    uint32_t maximumPipelineDurationUs = 10000;
    // Maximum allowed control jitter
    uint32_t maximumAllowedJitterUs = 3000;
    // Consecutive fault threshold
    uint32_t maximumConsecutiveFaults = 5;
    // Enable runtime health monitoring
    bool enableHealthMonitoring = true;
    // Enable timing supervision
    bool enableTimingSupervision = true;
    // Enable emergency supervision
    bool enableEmergencySupervision = true;
    // Enable degraded mode
    bool enableDegradedMode = true;
    // Enable telemetry
    bool enableTelemetry = true;
    // Enable runtime statistics
    bool enableRuntimeStatistics = true;
    // Enable deterministic execution
    bool enableDeterministicExecution = true;
    // Enable automatic fault recovery
    bool enableAutomaticFaultRecovery = false;
};