//====================================================
// File: RuntimeStatistics.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// RuntimeStatistics
//====================================================
//
// Runtime execution statistics.
//
// Used for:
//      diagnostics
//      telemetry
//      performance monitoring
//      runtime analysis
//
//====================================================

struct RuntimeStatistics
{
    // Total control loop executions
    uint64_t totalControlLoops = 0;

    // Successful control loops
    uint64_t successfulControlLoops = 0;

    // Failed control loops
    uint64_t failedControlLoops = 0;

    // Total sensor updates
    uint64_t sensorUpdates = 0;

    // Total obstacle analyses
    uint64_t obstacleAnalyses = 0;

    // Total navigation decisions
    uint64_t navigationDecisions = 0;

    // Total motion plans
    uint64_t motionPlans = 0;

    // Total motor executions
    uint64_t motorExecutions = 0;

    // Emergency stop count
    uint64_t emergencyStops = 0;

    // Fault count
    uint64_t totalFaults = 0;

    // Timing violations
    uint64_t timingViolations = 0;

    // Missed control cycles
    uint64_t missedControlCycles = 0;

    // Sensor timeout count
    uint64_t sensorTimeouts = 0;

    // Motion timeout count
    uint64_t motionTimeouts = 0;

    // Driver fault count
    uint64_t driverFaults = 0;

    // Last statistics update
    uint32_t lastStatisticsUpdateTimestampMs = 0;
};