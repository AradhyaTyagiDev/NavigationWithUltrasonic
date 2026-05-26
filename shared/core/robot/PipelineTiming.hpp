//====================================================
// File: PipelineTiming.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// PipelineTiming
//====================================================
//
// Real-time robotics pipeline timing.
//
// Critical for:
//      deterministic execution
//      latency tracking
//      jitter monitoring
//      runtime stability
//
//====================================================

struct PipelineTiming
{
    // Total control loop duration
    uint32_t controlLoopDurationUs = 0;
    // Sensor stage duration
    uint32_t sensorStageDurationUs = 0;
    // Filter stage duration
    uint32_t filterStageDurationUs = 0;
    // Obstacle stage duration
    uint32_t obstacleStageDurationUs = 0;
    // Navigation stage duration
    uint32_t navigationStageDurationUs = 0;
    // Motion stage duration
    uint32_t motionStageDurationUs = 0;
    // Motor stage duration
    uint32_t motorStageDurationUs = 0;
    // Worst-case loop duration
    uint32_t worstCaseLoopDurationUs = 0;
    // Average loop duration
    uint32_t averageLoopDurationUs = 0;
    // Pipeline jitter
    uint32_t pipelineJitterUs = 0;
    // Missed control cycles
    uint32_t missedControlCycles = 0;
    // Last update timestamp
    uint32_t lastUpdateTimestampMs = 0;
};