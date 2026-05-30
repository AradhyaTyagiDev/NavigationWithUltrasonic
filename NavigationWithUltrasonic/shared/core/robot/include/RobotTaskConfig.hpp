//====================================================
// File: RobotTaskConfig.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// RobotTaskConfig
//====================================================
//
// RTOS task configuration.
//
// Defines:
//      priorities
//      core affinity
//      update frequencies
//
//====================================================

struct RobotTaskConfig
{
    // Task priorities
    uint32_t sensorTaskPriority = 6;
    uint32_t controllerTaskPriority = 8;
    uint32_t driverTaskPriority = 7;
    uint32_t telemetryTaskPriority = 2;

    // Core affinity
    uint32_t sensorCore = 0;
    uint32_t controllerCore = 1;
    uint32_t driverCore = 1;
    uint32_t telemetryCore = 0;

    // Update frequencies
    uint32_t sensorHz = 50;
    uint32_t controllerHz = 50;
    uint32_t driverHz = 200;
    uint32_t telemetryHz = 5;

    // Stack sizes
    uint32_t sensorTaskStackSize = 4096;
    uint32_t controllerTaskStackSize = 8192;
    uint32_t driverTaskStackSize = 4096;
    uint32_t telemetryTaskStackSize = 4096;
};