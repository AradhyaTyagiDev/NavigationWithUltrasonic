//====================================================
// File: ObstacleManager.hpp
//====================================================

#pragma once

#include <stdint.h>

#include "ObstacleManagerConfig.hpp"
#include "ObstacleAnalysis.hpp"
#include "ObstacleMemory.hpp"
#include "DangerLevel.hpp"

#include "filter/include/FilteredSensorData.hpp"

class ObstacleManager
{
public:
    explicit ObstacleManager(
        const ObstacleManagerConfig &config);

    // Main environment interpretation pipeline
    ObstacleAnalysis process(
        const FilteredSensorData &sensorData,
        uint32_t currentTimestampMs);

private:
    // Environment interpretation
    DangerLevel determineDangerLevel(
        const FilteredSensorData &sensorData) const;

    // Danger hysteresis
    DangerLevel applyHysteresis(
        DangerLevel newLevel,
        float distanceCm) const;

    // Obstacle validation
    bool isObstacleValid(
        const FilteredSensorData &sensorData) const;

    // Sensor health interpretation
    bool isSensorHealthy(
        const FilteredSensorData &sensorData) const;

    // Environmental memory management
    void updateObstacleMemory(
        const ObstacleAnalysis &analysis,
        uint32_t currentTimestampMs);

    // Obstacle persistence validation
    bool shouldRememberObstacle(
        uint32_t currentTimestampMs) const;

private:
    // Configuration
    ObstacleManagerConfig m_config;

    // Temporal environment memory
    ObstacleMemory m_memory;

    // Current environment state
    DangerLevel m_currentDangerLevel =
        DangerLevel::Unknown;
};