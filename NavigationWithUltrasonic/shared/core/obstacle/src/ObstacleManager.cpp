//====================================================
// File: ObstacleManager.cpp
//====================================================

#include "obstacle/include/ObstacleManager.hpp"

#include <cmath>

uint32_t blockedDetectionFrames = 20;

namespace
{
    int dangerSeverity(
        DangerLevel level)
    {
        switch (level)
        {
        case DangerLevel::Safe:
            return 0;

        case DangerLevel::Caution:
            return 1;

        case DangerLevel::Avoid:
            return 2;

        case DangerLevel::Emergency:
            return 3;

        case DangerLevel::Blocked:
            return 4;

        case DangerLevel::Unknown:
        default:
            return -1;
        }
    }
}

//====================================================
// Constructor
//====================================================

ObstacleManager::ObstacleManager(
    const ObstacleManagerConfig &config)
    : m_config(config)
{
}

//====================================================
// Main processing pipeline
//====================================================

ObstacleAnalysis
ObstacleManager::process(
    const FilteredSensorData &sensorData,
    uint32_t currentTimestampMs)
{
    ObstacleAnalysis analysis;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    analysis.timestampMs =
        currentTimestampMs;

    //-----------------------------------------
    // Sensor health
    //-----------------------------------------

    analysis.sensorHealthy =
        isSensorHealthy(sensorData);

    //-----------------------------------------
    // Copy perception data
    //-----------------------------------------

    analysis.obstacleDistanceCm =
        sensorData.filteredDistanceCm;

    analysis.obstacleVelocityCmPerSec =
        sensorData.velocityCmPerSec;

    analysis.confidence =
        sensorData.confidence;

    analysis.isStable =
        sensorData.isStable;

    analysis.timeoutOccurred =
        sensorData.timeoutOccurred;

    analysis.consecutiveTimeouts =
        sensorData.consecutiveTimeouts;

    //-----------------------------------------
    // Obstacle validation
    //-----------------------------------------

    analysis.obstacleDetected =
        isObstacleValid(sensorData);

    //-----------------------------------------
    // Unknown environment
    //-----------------------------------------
    if (!analysis.sensorHealthy)
    {
        analysis.dangerLevel =
            DangerLevel::Unknown;

        // Solve: Obstacle exists BUT environment unknown
        analysis.obstacleDetected = false;
    }
    else
    {
        // Danger classification
        if (analysis.obstacleDetected)
        {
            DangerLevel newLevel =
                determineDangerLevel(sensorData);

            analysis.dangerLevel =
                applyHysteresis(
                    newLevel,
                    sensorData.filteredDistanceCm);
        }
        else
        {
            analysis.dangerLevel =
                DangerLevel::Safe;
        }
    }

    // Emergency detection
    analysis.emergencyDetected =
        (analysis.dangerLevel ==
         DangerLevel::Emergency);

    //-----------------------------------------
    // Temporal obstacle memory
    //-----------------------------------------

    updateObstacleMemory(
        analysis,
        currentTimestampMs);

    //-----------------------------------------
    // Obstacle persistence
    //-----------------------------------------

    analysis.obstacleRemembered =
        shouldRememberObstacle(
            currentTimestampMs);

    //-----------------------------------------
    // Blocked path detection
    //-----------------------------------------

    if (
        analysis.dangerLevel ==
            DangerLevel::Emergency &&
        m_memory.consecutiveDetectionFrames >= blockedDetectionFrames)
    {
        analysis.dangerLevel =
            DangerLevel::Blocked;
    }

    //-----------------------------------------
    // Save current danger state
    //-----------------------------------------

    m_currentDangerLevel =
        analysis.dangerLevel;

    return analysis;
}

//====================================================
// Danger classification
//====================================================

DangerLevel
ObstacleManager::determineDangerLevel(
    const FilteredSensorData &sensorData) const
{
    const float distance =
        sensorData.filteredDistanceCm;

    //-----------------------------------------
    // Emergency escalation
    //-----------------------------------------

    if (
        m_config.enableEmergencyOverride &&
        sensorData.velocityCmPerSec <
            m_config.emergencyVelocityThresholdCmPerSec)
    {
        return DangerLevel::Emergency;
    }

    //-----------------------------------------
    // Distance-based classification
    //-----------------------------------------

    if (
        distance <=
        m_config.emergencyDistanceCm)
    {
        return DangerLevel::Emergency;
    }

    if (
        distance <=
        m_config.avoidDistanceCm)
    {
        return DangerLevel::Avoid;
    }

    if (
        distance <=
        m_config.cautionDistanceCm)
    {
        return DangerLevel::Caution;
    }

    return DangerLevel::Safe;
}

//====================================================
// Hysteresis
//====================================================

DangerLevel
ObstacleManager::applyHysteresis(
    DangerLevel newLevel,
    float distanceCm) const
{
    if (
        dangerSeverity(newLevel) >
        dangerSeverity(m_currentDangerLevel))
    {
        return newLevel;
    }

    switch (m_currentDangerLevel)
    {
        //-----------------------------------------
        // Emergency persistence
        //-----------------------------------------

    case DangerLevel::Emergency:

        if (
            distanceCm <=
            (m_config.emergencyDistanceCm +
             m_config.hysteresisCm))
        {
            return DangerLevel::Emergency;
        }

        break;

        //-----------------------------------------
        // Avoid persistence
        //-----------------------------------------

    case DangerLevel::Avoid:

        if (
            distanceCm <=
            (m_config.avoidDistanceCm +
             m_config.hysteresisCm))
        {
            return DangerLevel::Avoid;
        }

        break;

        //-----------------------------------------
        // Caution persistence
        //-----------------------------------------

    case DangerLevel::Caution:

        if (
            distanceCm <=
            (m_config.cautionDistanceCm +
             m_config.hysteresisCm))
        {
            return DangerLevel::Caution;
        }

        break;

    default:
        break;
    }

    return newLevel;
}

//====================================================
// Obstacle validation
//====================================================

bool ObstacleManager::isObstacleValid(
    const FilteredSensorData &sensorData) const
{
    //-----------------------------------------
    // Confidence gating
    //-----------------------------------------

    if (
        m_config.enableConfidenceGating &&
        sensorData.confidence <
            m_config.minimumConfidence)
    {
        return false;
    }

    //-----------------------------------------
    // Stability validation
    //-----------------------------------------

    if (
        m_config.enableStabilityValidation &&
        !sensorData.isStable)
    {
        return false;
    }

    //-----------------------------------------
    // Sensor health
    //-----------------------------------------

    if (!isSensorHealthy(sensorData))
    {
        return false;
    }

    //-----------------------------------------
    // Valid obstacle range
    //-----------------------------------------

    return (
        sensorData.filteredDistanceCm <=
        m_config.cautionDistanceCm);
}

//====================================================
// Sensor health interpretation
//====================================================

bool ObstacleManager::isSensorHealthy(
    const FilteredSensorData &sensorData) const
{
    return (
        sensorData.consecutiveTimeouts <
        m_config.maxTimeoutTolerance);
}

//====================================================
// Temporal obstacle memory
//====================================================

void ObstacleManager::updateObstacleMemory(
    const ObstacleAnalysis &analysis,
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Obstacle detected
    //-----------------------------------------

    if (analysis.obstacleDetected)
    {
        //-----------------------------------------
        // Activate obstacle memory
        //-----------------------------------------

        m_memory.obstacleActive = true;

        m_memory.obstacleRemembered = true;

        //-----------------------------------------
        // Timing
        //-----------------------------------------

        if (m_memory.firstDetectedTimestampMs == 0)
        {
            m_memory.firstDetectedTimestampMs =
                currentTimestampMs;
        }

        m_memory.lastSeenTimestampMs =
            currentTimestampMs;

        //-----------------------------------------
        // Last known state
        //-----------------------------------------

        m_memory.lastDangerLevel =
            analysis.dangerLevel;

        m_memory.lastKnownDistanceCm =
            analysis.obstacleDistanceCm;

        m_memory.lastKnownVelocityCmPerSec =
            analysis.obstacleVelocityCmPerSec;

        m_memory.lastKnownConfidence =
            analysis.confidence;

        //-----------------------------------------
        // Detection counters
        //-----------------------------------------

        if (
            m_memory.consecutiveDetectionFrames <
            UINT32_MAX)
        {
            m_memory.consecutiveDetectionFrames++;
        }

        m_memory.consecutiveLostFrames = 0;

        //-----------------------------------------
        // Stability
        //-----------------------------------------

        m_memory.obstacleStable =
            (m_memory.consecutiveDetectionFrames >=
             m_config.minimumDetectionFrames);
    }
    else
    {
        //-----------------------------------------
        // Lost obstacle
        //-----------------------------------------

        if (
            m_memory.consecutiveLostFrames <
            UINT32_MAX)
        {
            m_memory.consecutiveLostFrames++;
        }

        //-----------------------------------------
        // Clear persistent state
        //-----------------------------------------

        if (
            m_memory.consecutiveLostFrames >
            m_config.maximumLostFrames)
        {
            m_memory.obstacleActive = false;

            m_memory.obstacleStable = false;

            m_memory.consecutiveDetectionFrames = 0;
        }
    }

    //-----------------------------------------
    // Sensor health snapshot
    //-----------------------------------------

    m_memory.sensorHealthy =
        analysis.sensorHealthy;
}

//====================================================
// Obstacle persistence
//====================================================

bool ObstacleManager::shouldRememberObstacle(
    uint32_t currentTimestampMs) const
{
    if (!m_config.enableObstacleMemory)
    {
        return false;
    }

    //-----------------------------------------
    // Memory timeout
    //-----------------------------------------

    return (
               currentTimestampMs -
               m_memory.lastSeenTimestampMs) <=
           m_config.obstacleMemoryMs;
}
