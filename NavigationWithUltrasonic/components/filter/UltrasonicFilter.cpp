#pragma once

#include "UltrasonicFilterTypes.hpp"
#include "UltrasonicFilterConfig.hpp"
#include "UltrasonicFilter.hpp"

#include <cmath>

static constexpr float SOUND_DIVIDER = 58.0f;

UltrasonicFilter::UltrasonicFilter(
    const UltrasonicFilterConfig &config)
    : m_config(config)
{
    m_filteredDistance = config.timeoutDistanceCm;
    m_lastDistance = config.timeoutDistanceCm;
}

FilteredSensorData UltrasonicFilter::process(
    uint32_t pulseWidthUs,
    uint32_t timestampMs)
{
    FilteredSensorData result;

    result.timestampMs = timestampMs;

    //-----------------------------------------
    // RAW → DISTANCE
    //-----------------------------------------

    float rawDistance = pulseToDistance(
        pulseWidthUs);

    result.rawDistanceCm = rawDistance;

    //-----------------------------------------
    // VALIDITY CHECK
    //-----------------------------------------
    bool validMeasurement = true;

    if (!isValidDistance(rawDistance))
    {
        validMeasurement = false;

        // Invalid measurement / timeout
        m_consecutiveTimeouts++;

        result.timeoutOccurred = true;

        result.consecutiveTimeouts = m_consecutiveTimeouts;

        result.confidence = 0.0f;

        rawDistance = m_config.timeoutDistanceCm;
    }
    else
    {
        // Valid measurement
        m_consecutiveTimeouts = 0;

        result.timeoutOccurred = false;

        result.consecutiveTimeouts = 0;
    }

    //-----------------------------------------
    // CONFIDENCE SCORE
    //-----------------------------------------
    float instantConfidence = 0.0f;

    if (validMeasurement)
    {
        // This evaluates: current jump, current validity, current anomaly
        instantConfidence = calculateConfidence(rawDistance);
    }

    /// smooth confidence evolution, long-term trust memory
    result.confidence = applyConfidenceDecay(instantConfidence);

    //-----------------------------------------
    // OUTLIER REJECTION
    //-----------------------------------------

    if (isOutlier(rawDistance))
    {
        // Outlier rejection should affect confidence but not completely discard the measurement, as it might be a real sudden change (e.g., fast obstacle)
        result.confidence *= 0.5f;

        rawDistance = m_lastDistance;
    }

    //-----------------------------------------
    // DEAD ZONE FILTER
    //-----------------------------------------

    rawDistance = applyDeadZone(
        rawDistance);

    //-----------------------------------------
    // ADOPTIVE EMA FILTER
    //-----------------------------------------

    float filteredDistance = applyAdaptiveEMA(
        rawDistance);

    //-----------------------------------------
    // VELOCITY
    //-----------------------------------------
    result.velocityCmPerSec = calculateVelocity(
        filteredDistance,
        timestampMs);

    if (fabs(result.velocityCmPerSec) > m_config.maxRealisticVelocityCmPerSec)
    {
        // Unrealistic jump detected
        result.confidence *= 0.2f;

        result.velocityCmPerSec = 0.0f;

        m_filteredDistance = m_lastDistance;
        filteredDistance = m_lastDistance;
    }

    // STABILITY WINDOW
    result.isStable = updateStability(filteredDistance) && result.confidence > 0.5f;

    // FINAL OBSTACLE STATE
    result.obstacleDetected =
        result.isStable &&
        filteredDistance < m_config.obstacleThresholdCm;

    result.filteredDistanceCm = filteredDistance;

    m_lastDistance = filteredDistance;

    return result;
}

float UltrasonicFilter::pulseToDistance(
    uint32_t pulseWidthUs)
{
    return static_cast<float>(pulseWidthUs) / SOUND_DIVIDER;
}

bool UltrasonicFilter::isValidDistance(
    float distanceCm)
{
    return distanceCm >= m_config.minDistanceCm &&
           distanceCm <= m_config.maxDistanceCm;
}

float UltrasonicFilter::calculateConfidence(
    float newDistance)
{
    float delta = fabs(newDistance - m_lastDistance);

    if (newDistance <= 0 ||
        newDistance > m_config.maxDistanceCm)
    {
        return 0.0f;
    }

    if (delta < m_config.smallJumpThresholdCm)
    {
        return 1.0f;
    }

    if (delta < m_config.largeJumpThresholdCm)
    {
        return 0.8f;
    }

    return 0.2f;
}

//-----------------------------------------
// Confidence smoothing factor
//-----------------------------------------
//
// Lower value:
//     smoother confidence
//
// Higher value:
//     faster confidence reaction
//-----------------------------------------
float UltrasonicFilter::applyConfidenceDecay(
    float newConfidence)
{
    // EMA-based confidence decay
    m_persistentConfidence =
        (m_config.confidenceDecayAlpha * newConfidence) +
        ((1.0f - m_config.confidenceDecayAlpha) * m_persistentConfidence);

    // Clamp safety
    if (m_persistentConfidence < 0.0f)
    {
        m_persistentConfidence = 0.0f;
    }

    if (m_persistentConfidence > 1.0f)
    {
        m_persistentConfidence = 1.0f;
    }

    return m_persistentConfidence;
}

bool UltrasonicFilter::isOutlier(
    float newDistance)
{
    float delta = fabs(newDistance - m_lastDistance);

    return delta > m_config.largeJumpThresholdCm;
}

float UltrasonicFilter::applyDeadZone(
    float newDistance)
{
    if (fabs(newDistance - m_lastDistance) < m_config.deadZoneCm)
    {
        return m_lastDistance;
    }

    return newDistance;
}

float UltrasonicFilter::applyAdaptiveEMA(
    float newDistance)
{
    // Distance change magnitude
    float delta =
        fabs(newDistance - m_filteredDistance);

    // Dynamic alpha calculation
    float alpha = 0.0f;

    // Very stable region
    if (delta < m_config.adaptiveEMAStableDeltaCm)
    {
        alpha = 0.20f;
    }
    else if (delta < m_config.adaptiveEMAModerateDeltaCm)
    {
        // Moderate movement
        alpha = 0.50f;
    }
    else
    {
        // Fast obstacle / rapid change
        alpha = 0.80f;
    }

    // EMA update
    m_filteredDistance =
        (alpha * newDistance) +
        ((1.0f - alpha) * m_filteredDistance);

    return m_filteredDistance;
}

float UltrasonicFilter::calculateVelocity(
    float currentDistance,
    uint32_t currentTimestamp)
{
    if (m_lastTimestamp == 0)
    {
        m_lastTimestamp = currentTimestamp;

        return 0.0f;
    }

    float deltaDistance =
        currentDistance - m_lastDistance;

    float deltaTimeSec =
        static_cast<float>(
            currentTimestamp - m_lastTimestamp) /
        1000.0f;

    if (deltaTimeSec <= 0.0f)
    {
        return 0.0f;
    }

    m_lastTimestamp = currentTimestamp;

    m_lastVelocity =
        deltaDistance / deltaTimeSec;

    return m_lastVelocity;
}

bool UltrasonicFilter::updateStability(
    float filteredDistance)
{
    if (filteredDistance < m_config.obstacleThresholdCm)
    {
        m_stableCounter++;
    }
    else
    {
        m_stableCounter = 0;
    }

    return m_stableCounter >=
           m_config.requiredStableFrames;
}