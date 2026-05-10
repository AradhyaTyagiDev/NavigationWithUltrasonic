#pragma once

#include "UltrasonicFilterTypes.hpp"
#include "UltrasonicFilterConfig.hpp"

class UltrasonicFilter
{
public:
    explicit UltrasonicFilter(
        const UltrasonicFilterConfig &config);

    FilteredSensorData process(
        uint32_t pulseWidthUs,
        uint32_t timestampMs);

private:
    float pulseToDistance(
        uint32_t pulseWidthUs);

    bool isValidDistance(
        float distanceCm);

    float calculateConfidence(
        float newDistance);

    bool isOutlier(
        float newDistance);

    float applyDeadZone(
        float newDistance);

    float applyAdaptiveEMA(
        float newDistance);

    float calculateVelocity(
        float currentDistance,
        uint32_t currentTimestamp);

    bool updateStability(
        float filteredDistance);

    float applyConfidenceDecay(
        float newConfidence);

private:
    UltrasonicFilterConfig m_config;

    float m_lastDistance = 400.0f;

    float m_filteredDistance = 400.0f;

    float m_lastVelocity = 0.0f;

    uint32_t m_lastTimestamp = 0;

    uint32_t m_stableCounter = 0;

    float m_persistentConfidence = 1.0f;

    uint32_t m_consecutiveTimeouts = 0;
};