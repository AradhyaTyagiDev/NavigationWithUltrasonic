/*Robot behavior strategy
    escape behavior
    cautious mode
    turn preference
    navigation states
*/

#pragma once

#include <stdint.h>

struct NavigationManagerConfig
{
    // Speed profiles
    float fastSpeedPercent = 1.0f;
    float normalSpeedPercent = 0.7f;
    float cautiousSpeedPercent = 0.4f;
    float escapeSpeedPercent = 0.6f;

    // Turn behavior
    float cautiousTurnAngleDeg = 20.0f;
    float avoidanceTurnAngleDeg = 35.0f;
    float escapeTurnAngleDeg = 90.0f;

    // State persistence
    uint32_t minimumStateDurationMs = 300;
    uint32_t persistentBehaviorDurationMs = 700;

    // Transition stabilization
    uint32_t minimumStableFrames = 3;

    // Cooldown
    uint32_t avoidanceCooldownMs = 500;
    uint32_t blockedRecoveryCooldownMs = 1000;

    // Emergency behavior
    uint32_t emergencyStateDurationMs = 500;

    // Escape behavior
    uint32_t maximumEscapeAttempts = 5;

    // Blocked-state escalation
    uint32_t blockedStateThreshold = 3;

    // Confidence adaptation
    float lowConfidenceThreshold = 0.5f;
    float criticalConfidenceThreshold = 0.2f;

    // Dynamic safety adaptation
    bool enableConfidenceAdaptation = true;
    bool enableBehaviorPersistence = true;
    bool enableEmergencyOverride = true;
    bool enableDegradedSafetyMode = true;

    // Directional strategy
    bool alternateAvoidanceDirection = true;
    bool enableRandomEscapeDirection = true;
};