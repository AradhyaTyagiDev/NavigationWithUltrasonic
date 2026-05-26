/*
Live behavioral runtime state memory: Robot's short-term behavioral brain state
It stores:
 - current behavioral context: previous behavior, current behavior, stable behavior
 - persistence state: still avoiding, just started avoiding, just stopped avoiding
 - transition state
 - cooldown state : don’t flip yet - anti-oscillation - turn persistence - escape persistence
 - directional memory: last turned left or right, turn preference
 - escape attempts - 3 failed escapes
 - blocked persistence - if blocked for 5 seconds, enter trapped mode

 Only One Instance: Because robot only has ONE active navigation behavior at a time.

Behavioral persistence memory. Supports:
- anti-oscillation
- cooldowns
- turn memory
- escape persistence
*/

#pragma once

#include <stdint.h>

#include "NavigationState.hpp"
#include "TurnDirection.hpp"

struct NavigationMemory
{
    NavigationState currentState = NavigationState::Idle;

    // Previous navigation state
    NavigationState previousState = NavigationState::Idle;

    // Current stable state
    NavigationState stableState = NavigationState::Idle;

    // Last avoidance direction
    TurnDirection lastTurnDirection = TurnDirection::Left;

    // State transition timing. when transition occurred
    uint32_t lastStateChangeTimestampMs = 0;

    // duration in current state
    uint32_t stateEntryTimestampMs = 0;

    // Persistence tracking
    uint32_t stableFrameCount = 0;

    // Escape tracking
    uint32_t consecutiveEscapeAttempts = 0;

    // Blocked-state persistence
    uint32_t blockedFrameCount = 0;

    // Cooldown tracking
    uint32_t cooldownUntilTimestampMs = 0;

    // Persistent behavior active
    bool persistentBehaviorActive = false;
};