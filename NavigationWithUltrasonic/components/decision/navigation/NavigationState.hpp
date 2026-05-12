#pragma once

enum class NavigationState
{
    // Robot inactive
    Idle,
    Forward,
    // Reduced-speed cautious movement
    CautiousForward,
    // Active obstacle avoidance
    Avoiding,
    // Immediate safety stop
    EmergencyStop,
    // Escape / recovery behavior
    EscapeMode,
    // No safe navigation path
    Blocked
};