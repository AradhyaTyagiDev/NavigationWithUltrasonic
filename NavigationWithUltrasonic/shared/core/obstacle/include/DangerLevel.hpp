#pragma once

enum class DangerLevel
{
    // Environment unknown / unreliable. low confidence, repeated timeout, unstable readings
    // Robot may:slow down, enter cautious mode, stop temporarily
    Unknown,
    // No immediate danger
    Safe,
    // Obstacle detected at safe distance : reduce speed, prepare turning, monitor closely, increase awareness
    Caution,
    // Obstacle requires avoidance action, plan avoidance, alter trajectory
    Avoid,
    // Immediate collision danger
    Emergency,
    // Path fully blocked / trapped. No safe path available. obstacle too close continuously
    // reverse, rotate, enter escape mode
    Blocked
};