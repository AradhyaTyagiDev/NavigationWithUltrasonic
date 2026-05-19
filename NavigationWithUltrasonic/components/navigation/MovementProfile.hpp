/*
Defines:
- target speed behavior
- navigation aggressiveness
*/

#pragma once

enum class MovementProfile
{
    // Maximum safe traversal
    Fast,
    // Standard navigation
    Normal,
    // Reduced-speed cautious movement
    Cautious,
    // Escape / recovery motion
    Escape,
    // Emergency halt
    Emergency
};