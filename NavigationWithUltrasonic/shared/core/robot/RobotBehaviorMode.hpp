//====================================================
// File: RobotBehaviorMode.hpp
//====================================================

#pragma once

//====================================================
// RobotBehaviorMode
//====================================================
//
// High-level robot behavioral personality.
//
// Controls:
//      navigation aggressiveness
//      safety sensitivity
//      locomotion confidence
//      obstacle tolerance
//
// Used by:
//      RobotController
//      NavigationManager
//      MotionPlanner
//
//====================================================
/*
Conservative Mode
    - larger danger distances
    - smoother EMA
    - slower motion
    - more cautious
Aggressive Mode
    - smaller safety margins
    - faster EMA
    - higher speeds
    - faster reactions

*/
enum class RobotBehaviorMode
{
    // Maximum safety
    Conservative,
    // Balanced production behavior
    Balanced,
    // Faster movement: More aggressive avoidance
    Aggressive,
    // Reduced movement: Sensor degradation mode
    SafeMode,
    // Experimental exploration behavior
    Exploration
};