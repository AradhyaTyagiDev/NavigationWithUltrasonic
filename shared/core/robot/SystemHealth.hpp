//====================================================
// File: SystemHealth.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// SystemHealth
//====================================================
//
// Global robotics runtime health.
//
// Tracks:
//      sensor health
//      driver health
//      timing health
//      locomotion health
//      runtime stability
//
//====================================================

struct SystemHealth
{
    // Overall system healthy
    bool systemHealthy = true;
    // Sensor subsystem
    bool sensorHealthy = true;
    // Filter subsystem
    bool filterHealthy = true;
    // Navigation subsystem
    bool navigationHealthy = true;
    // Motion subsystem
    bool motionHealthy = true;
    // Motor subsystem
    bool motorHealthy = true;
    // Driver subsystem
    bool driverHealthy = true;
    // Runtime timing healthy
    bool timingHealthy = true;
    // Emergency active
    bool emergencyActive = false;
    // Fault active
    bool faultActive = false;
    // Consecutive runtime faults
    uint32_t consecutiveFaultCount = 0;
    // Last fault timestamp
    uint32_t lastFaultTimestampMs = 0;
    // Last successful pipeline timestamp
    uint32_t lastHealthyTimestampMs = 0;
};