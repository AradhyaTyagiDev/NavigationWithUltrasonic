//====================================================
// File: SimulationUltrasonicSensor.hpp
//====================================================

#pragma once

#include <stdint.h>

#include <webots/DistanceSensor.hpp>

#include "IUltrasonicSensor.hpp"

//====================================================
// SimulationUltrasonicSensor
//
// Webots ultrasonic sensor adapter.
//
// PURPOSE
//
//  - provides simulated ultrasonic perception
//  - mimics real UltrasonicSensor behavior
//  - feeds RobotController using same API
//
// IMPORTANT
//
//  This class converts Webots distance sensor
//  output into UltrasonicSensorData.
//
//====================================================

class SimulationUltrasonicSensor final : public IUltrasonicSensor
{
public:
    //================================================
    // Configuration
    //================================================

    struct Config
    {
        //-----------------------------------------
        // Webots timestep
        //-----------------------------------------

        int timeStepMs = 32;

        //-----------------------------------------
        // Distance scale factor
        //
        // Converts Webots sensor units
        // into approximate pulse width
        //-----------------------------------------

        float distanceScaleFactor = 58.0f;

        //-----------------------------------------
        // Runtime enabled
        //-----------------------------------------

        bool enabled = true;
    };

public:
    SimulationUltrasonicSensor(
        webots::DistanceSensor *sensor,
        const Config &config);

    ~SimulationUltrasonicSensor()
        override = default;

    //================================================
    // Initialization
    //================================================

    bool initialize() override;

    //================================================
    // Start runtime
    //================================================

    bool start() override;

    //================================================
    // Shutdown
    //================================================

    void shutdown() override;

    //================================================
    // Fetch latest sensor data
    //================================================

    bool fetchLatestData(
        UltrasonicSensorData &outData)
        override;

    //================================================
    // Sensor health
    //================================================

    bool isHealthy() const override;

    //================================================
    // Runtime active
    //================================================

    bool isRunning() const override;

private:
    //================================================
    // Read Webots sensor
    //================================================

    bool readSensorFrame(
        UltrasonicSensorData &outData);

    //================================================
    // Timestamp utility
    //================================================

    uint32_t getCurrentTimestampMs() const;

private:
    //-----------------------------------------
    // Webots sensor
    //-----------------------------------------

    webots::DistanceSensor *m_sensor =
        nullptr;

    //-----------------------------------------
    // Configuration
    //-----------------------------------------

    Config m_config;

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    bool m_initialized = false;

    bool m_running = false;

    bool m_healthy = true;

    //-----------------------------------------
    // Latest frame cache
    //-----------------------------------------

    UltrasonicSensorData m_latestData;
};