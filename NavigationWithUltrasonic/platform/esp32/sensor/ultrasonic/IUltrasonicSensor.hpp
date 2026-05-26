//====================================================
// File: IUltrasonicSensor.hpp
//====================================================

#pragma once

#include "UltrasonicTypes.hpp"

//====================================================
// IUltrasonicSensor
//
// Hardware abstraction interface for
// ultrasonic perception systems.
//
// PURPOSE
//
//  - decouple robotics runtime from hardware
//  - support simulation + real hardware
//  - enable testing portability
//  - allow future sensor replacement
//
// IMPLEMENTATIONS
//
//  ESP32:
//      UltrasonicSensor
//
//  Webots:
//      SimulationUltrasonicSensor
//
// IMPORTANT
//
//  Interface intentionally minimal.
//
//  Sensor implementation owns:
//      - acquisition task
//      - buffering
//      - timing
//      - hardware runtime
//
//====================================================

class IUltrasonicSensor
{
public:
    virtual ~IUltrasonicSensor() = default;

    // Initialize sensor subsystem
    virtual bool initialize() = 0;

    // Start sensor runtime
    virtual bool start() = 0;

    // Shutdown sensor subsystem
    virtual void shutdown() = 0;

    //================================================
    // Fetch latest perception frame
    // IMPORTANT: Non-blocking
    // Returns:
    //      true  -> fresh/latest frame available
    //      false -> no frame available
    //================================================

    virtual bool fetchLatestData(
        UltrasonicSensorData &outData) = 0;

    // Sensor health
    virtual bool isHealthy() const = 0;

    // Runtime active
    virtual bool isRunning() const = 0;
};