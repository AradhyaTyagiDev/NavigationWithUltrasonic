//====================================================
// File: IMotorDriver.hpp
//====================================================

#pragma once

#include "MotorDriverCapabilities.hpp"
#include "MotorDriverCommand.hpp"
#include "MotorDriverStatus.hpp"
#include "MotorDriverTypes.hpp"

//====================================================
// Hardware Abstraction Layer (HAL): It represents: one motor driver hardware module: one TB6612FNG board
// IMotorDriver: Abstract hardware motor driver interface.
// Provides: hardware abstraction layer
// Converts: low-level motor commands
// Into: Physical H-bridge actuation
// BTS7960MotorDriver
// TB6612MotorDriver
// CytronMotorDriver
// L298NMotorDriver
// VESCMotorDriver

// This interface should remain:
//      - lightweight
//      - deterministic
//      - hardware-focused

class IMotorDriver
{
public:
    virtual ~IMotorDriver() = default;

    // Driver initialization
    // Configure: GPIO, PWM, timers, driver hardware
    virtual bool initialize() = 0;

    // Driver shutdown
    virtual void shutdown() = 0;

    // Execute motor command
    // Executes: low-level hardware-safe command
    virtual void executeCommand(
        const MotorDriverCommand &command) = 0;

    // TB6612 updates both motors together, better synchronization
    virtual void executeDualCommand(
        const MotorDriverCommand &leftCommand,
        const MotorDriverCommand &rightCommand) = 0;

    // Stop all motors, Should: safely stop motors
    virtual void stopAllMotors() = 0;

    // Emergency stop, Should: immediately disable motion
    virtual void emergencyStop() = 0;

    // Clear emergency stop
    virtual void clearEmergencyStop() = 0;

    // Apply brake mode
    virtual void applyBrakeMode(
        BrakeMode brakeMode) = 0;

    // Set motor direction
    virtual void setMotorDirection(
        MotorChannel channel,
        MotorDirection direction) = 0;

    // Set PWM duty
    virtual void setPWMDuty(
        MotorChannel channel,
        uint32_t pwmDuty) = 0;

    // Enable motor output
    virtual void enableMotor(
        MotorChannel channel) = 0;

    // Disable motor output
    virtual void disableMotor(MotorChannel channel) = 0;

    // Driver ready state
    virtual bool isReady() const = 0;

    // Fault detection
    virtual bool hasFault() const = 0;

    // Hardware capabilities
    virtual MotorDriverCapabilities getCapabilities() const = 0;

    // Runtime status
    virtual MotorDriverStatus getStatus() const = 0;

    // Hardware type
    virtual MotorHardwareType getHardwareType() const = 0;

    // Runtime update loop: Update watchdog timeout.
    virtual void update(uint32_t currentTimestampMs) = 0;

    // Reset driver runtime: fault recovery, emergency recovery
    virtual void reset() = 0;

    // Driver runtime state
    virtual MotorDriverState getDriverState() const = 0;
};
