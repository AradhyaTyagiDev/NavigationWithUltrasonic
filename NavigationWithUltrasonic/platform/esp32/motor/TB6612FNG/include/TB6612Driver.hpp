//====================================================
// File: TB6612Driver.hpp
//====================================================

#pragma once

#include "motor/TB6612FNG/include/TB6612DriverConfig.hpp"

#include "motor/TB6612FNG/include/TB6612DriverMemory.hpp"

#include "interfaces/include/motor/driver/IMotorDriver.hpp"
#include "interfaces/include/synchronization/SynchronizedObject.hpp"
#include "interfaces/include/logging/ILogger.hpp"
#include "interfaces/include/timing/ITimer.hpp"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include <stdint.h>

//====================================================
// TB6612Driver
// Production-grade dual motor driver for
// TB6612FNG motor controller.
// Hardware execution layer responsible for:
//  - GPIO control
//  - PWM generation
//  - motor direction control
//  - brake control
//  - emergency stop
//  - safe hardware transitions
//  - synchronized dual-motor updates
//
// This layer is:
//  - hardware-focused
//  - deterministic
//  - real-time safe
//----------------------------------------------------
// HARDWARE
//----------------------------------------------------
//
// TB6612FNG:
//
//      Channel A -> Left Motor
//      Channel B -> Right Motor
//====================================================

class TB6612Driver final : public IMotorDriver
{
public:
    explicit TB6612Driver(IMutex &mutex, ILogger &logger, ITimer &timer, const TB6612DriverConfig &config);

    // Destructor
    ~TB6612Driver() override = default;

    // Initialization
    bool initialize() override;

    void shutdown() override;

    //================================================
    // Runtime update loop
    // Handles:
    //  - watchdog timeout
    //  - PWM ramping
    //  - runtime maintenance
    //  - safety supervision
    void update(uint32_t currentTimestampMs) override;

    // Runtime reset
    void reset() override;

    // Driver runtime state
    MotorDriverState getDriverState() const override;

    // Motor command execution
    void executeCommand(const MotorDriverCommand &command) override;

    //================================================
    // Dual synchronized command execution
    // Important for:
    //      differential drive robots
    // Reduces:
    //      wheel update jitter
    void executeDualCommand(const MotorDriverCommand &leftCommand, const MotorDriverCommand &rightCommand) override;

    // Stop motors
    void stopAllMotors() override;

    // Emergency stop
    void emergencyStop() override;

    void clearEmergencyStop() override;

    // Brake control
    void applyBrakeMode(BrakeMode brakeMode) override;

    // Direction control
    void setMotorDirection(MotorChannel channel, MotorDirection direction) override;

    // PWM control
    void setPWMDuty(MotorChannel channel, uint32_t pwmDuty) override;

    // Motor enable/disable
    void enableMotor(MotorChannel channel) override;

    void disableMotor(MotorChannel channel) override;

    // Driver state
    bool isReady() const override;

    bool hasFault() const override;

    // Capabilities
    MotorDriverCapabilities getCapabilities() const override;

    // Runtime status
    MotorDriverStatus getStatus() const override;

    // Hardware type
    MotorHardwareType getHardwareType() const override;

private:
    // GPIO initialization
    bool initializeGPIO();

    // PWM initialization
    bool initializePWM();

    // Standby control
    void setStandbyMode(bool enabled);

    // Direction application
    void applyMotorDirection(MotorChannel channel, MotorDirection direction);

    // Brake application
    void applyMotorBrake(MotorChannel channel, BrakeMode brakeMode);

    // PWM application
    void applyPWMDuty(MotorChannel channel, uint32_t pwmDuty);

    // PWM validation
    uint32_t validatePWMDuty(uint32_t pwmDuty) const;

    // Deadzone compensation
    uint32_t applyDeadzoneCompensation(uint32_t pwmDuty) const;

    // Startup boost compensation
    uint32_t applyStartupBoost(MotorChannel channel, uint32_t pwmDuty, uint32_t currentTimestampMs);

    // Lightweight PWM ramping
    uint32_t applyPWMRamping(MotorChannel channel, uint32_t targetPWMDuty);

    // Safe direction transition
    // Prevents: forward -> reverse instantly
    bool validateDirectionTransition(MotorChannel channel, MotorDirection newDirection, uint32_t currentTimestampMs);

    // Safe reverse sequence
    // Forward -> brake -> coast -> reverse
    void performSafeReverseSequence(MotorChannel channel, MotorDirection newDirection);

    // PWM channel commit
    void commitPWMUpdate(MotorChannel channel);

    // Synchronized dual update
    void commitSynchronizedPWMUpdate();

    // Motor stop helper
    void stopMotor(MotorChannel channel);

    // Emergency brake helper
    void applyEmergencyBrake();

    // Runtime memory update
    void updateMotorMemory(MotorChannel channel, MotorDirection direction, uint32_t pwmDuty, uint32_t currentTimestampMs);

    // Watchdog timeout validation
    bool hasCommandTimedOut(uint32_t currentTimestampMs) const;

    // Timeout handler
    void handleCommandTimeout();

    // Strict state transition validation
    bool validateStateTransition(MotorDriverState currentState, MotorDriverState newState) const;

    // Runtime state transition
    void transitionToState(MotorDriverState newState);

    // Channel mapping utilities
    // Timestamp utility
    uint32_t getCurrentTimestampMs() const;

    // Channel mapping utilities
    gpio_num_t getIN1Pin(MotorChannel channel) const;

    gpio_num_t getIN2Pin(MotorChannel channel) const;

    ledc_channel_t getPWMChannel(MotorChannel channel) const;

    // Motor inversion utility
    bool isMotorInverted(MotorChannel channel) const;

    void executeCommandInternal(const MotorDriverCommand &command);

private:
    IMutex &m_mutex;

    ILogger &m_logger;

    ITimer &m_timer;

    // Configuration
    TB6612DriverConfig m_config;

    // Runtime memory
    TB6612DriverMemory m_memory;

    // Hardware capabilities
    MotorDriverCapabilities m_capabilities;
};
