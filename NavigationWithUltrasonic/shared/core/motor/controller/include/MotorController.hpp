//====================================================
// File: MotorController.hpp
//====================================================

#pragma once

#include "interfaces/include/motor/driver/MotorDriverCommand.hpp"
#include "interfaces/include/motor/driver/IMotorDriver.hpp"
#include "interfaces/include/synchronization/IMutex.hpp"
#include "interfaces/include/logging/ILogger.hpp"
#include "interfaces/include/timing/ITimer.hpp"

#include "motion/include/MotionCommand.hpp"
#include "motor/controller/include/MotorControllerConfig.hpp"
#include "motor/controller/include/MotorControllerMemory.hpp"
#include "motor/controller/include/MotorState.hpp"
#include "motor/controller/include/WheelCommand.hpp"
#include "motor/controller/include/WheelState.hpp"

#include <stdint.h>

//====================================================
// MotorController: Real-time locomotion execution coordinator.
// Converts: MotionCommand into: MotorDriverCommand: synchronized safe motor execution through: IMotorDriver
//  - locomotion execution coordination
//  - wheel synchronization
//  - acceleration coordination
//  - coordinated braking
//  - emergency coordination
//  - motion execution state machine
//  - runtime monitoring
//  - locomotion safety validation
//  - reverse transition protection
//  - fault handling
// Those belong to: IMotorDriver
//====================================================

class MotorController final
{
public:
    // Constructor
    MotorController(
        IMotorDriver &motorDriver,
        IMutex &mutex,
        ILogger &logger,
        ITimer &timer,
        const MotorControllerConfig &config);

    // Initialization
    bool initialize();

    // Shutdown
    void shutdown();

    // Main locomotion execution pipeline, Input: MotionCommand, Output: MotorDriverCommand execution
    void executeMotion(const MotionCommand &motionCommand);

    bool tryExecuteMotion(const MotionCommand &motionCommand);

    // Runtime update loop
    // Handles: synchronization, monitoring, fault handling, timeout supervision, execution state transitions
    void update(uint32_t currentTimestampMs);

    // Emergency stop
    void emergencyStop();

    // Clear emergency stop
    void clearEmergencyStop();

    // Stop locomotion
    void stop();

    // Reset runtime state
    void reset();

    // Current motor execution state
    MotorState getCurrentState() const;

    // Runtime memory
    MotorControllerMemory getMemory() const;

    // Fault state
    bool hasFault() const;

    // Emergency state
    bool isEmergencyStopActive() const;

private:
    // Motion validation
    bool validateMotionCommand(const MotionCommand &motionCommand) const;

    // Motion safety validation
    bool validateMotionSafety(const MotionCommand &motionCommand) const;

    // Generate wheel commands
    void generateWheelCommands(
        const MotionCommand &motionCommand,
        WheelCommand &leftWheelCommand,
        WheelCommand &rightWheelCommand);

    void executeMotionInternal(const MotionCommand &motionCommand);

    // Generate left wheel command
    WheelCommand generateLeftWheelCommand(const MotionCommand &motionCommand);

    // Generate right wheel command
    WheelCommand generateRightWheelCommand(const MotionCommand &motionCommand);

    // Apply wheel synchronization
    // Ensures: stable differential drive execution, synchronized acceleration, coordinated locomotion
    void applyWheelSynchronization(WheelCommand &leftWheelCommand, WheelCommand &rightWheelCommand);

    // Apply motion smoothing
    void applyMotionSmoothing(
        WheelCommand &leftWheelCommand,
        WheelCommand &rightWheelCommand,
        uint32_t currentTimestampMs);

    // Apply acceleration limits
    void applyAccelerationLimits(
        WheelCommand &wheelCommand,
        WheelState &wheelState,
        uint32_t currentTimestampMs);

    // Apply deceleration limits
    void applyDecelerationLimits(
        WheelCommand &wheelCommand,
        WheelState &wheelState,
        uint32_t currentTimestampMs);

    // Apply startup boost
    void applyStartupBoost(WheelCommand &wheelCommand, const WheelState &wheelState);

    // Apply deadzone compensation
    void applyDeadzoneCompensation(WheelCommand &wheelCommand);

    // Apply coordinated braking
    void applyCoordinatedBraking(WheelCommand &leftWheelCommand, WheelCommand &rightWheelCommand);

    // Apply emergency behavior
    void applyEmergencyBehavior(WheelCommand &leftWheelCommand, WheelCommand &rightWheelCommand);

    // Reverse transition protection
    bool validateReverseTransition(const WheelCommand &wheelCommand, const WheelState &wheelState) const;

    // Generate motor driver command
    MotorDriverCommand
    generateMotorDriverCommand(
        MotorChannel channel,
        const WheelCommand &wheelCommand,
        uint32_t sequenceId) const;

    // Execute synchronized wheel commands
    void executeWheelCommands(
        const MotorDriverCommand &leftCommand,
        const MotorDriverCommand &rightCommand);

    // Runtime state transition
    void transitionToState(MotorState newState);

    // Determine runtime state
    MotorState determineMotorState(const MotionCommand &motionCommand) const;

    // Fault handling
    void handleFault(const char *reason);

    // Motion timeout supervision
    bool hasMotionTimedOut(uint32_t currentTimestampMs) const;

    // Runtime monitoring
    void performRuntimeMonitoring(uint32_t currentTimestampMs);

    // Update wheel runtime state
    void updateWheelState(
        WheelState &wheelState,
        const WheelCommand &wheelCommand,
        uint32_t currentTimestampMs);

    // Update runtime memory
    void updateRuntimeMemory(const MotionCommand &motionCommand, uint32_t currentTimestampMs);

    // Timestamp utility
    uint32_t getCurrentTimestampMs() const;

    bool validateStateTransition(MotorState currentState, MotorState newState) const;

    void stopInternal();

    void emergencyStopInternal();

    bool validateDriverHealth() const;

    bool shouldSkipUpdate(uint32_t currentTimestampMs) const;

private:
    // Hardware abstraction layer
    IMotorDriver &m_motorDriver;

    // Runtime locomotion memory
    MotorControllerMemory m_memory;

    IMutex &m_mutex;

    ILogger &m_logger;

    ITimer &m_timer;

    // Runtime sequence generator: Generates unique execution sequence IDs.
    // Useful for: synchronization, telemetry, debugging, RTOS tracing
    uint32_t m_sequenceCounter = 0;

    // Configuration
    MotorControllerConfig m_config;
};
