//====================================================
// File: MotorController.cpp
//====================================================

#include "MotorController.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MotorController";

//====================================================
// Constructor
//====================================================

MotorController::MotorController(
    IMotorDriver &motorDriver,
    const MotorControllerConfig &config)
    : m_motorDriver(motorDriver),
      m_config(config)
{
}

//====================================================
// Initialization
//====================================================

bool MotorController::initialize()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Driver initialization
    //-----------------------------------------

    if (!m_motorDriver.initialize())
    {
        ESP_LOGE(
            TAG,
            "Motor driver initialization failed");

        return false;
    }

    //-----------------------------------------
    // Reset runtime memory
    //-----------------------------------------

    m_memory = {};

    //-----------------------------------------
    // Initial runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::Idle);

    ESP_LOGI(
        TAG,
        "MotorController initialized");

    return true;
}

//====================================================
// Shutdown
//====================================================

void MotorController::shutdown()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Stop locomotion
    //-----------------------------------------

    stopInternal();

    //-----------------------------------------
    // Shutdown driver
    //-----------------------------------------

    m_motorDriver.shutdown();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::Idle);

    ESP_LOGI(
        TAG,
        "MotorController shutdown");
}

//====================================================
// Main motion execution pipeline
//====================================================

void MotorController::executeMotion(
    const MotionCommand &motionCommand)
{
    ScopedControllerLock lock(this);

    executeMotionInternal(
        motionCommand);
}

//====================================================
// Internal motion execution
//====================================================

void MotorController::executeMotionInternal(
    const MotionCommand &motionCommand)
{
    //-----------------------------------------
    // Emergency protection
    //-----------------------------------------

    if (m_memory.emergencyStopActive)
    {
        ESP_LOGW(
            TAG,
            "Ignoring motion during emergency");

        return;
    }

    //-----------------------------------------
    // Driver health validation
    //-----------------------------------------

    if (!validateDriverHealth())
    {
        handleFault(
            "Driver health validation failed");

        return;
    }

    //-----------------------------------------
    // Motion validation
    //-----------------------------------------

    if (!validateMotionCommand(
            motionCommand))
    {
        handleFault(
            "Invalid motion command");

        return;
    }

    //-----------------------------------------
    // Motion safety validation
    //-----------------------------------------

    if (!validateMotionSafety(
            motionCommand))
    {
        handleFault(
            "Unsafe motion command");

        return;
    }

    //-----------------------------------------
    // Runtime timestamp
    //-----------------------------------------

    const uint32_t currentTimestampMs =
        getCurrentTimestampMs();

    //-----------------------------------------
    // Wheel commands
    //-----------------------------------------

    WheelCommand leftWheelCommand;

    WheelCommand rightWheelCommand;

    generateWheelCommands(
        motionCommand,
        leftWheelCommand,
        rightWheelCommand);

    //-----------------------------------------
    // Synchronization
    //-----------------------------------------

    applyWheelSynchronization(
        leftWheelCommand,
        rightWheelCommand);

    //-----------------------------------------
    // Motion smoothing
    //-----------------------------------------

    applyMotionSmoothing(
        leftWheelCommand,
        rightWheelCommand,
        currentTimestampMs);

    //-----------------------------------------
    // Deadzone compensation
    //-----------------------------------------

    applyDeadzoneCompensation(
        leftWheelCommand);

    applyDeadzoneCompensation(
        rightWheelCommand);

    //-----------------------------------------
    // Startup boost
    //-----------------------------------------

    applyStartupBoost(
        leftWheelCommand,
        m_memory.leftWheelState);

    applyStartupBoost(
        rightWheelCommand,
        m_memory.rightWheelState);

    //-----------------------------------------
    // Reverse transition validation
    //-----------------------------------------

    if (!validateReverseTransition(
            leftWheelCommand,
            m_memory.leftWheelState))
    {
        leftWheelCommand.speedPercent =
            0.0f;
    }

    if (!validateReverseTransition(
            rightWheelCommand,
            m_memory.rightWheelState))
    {
        rightWheelCommand.speedPercent =
            0.0f;
    }

    //-----------------------------------------
    // Coordinated braking
    //-----------------------------------------

    applyCoordinatedBraking(
        leftWheelCommand,
        rightWheelCommand);

    //-----------------------------------------
    // Emergency behavior
    //-----------------------------------------

    if (
        motionCommand.emergencyBrakingActive)
    {
        applyEmergencyBehavior(
            leftWheelCommand,
            rightWheelCommand);
    }

    //-----------------------------------------
    // Sequence ID
    //-----------------------------------------

    const uint32_t sequenceId =
        ++m_sequenceCounter;

    //-----------------------------------------
    // Generate driver commands
    //-----------------------------------------

    MotorDriverCommand leftDriverCommand =
        generateMotorDriverCommand(
            MotorChannel::Left,
            leftWheelCommand,
            sequenceId,
            currentTimestampMs);

    MotorDriverCommand rightDriverCommand =
        generateMotorDriverCommand(
            MotorChannel::Right,
            rightWheelCommand,
            sequenceId,
            currentTimestampMs);

    //-----------------------------------------
    // Execute wheel commands
    //-----------------------------------------

    executeWheelCommands(
        leftDriverCommand,
        rightDriverCommand);

    //-----------------------------------------
    // Update runtime wheel state
    //-----------------------------------------

    updateWheelState(
        m_memory.leftWheelState,
        leftWheelCommand,
        currentTimestampMs);

    updateWheelState(
        m_memory.rightWheelState,
        rightWheelCommand,
        currentTimestampMs);

    //-----------------------------------------
    // Update runtime memory
    //-----------------------------------------

    updateRuntimeMemory(
        motionCommand,
        currentTimestampMs);

    //-----------------------------------------
    // State transition
    //-----------------------------------------

    transitionToState(
        determineMotorState(
            motionCommand));

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.executedCommandCount++;

    m_memory.motionExecutionActive =
        true;
}

//====================================================
// Runtime update loop
//====================================================

void MotorController::update(
    uint32_t currentTimestampMs)
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Skip ultra-fast updates
    //-----------------------------------------

    if (shouldSkipUpdate(currentTimestampMs))
    {
        return;
    }

    //-----------------------------------------
    // Motion timeout supervision
    //-----------------------------------------

    if (
        hasMotionTimedOut(
            currentTimestampMs))
    {
        ESP_LOGW(
            TAG,
            "Motion timeout detected");

        stopInternal();
    }

    //-----------------------------------------
    // Runtime monitoring
    //-----------------------------------------

    performRuntimeMonitoring(
        currentTimestampMs);

    //-----------------------------------------
    // Driver update
    //-----------------------------------------

    m_motorDriver.update(
        currentTimestampMs);
}

//====================================================
// Emergency stop
//====================================================

void MotorController::emergencyStop()
{
    ScopedControllerLock lock(this);

    emergencyStopInternal();
}

//====================================================
// Internal emergency stop
//====================================================

void MotorController::emergencyStopInternal()
{
    //-----------------------------------------
    // Already active
    //-----------------------------------------

    if (m_memory.emergencyStopActive)
    {
        return;
    }

    //-----------------------------------------
    // Activate runtime emergency state
    //-----------------------------------------

    m_memory.emergencyStopActive =
        true;

    m_memory.lastEmergencyTimestampMs =
        getCurrentTimestampMs();

    //-----------------------------------------
    // Driver emergency stop
    //-----------------------------------------

    m_motorDriver.emergencyStop();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::EmergencyStop);

    ESP_LOGE(
        TAG,
        "Emergency stop activated");
}

//====================================================
// Clear emergency stop
//====================================================

void MotorController::clearEmergencyStop()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Driver clear
    //-----------------------------------------

    m_motorDriver.clearEmergencyStop();

    //-----------------------------------------
    // Runtime reset
    //-----------------------------------------

    m_memory.emergencyStopActive =
        false;

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::Idle);

    ESP_LOGI(
        TAG,
        "Emergency stop cleared");
}

//====================================================
// Stop locomotion
//====================================================

void MotorController::stop()
{
    ScopedControllerLock lock(this);

    stopInternal();
}

//====================================================
// Internal stop
//====================================================

void MotorController::stopInternal()
{
    //-----------------------------------------
    // Driver stop
    //-----------------------------------------

    m_motorDriver.stopAllMotors();

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::Idle);

    //-----------------------------------------
    // Runtime flags
    //-----------------------------------------

    m_memory.motionExecutionActive =
        false;
}

//====================================================
// Runtime reset
//====================================================

void MotorController::reset()
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Stop locomotion
    //-----------------------------------------

    stopInternal();

    //-----------------------------------------
    // Driver reset
    //-----------------------------------------

    m_motorDriver.reset();

    //-----------------------------------------
    // Reset runtime memory
    //-----------------------------------------

    m_memory = {};

    //-----------------------------------------
    // Reset sequence generator
    //-----------------------------------------

    m_sequenceCounter = 0;

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::Idle);

    ESP_LOGI(
        TAG,
        "MotorController reset");
}

//====================================================
// Current runtime state
//====================================================

MotorState MotorController::getCurrentState() const
{
    return m_memory.currentState;
}

//====================================================
// Runtime memory
//====================================================

const MotorControllerMemory &
MotorController::getMemory() const
{
    return m_memory;
}

//====================================================
// Fault state
//====================================================

bool MotorController::hasFault() const
{
    return m_memory.faultActive;
}

//====================================================
// Emergency state
//====================================================

bool MotorController::isEmergencyStopActive() const
{
    return m_memory.emergencyStopActive;
}

//====================================================
// Driver health validation
//====================================================

bool MotorController::validateDriverHealth() const
{
    //-----------------------------------------
    // Ready state
    //-----------------------------------------

    if (!m_motorDriver.isReady())
    {
        return false;
    }

    //-----------------------------------------
    // Driver fault
    //-----------------------------------------

    if (m_motorDriver.hasFault())
    {
        return false;
    }

    return true;
}

//====================================================
// Motion validation
//====================================================

bool MotorController::validateMotionCommand(
    const MotionCommand &motionCommand) const
{
    //-----------------------------------------
    // Finite validation
    //-----------------------------------------

    if (!std::isfinite(
            motionCommand.leftWheelSpeed))
    {
        return false;
    }

    if (!std::isfinite(
            motionCommand.rightWheelSpeed))
    {
        return false;
    }

    //-----------------------------------------
    // Range validation
    //-----------------------------------------

    if (
        motionCommand.leftWheelSpeed < -1.0f ||
        motionCommand.leftWheelSpeed > 1.0f)
    {
        return false;
    }

    if (
        motionCommand.rightWheelSpeed < -1.0f ||
        motionCommand.rightWheelSpeed > 1.0f)
    {
        return false;
    }

    //-----------------------------------------
    // Confidence validation
    //-----------------------------------------

    if (
        motionCommand.motionConfidence <
        0.0f)
    {
        return false;
    }

    return true;
}

//====================================================
// Motion safety validation
//====================================================

bool MotorController::validateMotionSafety(const MotionCommand &motionCommand) const
{
    //-----------------------------------------
    // Future:
    // wheel divergence
    // instability
    // unsafe acceleration
    //-----------------------------------------

    (void)motionCommand;

    return true;
}

//====================================================
// Generate wheel commands
//====================================================

void MotorController::generateWheelCommands(
    const MotionCommand &motionCommand,
    WheelCommand &leftWheelCommand,
    WheelCommand &rightWheelCommand)
{
    leftWheelCommand =
        generateLeftWheelCommand(
            motionCommand);

    rightWheelCommand =
        generateRightWheelCommand(
            motionCommand);
}

//====================================================
// Left wheel command generation
//====================================================

WheelCommand
MotorController::generateLeftWheelCommand(
    const MotionCommand &motionCommand)
{
    WheelCommand command;

    //-----------------------------------------
    // Speed
    //-----------------------------------------

    const float speed =
        motionCommand.leftWheelSpeed;

    command.speedPercent =
        std::fabs(speed);

    //-----------------------------------------
    // Direction
    //-----------------------------------------

    if (speed > 0.0f)
    {
        command.direction =
            MotorDirection::Forward;
    }
    else if (speed < 0.0f)
    {
        command.direction =
            MotorDirection::Reverse;
    }
    else
    {
        command.direction =
            MotorDirection::Stop;
    }

    //-----------------------------------------
    // Brake mode
    //-----------------------------------------

    command.brakeMode =
        motionCommand.brakingActive
            ? BrakeMode::Active
            : BrakeMode::Coast;

    //-----------------------------------------
    // Emergency
    //-----------------------------------------

    command.emergencyBrake =
        motionCommand.emergencyBrakingActive;

    //-----------------------------------------
    // Runtime timestamp
    //-----------------------------------------

    command.timestampMs =
        getCurrentTimestampMs();

    return command;
}

//====================================================
// Right wheel command generation
//====================================================

WheelCommand
MotorController::generateRightWheelCommand(
    const MotionCommand &motionCommand)
{
    WheelCommand command;

    //-----------------------------------------
    // Speed
    //-----------------------------------------

    const float speed =
        motionCommand.rightWheelSpeed;

    command.speedPercent =
        std::fabs(speed);

    //-----------------------------------------
    // Direction
    //-----------------------------------------

    if (speed > 0.0f)
    {
        command.direction =
            MotorDirection::Forward;
    }
    else if (speed < 0.0f)
    {
        command.direction =
            MotorDirection::Reverse;
    }
    else
    {
        command.direction =
            MotorDirection::Stop;
    }

    //-----------------------------------------
    // Brake mode
    //-----------------------------------------

    command.brakeMode =
        motionCommand.brakingActive
            ? BrakeMode::Active
            : BrakeMode::Coast;

    //-----------------------------------------
    // Emergency
    //-----------------------------------------

    command.emergencyBrake =
        motionCommand.emergencyBrakingActive;

    //-----------------------------------------
    // Runtime timestamp
    //-----------------------------------------

    command.timestampMs =
        getCurrentTimestampMs();

    return command;
}

//====================================================
// Wheel synchronization
//====================================================

void MotorController::applyWheelSynchronization(
    WheelCommand &leftWheelCommand,
    WheelCommand &rightWheelCommand)
{
    if (!m_config.enableWheelSynchronization)
    {
        return;
    }

    //-----------------------------------------
    // Wheel divergence protection
    //-----------------------------------------

    const float speedDifference =
        std::fabs(
            leftWheelCommand.speedPercent -
            rightWheelCommand.speedPercent);

    //-----------------------------------------
    // Synchronization threshold
    //-----------------------------------------

    constexpr float maximumAllowedDifference =
        0.6f;

    if (
        speedDifference >
        maximumAllowedDifference)
    {
        const float average =
            (leftWheelCommand.speedPercent +
             rightWheelCommand.speedPercent) *
            0.5f;

        leftWheelCommand.speedPercent =
            average;

        rightWheelCommand.speedPercent =
            average;
    }

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    m_memory.wheelSynchronizationActive =
        true;
}

//====================================================
// Motion smoothing
//====================================================

void MotorController::applyMotionSmoothing(
    WheelCommand &leftWheelCommand,
    WheelCommand &rightWheelCommand,
    uint32_t currentTimestampMs)
{
    if (!m_config.enableMotionSmoothing)
    {
        return;
    }

    //-----------------------------------------
    // Left wheel
    //-----------------------------------------

    applyAccelerationLimits(
        leftWheelCommand,
        m_memory.leftWheelState,
        currentTimestampMs);

    applyDecelerationLimits(
        leftWheelCommand,
        m_memory.leftWheelState,
        currentTimestampMs);

    //-----------------------------------------
    // Right wheel
    //-----------------------------------------

    applyAccelerationLimits(
        rightWheelCommand,
        m_memory.rightWheelState,
        currentTimestampMs);

    applyDecelerationLimits(
        rightWheelCommand,
        m_memory.rightWheelState,
        currentTimestampMs);
}

//====================================================
// Acceleration limiting
//====================================================

void MotorController::applyAccelerationLimits(
    WheelCommand &wheelCommand,
    WheelState &wheelState,
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Delta time
    //-----------------------------------------

    const float deltaTimeSec =
        static_cast<float>(
            currentTimestampMs -
            wheelState.lastUpdateTimestampMs) /
        1000.0f;

    if (deltaTimeSec <= 0.0f)
    {
        return;
    }

    //-----------------------------------------
    // Maximum allowed acceleration
    //-----------------------------------------

    const float maximumDelta =
        m_config.maximumAccelerationPercentPerSec *
        deltaTimeSec;

    //-----------------------------------------
    // Speed delta
    //-----------------------------------------

    const float delta =
        wheelCommand.speedPercent -
        wheelState.currentSpeedPercent;

    //-----------------------------------------
    // Clamp acceleration
    //-----------------------------------------

    if (delta > maximumDelta)
    {
        wheelCommand.speedPercent =
            wheelState.currentSpeedPercent +
            maximumDelta;
    }
}

//====================================================
// Deceleration limiting
//====================================================

void MotorController::applyDecelerationLimits(
    WheelCommand &wheelCommand,
    WheelState &wheelState,
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Delta time
    //-----------------------------------------

    const float deltaTimeSec =
        static_cast<float>(
            currentTimestampMs -
            wheelState.lastUpdateTimestampMs) /
        1000.0f;

    if (deltaTimeSec <= 0.0f)
    {
        return;
    }

    //-----------------------------------------
    // Maximum allowed deceleration
    //-----------------------------------------

    const float maximumDelta =
        m_config.maximumDecelerationPercentPerSec *
        deltaTimeSec;

    //-----------------------------------------
    // Deceleration delta
    //-----------------------------------------

    const float delta =
        wheelState.currentSpeedPercent -
        wheelCommand.speedPercent;

    //-----------------------------------------
    // Clamp deceleration
    //-----------------------------------------

    if (delta > maximumDelta)
    {
        wheelCommand.speedPercent =
            wheelState.currentSpeedPercent -
            maximumDelta;
    }
}

//====================================================
// Startup boost
//====================================================

void MotorController::applyStartupBoost(
    WheelCommand &wheelCommand,
    const WheelState &wheelState)
{
    if (!m_config.enableStartupBoost)
    {
        return;
    }

    //-----------------------------------------
    // Startup transition
    //-----------------------------------------

    if (
        wheelState.currentSpeedPercent <
            0.01f &&
        wheelCommand.speedPercent > 0.0f)
    {
        wheelCommand.speedPercent =
            std::max(
                wheelCommand.speedPercent,
                m_config.minimumEffectiveSpeedPercent);
    }
}

//====================================================
// Deadzone compensation
//====================================================

void MotorController::applyDeadzoneCompensation(
    WheelCommand &wheelCommand)
{
    //-----------------------------------------
    // Deadzone
    //-----------------------------------------

    if (
        wheelCommand.speedPercent > 0.0f &&
        wheelCommand.speedPercent <
            m_config.minimumEffectiveSpeedPercent)
    {
        wheelCommand.speedPercent =
            m_config.minimumEffectiveSpeedPercent;
    }
}

//====================================================
// Coordinated braking
//====================================================

void MotorController::applyCoordinatedBraking(
    WheelCommand &leftWheelCommand,
    WheelCommand &rightWheelCommand)
{
    if (!m_config.enableCoordinatedBraking)
    {
        return;
    }

    //-----------------------------------------
    // Both wheels stopped
    //-----------------------------------------

    if (
        leftWheelCommand.speedPercent <= 0.01f &&
        rightWheelCommand.speedPercent <= 0.01f)
    {
        leftWheelCommand.brakeMode =
            BrakeMode::Active;

        rightWheelCommand.brakeMode =
            BrakeMode::Active;

        m_memory.coordinatedBrakingActive =
            true;
    }
    else
    {
        m_memory.coordinatedBrakingActive =
            false;
    }
}

//====================================================
// Emergency behavior
//====================================================

void MotorController::applyEmergencyBehavior(
    WheelCommand &leftWheelCommand,
    WheelCommand &rightWheelCommand)
{
    //-----------------------------------------
    // Emergency braking
    //-----------------------------------------

    leftWheelCommand.speedPercent =
        0.0f;

    rightWheelCommand.speedPercent =
        0.0f;

    //-----------------------------------------
    // Emergency brake flag
    //-----------------------------------------

    leftWheelCommand.emergencyBrake =
        true;

    rightWheelCommand.emergencyBrake =
        true;

    //-----------------------------------------
    // Active braking
    //-----------------------------------------

    leftWheelCommand.brakeMode =
        BrakeMode::Active;

    rightWheelCommand.brakeMode =
        BrakeMode::Active;
}

//====================================================
// Reverse transition validation
//====================================================

bool MotorController::validateReverseTransition(
    const WheelCommand &wheelCommand,
    const WheelState &wheelState) const
{
    //-----------------------------------------
    // Safe reverse disabled
    //-----------------------------------------

    if (!m_config.enableSafeReverseTransition)
    {
        return true;
    }

    //-----------------------------------------
    // Current direction
    //-----------------------------------------

    const MotorDirection currentDirection =
        wheelState.currentDirection;

    //-----------------------------------------
    // New direction
    //-----------------------------------------

    const MotorDirection newDirection =
        wheelCommand.direction;

    //-----------------------------------------
    // Prevent instant reversal
    //-----------------------------------------

    if (
        currentDirection ==
            MotorDirection::Forward &&
        newDirection ==
            MotorDirection::Reverse)
    {
        return (
            wheelState.currentSpeedPercent <
            0.1f);
    }

    //-----------------------------------------
    // Reverse → forward
    //-----------------------------------------

    if (
        currentDirection ==
            MotorDirection::Reverse &&
        newDirection ==
            MotorDirection::Forward)
    {
        return (
            wheelState.currentSpeedPercent <
            0.1f);
    }

    return true;
}

//====================================================
// Driver command generation
//====================================================

MotorDriverCommand
MotorController::generateMotorDriverCommand(
    MotorChannel channel,
    const WheelCommand &wheelCommand,
    uint32_t sequenceId,
    uint32_t timestampMs) const
{
    MotorDriverCommand command;

    //-----------------------------------------
    // Channel
    //-----------------------------------------

    command.channel =
        channel;

    //-----------------------------------------
    // Direction
    //-----------------------------------------

    command.direction =
        wheelCommand.direction;

    //-----------------------------------------
    // PWM percentage
    //-----------------------------------------

    command.pwmPercent =
        wheelCommand.speedPercent;

    //-----------------------------------------
    // Brake mode
    //-----------------------------------------

    command.brakeMode =
        wheelCommand.brakeMode;

    //-----------------------------------------
    // Enable
    //-----------------------------------------

    command.enabled =
        wheelCommand.enabled;

    //-----------------------------------------
    // Emergency stop
    //-----------------------------------------

    command.emergencyStop =
        wheelCommand.emergencyBrake;

    //-----------------------------------------
    // Sequence ID
    //-----------------------------------------

    command.sequenceId =
        sequenceId;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    command.timestampMs =
        timestampMs;

    return command;
}

//====================================================
// Execute wheel commands
//====================================================

void MotorController::executeWheelCommands(
    const MotorDriverCommand &leftCommand,
    const MotorDriverCommand &rightCommand)
{
    //-----------------------------------------
    // Synchronized driver execution
    //-----------------------------------------

    m_motorDriver.executeDualCommand(
        leftCommand,
        rightCommand);
}

//====================================================
// Runtime state transition
//====================================================

void MotorController::transitionToState(
    MotorState newState)
{
    //-----------------------------------------
    // Validate transition
    //-----------------------------------------

    if (
        !validateStateTransition(
            m_memory.currentState,
            newState))
    {
        return;
    }

    //-----------------------------------------
    // Previous state
    //-----------------------------------------

    m_memory.previousState =
        m_memory.currentState;

    //-----------------------------------------
    // Current state
    //-----------------------------------------

    m_memory.currentState =
        newState;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    m_memory.lastStateTransitionTimestampMs =
        getCurrentTimestampMs();
}

//====================================================
// State transition validation
//====================================================

bool MotorController::validateStateTransition(
    MotorState currentState,
    MotorState newState) const
{
    //-----------------------------------------
    // Fault protection
    //-----------------------------------------

    if (
        currentState ==
            MotorState::Fault &&
        newState !=
            MotorState::Idle)
    {
        return false;
    }

    //-----------------------------------------
    // Emergency protection
    //-----------------------------------------

    if (
        currentState ==
            MotorState::EmergencyStop &&
        newState !=
            MotorState::Idle)
    {
        return false;
    }

    return true;
}

//====================================================
// Runtime state determination
//====================================================

MotorState
MotorController::determineMotorState(
    const MotionCommand &motionCommand) const
{
    //-----------------------------------------
    // Emergency braking
    //-----------------------------------------

    if (
        motionCommand.emergencyBrakingActive)
    {
        return MotorState::EmergencyStop;
    }

    //-----------------------------------------
    // Braking
    //-----------------------------------------

    if (
        motionCommand.brakingActive)
    {
        return MotorState::Braking;
    }

    //-----------------------------------------
    // Average wheel speed
    //-----------------------------------------

    const float averageSpeed =
        (std::fabs(
             motionCommand.leftWheelSpeed) +
         std::fabs(
             motionCommand.rightWheelSpeed)) *
        0.5f;

    //-----------------------------------------
    // Idle
    //-----------------------------------------

    if (averageSpeed <= 0.01f)
    {
        return MotorState::Idle;
    }

    //-----------------------------------------
    // Reverse
    //-----------------------------------------

    if (
        motionCommand.reverseMotionActive)
    {
        return MotorState::Reverse;
    }

    //-----------------------------------------
    // Accelerating
    //-----------------------------------------

    if (averageSpeed < 0.3f)
    {
        return MotorState::Accelerating;
    }

    //-----------------------------------------
    // Cruising
    //-----------------------------------------

    return MotorState::Cruising;
}

//====================================================
// Fault handling
//====================================================

void MotorController::handleFault(
    const char *reason)
{
    //-----------------------------------------
    // Already faulted
    //-----------------------------------------

    if (m_memory.faultActive)
    {
        return;
    }

    //-----------------------------------------
    // Runtime fault state
    //-----------------------------------------

    m_memory.faultActive =
        true;

    //-----------------------------------------
    // Runtime state
    //-----------------------------------------

    transitionToState(
        MotorState::Fault);

    //-----------------------------------------
    // Emergency stop
    //-----------------------------------------

    m_motorDriver.emergencyStop();

    //-----------------------------------------
    // Logging
    //-----------------------------------------

    ESP_LOGE(
        TAG,
        "MotorController fault: %s",
        reason);
}

//====================================================
// Motion timeout supervision
//====================================================

bool MotorController::hasMotionTimedOut(
    uint32_t currentTimestampMs) const
{
    //-----------------------------------------
    // No command received yet
    //-----------------------------------------

    if (
        m_memory.lastCommandTimestampMs ==
        0)
    {
        return false;
    }

    //-----------------------------------------
    // Timeout validation
    //-----------------------------------------

    return (
        (
            currentTimestampMs -
            m_memory.lastCommandTimestampMs) >
        m_config.motorCommandTimeoutMs);
}

//====================================================
// Runtime monitoring
//====================================================

void MotorController::performRuntimeMonitoring(
    uint32_t currentTimestampMs)
{
    (void)currentTimestampMs;

    //-----------------------------------------
    // Driver fault monitoring
    //-----------------------------------------

    if (m_motorDriver.hasFault())
    {
        handleFault(
            "Runtime driver fault");
    }
}

//====================================================
// Update wheel runtime state
//====================================================

void MotorController::updateWheelState(
    WheelState &wheelState,
    const WheelCommand &wheelCommand,
    uint32_t currentTimestampMs)
{
    //-----------------------------------------
    // Speed
    //-----------------------------------------

    wheelState.currentSpeedPercent =
        wheelCommand.speedPercent;

    //-----------------------------------------
    // Target speed
    //-----------------------------------------

    wheelState.targetSpeedPercent =
        wheelCommand.speedPercent;

    //-----------------------------------------
    // Direction
    //-----------------------------------------

    wheelState.currentDirection =
        wheelCommand.direction;

    //-----------------------------------------
    // Brake state
    //-----------------------------------------

    wheelState.currentBrakeMode =
        wheelCommand.brakeMode;

    //-----------------------------------------
    // Braking runtime
    //-----------------------------------------

    wheelState.brakingActive =
        (wheelCommand.brakeMode !=
         BrakeMode::Coast);

    //-----------------------------------------
    // Emergency runtime
    //-----------------------------------------

    wheelState.emergencyBrakeActive =
        wheelCommand.emergencyBrake;

    //-----------------------------------------
    // Enabled
    //-----------------------------------------

    wheelState.enabled =
        wheelCommand.enabled;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    wheelState.lastUpdateTimestampMs =
        currentTimestampMs;
}

//====================================================
// Runtime memory update
//====================================================

void MotorController::updateRuntimeMemory(
    const MotionCommand &motionCommand,
    uint32_t currentTimestampMs)
{
    (void)motionCommand;

    //-----------------------------------------
    // Timestamp
    //-----------------------------------------

    m_memory.lastCommandTimestampMs =
        currentTimestampMs;

    //-----------------------------------------
    // Sequence ID
    //-----------------------------------------

    m_memory.currentSequenceId =
        m_sequenceCounter;

    //-----------------------------------------
    // Runtime execution
    //-----------------------------------------

    m_memory.motionExecutionActive =
        true;
}

//====================================================
// Skip ultra-fast updates
//====================================================

bool MotorController::shouldSkipUpdate(uint32_t currentTimestampMs) const
{
    //-----------------------------------------
    // Minimum update interval
    //-----------------------------------------

    constexpr uint32_t minimumUpdateIntervalMs =
        2;

    return (
        (
            currentTimestampMs -
            m_memory.lastSynchronizationTimestampMs) <
        minimumUpdateIntervalMs);
}

//====================================================
// Timestamp utility
//====================================================

uint32_t MotorController::getCurrentTimestampMs() const
{
    return static_cast<uint32_t>(
        esp_timer_get_time() / 1000ULL);
}