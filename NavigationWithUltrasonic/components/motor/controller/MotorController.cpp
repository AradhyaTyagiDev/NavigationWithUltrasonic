//====================================================
// File: MotorController.cpp
//====================================================

#include "MotorController.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"

static const char *TAG =
    "MotorController";

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
    // Driver validation
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
    // Initial state
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

    stopInternal();

    m_motorDriver.shutdown();

    transitionToState(
        MotorState::Idle);

    ESP_LOGI(
        TAG,
        "MotorController shutdown");
}

//====================================================
// Main execution entry
//====================================================

void MotorController::executeMotion(
    const MotionCommand &motionCommand)
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Emergency state protection
    //-----------------------------------------

    if (m_memory.emergencyStopActive)
    {
        return;
    }

    //-----------------------------------------
    // Driver health validation
    //-----------------------------------------

    if (!m_motorDriver.isReady())
    {
        handleFault(
            "Driver not ready");

        return;
    }

    if (m_motorDriver.hasFault())
    {
        handleFault(
            "Driver fault");

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
    // Execute internally
    //-----------------------------------------

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
    // Timestamp
    //-----------------------------------------

    const uint32_t timestampMs =
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
        timestampMs);

    //-----------------------------------------
    // Braking coordination
    //-----------------------------------------

    applyCoordinatedBraking(
        leftWheelCommand,
        rightWheelCommand);

    //-----------------------------------------
    // Emergency handling
    //-----------------------------------------

    if (motionCommand.emergencyStop)
    {
        applyEmergencyBehavior(
            leftWheelCommand,
            rightWheelCommand);
    }

    //-----------------------------------------
    // Driver commands
    //-----------------------------------------

    const uint32_t sequenceId =
        ++m_sequenceCounter;

    MotorDriverCommand leftDriverCommand =
        generateMotorDriverCommand(
            MotorChannel::Left,
            leftWheelCommand,
            sequenceId,
            timestampMs);

    MotorDriverCommand rightDriverCommand =
        generateMotorDriverCommand(
            MotorChannel::Right,
            rightWheelCommand,
            sequenceId,
            timestampMs);

    //-----------------------------------------
    // Execute
    //-----------------------------------------

    executeWheelCommands(
        leftDriverCommand,
        rightDriverCommand);

    //-----------------------------------------
    // Update wheel state
    //-----------------------------------------

    updateWheelState(
        m_memory.leftWheelState,
        leftWheelCommand,
        timestampMs);

    updateWheelState(
        m_memory.rightWheelState,
        rightWheelCommand,
        timestampMs);

    //-----------------------------------------
    // Update runtime memory
    //-----------------------------------------

    updateRuntimeMemory(
        motionCommand,
        timestampMs);

    //-----------------------------------------
    // Update state
    //-----------------------------------------

    transitionToState(
        determineMotorState(
            motionCommand));

    //-----------------------------------------
    // Statistics
    //-----------------------------------------

    m_memory.executedCommandCount++;
}

//====================================================
// Runtime update loop
//====================================================

void MotorController::update(
    uint32_t currentTimestampMs)
{
    ScopedControllerLock lock(this);

    //-----------------------------------------
    // Motion timeout
    //-----------------------------------------

    if (
        hasMotionTimedOut(
            currentTimestampMs))
    {
        ESP_LOGW(
            TAG,
            "Motion timeout");

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
    // Activate emergency state
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

    ESP_LOGW(
        TAG,
        "Emergency stop activated");
}

//====================================================
// Clear emergency stop
//====================================================

void MotorController::clearEmergencyStop()
{
    ScopedControllerLock lock(this);

    m_motorDriver.clearEmergencyStop();

    m_memory.emergencyStopActive =
        false;

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
    m_motorDriver.stopAllMotors();

    transitionToState(
        MotorState::Idle);

    m_memory.motionExecutionActive =
        false;
}

//====================================================
// Reset runtime state
//====================================================

void MotorController::reset()
{
    ScopedControllerLock lock(this);

    stopInternal();

    m_memory = {};

    m_sequenceCounter = 0;

    transitionToState(
        MotorState::Idle);

    ESP_LOGI(
        TAG,
        "MotorController reset");
}

//====================================================
// Runtime state
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
// Motion validation
//====================================================

bool MotorController::validateMotionCommand(
    const MotionCommand &motionCommand) const
{
    //-----------------------------------------
    // Finite validation
    //-----------------------------------------

    if (!std::isfinite(
            motionCommand.leftWheelSpeedPercent))
    {
        return false;
    }

    if (!std::isfinite(
            motionCommand.rightWheelSpeedPercent))
    {
        return false;
    }

    //-----------------------------------------
    // Range validation
    //-----------------------------------------

    if (
        motionCommand.leftWheelSpeedPercent < -1.0f ||
        motionCommand.leftWheelSpeedPercent > 1.0f)
    {
        return false;
    }

    if (
        motionCommand.rightWheelSpeedPercent < -1.0f ||
        motionCommand.rightWheelSpeedPercent > 1.0f)
    {
        return false;
    }

    return true;
}

//====================================================
// Motion safety validation
//====================================================

bool MotorController::validateMotionSafety(
    const MotionCommand &motionCommand) const
{
    //-----------------------------------------
    // Future:
    // acceleration spikes
    // unsafe reversals
    // wheel divergence
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
// Left wheel command
//====================================================

WheelCommand
MotorController::generateLeftWheelCommand(
    const MotionCommand &motionCommand)
{
    WheelCommand command;

    const float speed =
        motionCommand.leftWheelSpeedPercent;

    command.speedPercent =
        std::fabs(speed);

    command.direction =
        (speed >= 0.0f)
            ? MotorDirection::Forward
            : MotorDirection::Reverse;

    command.timestampMs =
        getCurrentTimestampMs();

    return command;
}

//====================================================
// Right wheel command
//====================================================

WheelCommand
MotorController::generateRightWheelCommand(
    const MotionCommand &motionCommand)
{
    WheelCommand command;

    const float speed =
        motionCommand.rightWheelSpeedPercent;

    command.speedPercent =
        std::fabs(speed);

    command.direction =
        (speed >= 0.0f)
            ? MotorDirection::Forward
            : MotorDirection::Reverse;

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
    // Prevent wheel divergence
    //-----------------------------------------

    const float difference =
        std::fabs(
            leftWheelCommand.speedPercent -
            rightWheelCommand.speedPercent);

    if (difference > 0.5f)
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

    applyAccelerationLimits(
        leftWheelCommand,
        m_memory.leftWheelState,
        currentTimestampMs);

    applyAccelerationLimits(
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
    const float deltaTimeSec =
        static_cast<float>(
            currentTimestampMs -
            wheelState.lastUpdateTimestampMs) /
        1000.0f;

    if (deltaTimeSec <= 0.0f)
    {
        return;
    }

    const float maximumDelta =
        m_config.maximumAccelerationPercentPerSec *
        deltaTimeSec;

    const float delta =
        wheelCommand.speedPercent -
        wheelState.currentSpeedPercent;

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
    const float deltaTimeSec =
        static_cast<float>(
            currentTimestampMs -
            wheelState.lastUpdateTimestampMs) /
        1000.0f;

    if (deltaTimeSec <= 0.0f)
    {
        return;
    }

    const float maximumDelta =
        m_config.maximumDecelerationPercentPerSec *
        deltaTimeSec;

    const float delta =
        wheelState.currentSpeedPercent -
        wheelCommand.speedPercent;

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
    leftWheelCommand.emergencyBrake =
        true;

    rightWheelCommand.emergencyBrake =
        true;

    leftWheelCommand.speedPercent =
        0.0f;

    rightWheelCommand.speedPercent =
        0.0f;

    leftWheelCommand.brakeMode =
        BrakeMode::Active;

    rightWheelCommand.brakeMode =
        BrakeMode::Active;
}

//====================================================
// Reverse transition protection
//====================================================

bool MotorController::validateReverseTransition(
    const WheelCommand &wheelCommand,
    const WheelState &wheelState) const
{
    //-----------------------------------------
    // Prevent rapid reverse switching
    //-----------------------------------------

    if (
        wheelState.currentDirection ==
            MotorDirection::Forward &&
        wheelCommand.direction ==
            MotorDirection::Reverse)
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

    command.channel =
        channel;

    command.direction =
        wheelCommand.direction;

    command.pwmPercent =
        wheelCommand.speedPercent;

    command.brakeMode =
        wheelCommand.brakeMode;

    command.enabled =
        wheelCommand.enabled;

    command.emergencyStop =
        wheelCommand.emergencyBrake;

    command.sequenceId =
        sequenceId;

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
    if (
        !validateStateTransition(
            m_memory.currentState,
            newState))
    {
        return;
    }

    m_memory.previousState =
        m_memory.currentState;

    m_memory.currentState =
        newState;

    m_memory.lastStateTransitionTimestampMs =
        getCurrentTimestampMs();
}

//====================================================
// State validation
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
        newState != MotorState::Idle)
    {
        return false;
    }

    //-----------------------------------------
    // Emergency protection
    //-----------------------------------------

    if (
        currentState ==
            MotorState::EmergencyStop &&
        newState != MotorState::Idle)
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
    if (motionCommand.emergencyStop)
    {
        return MotorState::EmergencyStop;
    }

    const float averageSpeed =
        (std::fabs(
             motionCommand.leftWheelSpeedPercent) +
         std::fabs(
             motionCommand.rightWheelSpeedPercent)) *
        0.5f;

    if (averageSpeed <= 0.01f)
    {
        return MotorState::Idle;
    }

    if (averageSpeed < 0.3f)
    {
        return MotorState::Accelerating;
    }

    return MotorState::Cruising;
}

//====================================================
// Fault handling
//====================================================

void MotorController::handleFault(
    const char *reason)
{
    ESP_LOGE(
        TAG,
        "Fault: %s",
        reason);

    m_memory.faultActive =
        true;

    transitionToState(
        MotorState::Fault);

    m_motorDriver.emergencyStop();
}

//====================================================
// Motion timeout supervision
//====================================================

bool MotorController::hasMotionTimedOut(
    uint32_t currentTimestampMs) const
{
    if (
        m_memory.lastCommandTimestampMs == 0)
    {
        return false;
    }

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
            "Driver runtime fault");
    }
}

//====================================================
// Wheel runtime update
//====================================================

void MotorController::updateWheelState(
    WheelState &wheelState,
    const WheelCommand &wheelCommand,
    uint32_t currentTimestampMs)
{
    wheelState.currentSpeedPercent =
        wheelCommand.speedPercent;

    wheelState.targetSpeedPercent =
        wheelCommand.speedPercent;

    wheelState.currentDirection =
        wheelCommand.direction;

    wheelState.currentBrakeMode =
        wheelCommand.brakeMode;

    wheelState.brakingActive =
        (wheelCommand.brakeMode !=
         BrakeMode::Coast);

    wheelState.emergencyBrakeActive =
        wheelCommand.emergencyBrake;

    wheelState.enabled =
        wheelCommand.enabled;

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

    m_memory.lastCommandTimestampMs =
        currentTimestampMs;

    m_memory.currentSequenceId =
        m_sequenceCounter;

    m_memory.motionExecutionActive =
        true;
}

//====================================================
// Timestamp utility
//====================================================

uint32_t MotorController::getCurrentTimestampMs() const
{
    return static_cast<uint32_t>(
        esp_log_timestamp());
}