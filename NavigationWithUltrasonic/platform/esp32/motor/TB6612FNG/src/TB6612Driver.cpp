//====================================================
// File: TB6612Driver.cpp
//====================================================

#include "motor/TB6612FNG/include/TB6612Driver.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include <algorithm>
#include <cmath>

//====================================================
// Constants
//====================================================

namespace
{
    constexpr const char *TAG =
        "TB6612Driver";
}

//====================================================
// Constructor
//====================================================

TB6612Driver::TB6612Driver(
    const TB6612DriverConfig &config)
    : m_config(config)
{
    //-----------------------------------------
    // Driver capabilities
    //-----------------------------------------

    m_capabilities.supportsPWM = true;

    m_capabilities.supportsReverse = true;

    m_capabilities.supportsDualChannel = true;

    m_capabilities.supportsActiveBraking = true;

    m_capabilities.supportsEmergencyStop = true;
}

//====================================================
// Initialization
//====================================================

bool TB6612Driver::initialize()
{
    //-----------------------------------------
    // Initialize GPIO
    //-----------------------------------------

    if (!initializeGPIO())
    {
        ESP_LOGE(
            TAG,
            "Failed to initialize GPIO");

        return false;
    }

    //-----------------------------------------
    // Initialize PWM
    //-----------------------------------------

    if (!initializePWM())
    {
        ESP_LOGE(
            TAG,
            "Failed to initialize PWM");

        return false;
    }

    //-----------------------------------------
    // Safe startup state
    //-----------------------------------------

    stopAllMotors();

    setStandbyMode(false);

    //-----------------------------------------
    // Update memory
    //-----------------------------------------

    m_memory.initialized = true;

    transitionToState(MotorDriverState::Ready);

    ESP_LOGI(
        TAG,
        "TB6612 driver initialized");

    return true;
}

//====================================================
// Shutdown
//====================================================

void TB6612Driver::shutdown()
{
    stopAllMotors();

    setStandbyMode(false);

    transitionToState(MotorDriverState::Uninitialized);

    m_memory.initialized = false;

    ESP_LOGI(
        TAG,
        "TB6612 driver shutdown");
}

//====================================================
// GPIO Initialization
//====================================================

bool TB6612Driver::initializeGPIO()
{
    //-----------------------------------------
    // Configure output pins
    //-----------------------------------------

    const gpio_config_t config =
        {
            .pin_bit_mask =
                (1ULL << m_config.leftMotorIN1Pin) |
                (1ULL << m_config.leftMotorIN2Pin) |
                (1ULL << m_config.rightMotorIN1Pin) |
                (1ULL << m_config.rightMotorIN2Pin) |
                (1ULL << m_config.standbyPin),

            .mode = GPIO_MODE_OUTPUT,

            .pull_up_en = GPIO_PULLUP_DISABLE,

            .pull_down_en = GPIO_PULLDOWN_DISABLE,

            .intr_type = GPIO_INTR_DISABLE};

    const esp_err_t result =
        gpio_config(&config);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "GPIO configuration failed");

        return false;
    }

    //-----------------------------------------
    // Safe default states
    //-----------------------------------------

    gpio_set_level(
        m_config.leftMotorIN1Pin,
        0);

    gpio_set_level(
        m_config.leftMotorIN2Pin,
        0);

    gpio_set_level(
        m_config.rightMotorIN1Pin,
        0);

    gpio_set_level(
        m_config.rightMotorIN2Pin,
        0);

    gpio_set_level(
        m_config.standbyPin,
        0);

    return true;
}

//====================================================
// PWM Initialization
//====================================================

bool TB6612Driver::initializePWM()
{
    //-----------------------------------------
    // Timer configuration
    //-----------------------------------------

    const ledc_timer_config_t timerConfig =
        {
            .speed_mode =
                m_config.ledcMode,

            .duty_resolution =
                m_config.pwmResolution,

            .timer_num =
                m_config.pwmTimer,

            .freq_hz =
                static_cast<int>(
                    m_config.pwmFrequencyHz),

            .clk_cfg =
                LEDC_AUTO_CLK,

#if ESP_IDF_VERSION_MAJOR >= 5
            .deconfigure = false
#endif
        };

    if (
        ledc_timer_config(
            &timerConfig) != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "PWM timer config failed");

        return false;
    }

    //-----------------------------------------
    // Left PWM channel
    //-----------------------------------------

    ledc_channel_config_t leftChannel =
        {
            .gpio_num =
                m_config.leftMotorPWMPin,

            .speed_mode =
                m_config.ledcMode,

            .channel =
                m_config.leftPWMChannel,

            .intr_type =
                LEDC_INTR_DISABLE,

            .timer_sel =
                m_config.pwmTimer,

            .duty = 0,

            .hpoint = 0,

#if ESP_IDF_VERSION_MAJOR >= 5
            .sleep_mode =
                LEDC_SLEEP_MODE_NO_ALIVE_NO_PD
#endif
        };

    //-----------------------------------------
    // Right PWM channel
    //-----------------------------------------

    ledc_channel_config_t rightChannel =
        {
            .gpio_num =
                m_config.rightMotorPWMPin,

            .speed_mode =
                m_config.ledcMode,

            .channel =
                m_config.rightPWMChannel,

            .intr_type =
                LEDC_INTR_DISABLE,

            .timer_sel =
                m_config.pwmTimer,

            .duty = 0,

            .hpoint = 0,

#if ESP_IDF_VERSION_MAJOR >= 5
            .sleep_mode =
                LEDC_SLEEP_MODE_NO_ALIVE_NO_PD
#endif
        };

    if (
        ledc_channel_config(
            &leftChannel) != ESP_OK)
    {
        return false;
    }

    if (
        ledc_channel_config(
            &rightChannel) != ESP_OK)
    {
        return false;
    }

    return true;
}

//====================================================
// Execute Command
//====================================================
void TB6612Driver::executeCommandInternal(const MotorDriverCommand &command)
{
    // Ready validation
    if (!isReady())
    {
        return;
    }

    // Emergency stop validation
    if (m_memory.emergencyStopActive)
    {
        return;
    }

    // Timestamp
    const uint32_t timestampMs =
        getCurrentTimestampMs();

    // Update watchdog timestamp
    m_memory.lastCommandTimestampMs =
        timestampMs;

    // Enable standby
    setStandbyMode(true);

    // Direction transition validation
    if (
        !validateDirectionTransition(
            command.channel,
            command.direction,
            timestampMs))
    {
        m_memory.rejectedCommandCount++;

        return;
    }

    // Safe reverse sequence
    if (
        m_config.enableSafeReverseSequence)
    {
        performSafeReverseSequence(
            command.channel,
            command.direction);
    }
    else
    {
        applyMotorDirection(
            command.channel,
            command.direction);
    }

    // PWM validation
    uint32_t pwmDuty =
        validatePWMDuty(
            command.pwmDuty);

    // Deadzone compensation
    pwmDuty =
        applyDeadzoneCompensation(
            pwmDuty);

    // Startup boost
    pwmDuty =
        applyStartupBoost(
            command.channel,
            pwmDuty,
            timestampMs);

    // PWM ramping
    pwmDuty =
        applyPWMRamping(
            command.channel,
            pwmDuty);

    // Apply PWM
    applyPWMDuty(
        command.channel,
        pwmDuty);

    // Update runtime memory
    updateMotorMemory(
        command.channel,
        command.direction,
        pwmDuty,
        timestampMs);

    // Transition to running
    transitionToState(
        MotorDriverState::Running);

    // Statistics
    m_memory.executedCommandCount++;
}

void TB6612Driver::executeCommand(const MotorDriverCommand &command)
{
    ScopedDriverLock lock(this);
    executeCommandInternal(command);
}

// Execute Dual Commands
void TB6612Driver::executeDualCommand(
    const MotorDriverCommand &leftCommand,
    const MotorDriverCommand &rightCommand)
{
    ScopedDriverLock lock(this);

    // Execute left
    executeCommandInternal(leftCommand);
    // Execute right
    executeCommandInternal(rightCommand);

    // Synchronized commit
    if (
        m_config
            .enableSynchronizedDualMotorUpdate)
    {
        commitSynchronizedPWMUpdate();
    }
}

//====================================================
// Stop All Motors
//====================================================

void TB6612Driver::stopAllMotors()
{
    ScopedDriverLock lock(this);

    stopMotor(MotorChannel::Left);

    stopMotor(MotorChannel::Right);

    transitionToState(MotorDriverState::Ready);
}

//====================================================
// Emergency Stop
//====================================================

void TB6612Driver::emergencyStop()
{
    ScopedDriverLock lock(this);

    applyEmergencyBrake();

    setStandbyMode(false);

    m_memory.emergencyStopActive = true;

    transitionToState(MotorDriverState::EmergencyStopped);

    m_memory.lastEmergencyBrakeTimestampMs =
        getCurrentTimestampMs();

    ESP_LOGW(
        TAG,
        "Emergency stop activated");
}

//====================================================
// Clear Emergency Stop
//====================================================

void TB6612Driver::clearEmergencyStop()
{
    ScopedDriverLock lock(this);

    // Stop motors safely
    stopMotor(MotorChannel::Left);
    stopMotor(MotorChannel::Right);

    // Clear emergency state
    m_memory.emergencyStopActive = false;

    // Re-enable standby
    setStandbyMode(true);

    // Transition state
    transitionToState(MotorDriverState::Ready);

    ESP_LOGI(TAG, "Emergency stop cleared");
}

//====================================================
// Apply Brake Mode
//====================================================

void TB6612Driver::applyBrakeMode(BrakeMode brakeMode)
{
    ScopedDriverLock lock(this);

    applyMotorBrake(
        MotorChannel::Left,
        brakeMode);

    applyMotorBrake(
        MotorChannel::Right,
        brakeMode);

    m_memory.currentBrakeMode =
        brakeMode;
}

//====================================================
// Set Motor Direction
//====================================================

void TB6612Driver::setMotorDirection(
    MotorChannel channel,
    MotorDirection direction)
{
    ScopedDriverLock lock(this);

    applyMotorDirection(
        channel,
        direction);
}

//====================================================
// Set PWM Duty
//====================================================

void TB6612Driver::setPWMDuty(
    MotorChannel channel,
    uint32_t pwmDuty)
{
    ScopedDriverLock lock(this);

    applyPWMDuty(
        channel,
        pwmDuty);
}

//====================================================
// Enable Motor
//====================================================

void TB6612Driver::enableMotor(
    MotorChannel channel)
{
    ScopedDriverLock lock(this);

    setStandbyMode(true);

    if (
        channel ==
        MotorChannel::Left)
    {
        m_memory.leftMotorEnabled = true;
    }
    else
    {
        m_memory.rightMotorEnabled = true;
    }
}

//====================================================
// Disable Motor
//====================================================

void TB6612Driver::disableMotor(
    MotorChannel channel)
{
    ScopedDriverLock lock(this);

    stopMotor(channel);

    if (
        channel ==
        MotorChannel::Left)
    {
        m_memory.leftMotorEnabled = false;
    }
    else
    {
        m_memory.rightMotorEnabled = false;
    }
}

//====================================================
// Driver State
//====================================================

bool TB6612Driver::isReady() const
{
    return (
        m_memory.initialized &&
        !m_memory.faultDetected);
}

//====================================================
// Fault State
//====================================================

bool TB6612Driver::hasFault() const
{
    return m_memory.faultDetected;
}

//====================================================
// Capabilities
//====================================================

MotorDriverCapabilities
TB6612Driver::getCapabilities() const
{
    return m_capabilities;
}

//====================================================
// Runtime Status
//====================================================

MotorDriverStatus
TB6612Driver::getStatus() const
{
    MotorDriverStatus status;

    status.state =
        m_memory.currentState;

    status.faultDetected =
        m_memory.faultDetected;

    status.emergencyStopActive =
        m_memory.emergencyStopActive;

    status.leftMotorRunning =
        (m_memory.currentLeftPWMDuty > 0);

    status.rightMotorRunning =
        (m_memory.currentRightPWMDuty > 0);

    status.currentLeftPWM =
        m_memory.currentLeftPWMDuty;

    status.currentRightPWM =
        m_memory.currentRightPWMDuty;

    status.lastUpdateTimestampMs =
        m_memory.lastUpdateTimestampMs;

    return status;
}

MotorDriverState TB6612Driver::getDriverState() const
{
    return m_memory.currentState;
}

//====================================================
// Hardware Type
//====================================================

MotorHardwareType
TB6612Driver::getHardwareType() const
{
    return MotorHardwareType::TB6612FNG;
}

//====================================================
// Standby Mode
//====================================================

void TB6612Driver::setStandbyMode(
    bool enabled)
{
    gpio_set_level(
        m_config.standbyPin,
        enabled ? 1 : 0);
}

//====================================================
// Apply Motor Direction
//====================================================

void TB6612Driver::applyMotorDirection(
    MotorChannel channel,
    MotorDirection direction)
{
    gpio_num_t in1 =
        getIN1Pin(channel);

    gpio_num_t in2 =
        getIN2Pin(channel);

    //-----------------------------------------
    // Motor inversion
    //-----------------------------------------

    bool inverted =
        isMotorInverted(channel);

    switch (direction)
    {
    case MotorDirection::Forward:

        gpio_set_level(
            in1,
            inverted ? 0 : 1);

        gpio_set_level(
            in2,
            inverted ? 1 : 0);

        break;

    case MotorDirection::Reverse:

        gpio_set_level(
            in1,
            inverted ? 1 : 0);

        gpio_set_level(
            in2,
            inverted ? 0 : 1);

        break;

    case MotorDirection::Brake:

        gpio_set_level(in1, 1);

        gpio_set_level(in2, 1);

        break;

    case MotorDirection::Stop:
    default:

        gpio_set_level(in1, 0);

        gpio_set_level(in2, 0);

        break;
    }
}

//====================================================
// Apply Motor Brake
//====================================================

void TB6612Driver::applyMotorBrake(
    MotorChannel channel,
    BrakeMode brakeMode)
{
    switch (brakeMode)
    {
    case BrakeMode::Active:

        applyMotorDirection(
            channel,
            MotorDirection::Brake);

        break;

    case BrakeMode::Emergency:

        applyMotorDirection(
            channel,
            MotorDirection::Brake);

        applyPWMDuty(
            channel,
            0);

        break;

    case BrakeMode::Coast:
    default:

        applyMotorDirection(
            channel,
            MotorDirection::Stop);

        applyPWMDuty(
            channel,
            0);

        break;
    }
}

//====================================================
// Apply PWM Duty
//====================================================

void TB6612Driver::applyPWMDuty(
    MotorChannel channel,
    uint32_t pwmDuty)
{
    const ledc_channel_t pwmChannel =
        getPWMChannel(channel);

    ledc_set_duty(
        m_config.ledcMode,
        pwmChannel,
        pwmDuty);

    commitPWMUpdate(channel);
}

//====================================================
// Validate PWM
//====================================================

uint32_t TB6612Driver::validatePWMDuty(
    uint32_t pwmDuty) const
{
    if (
        !m_config.enablePWMClamping)
    {
        return pwmDuty;
    }

    return std::clamp(
        pwmDuty,
        static_cast<uint32_t>(0),
        m_config.maximumPWMDuty);
}

//====================================================
// Deadzone Compensation
//====================================================

uint32_t
TB6612Driver::applyDeadzoneCompensation(
    uint32_t pwmDuty) const
{
    if (
        !m_config.enableDeadzoneCompensation)
    {
        return pwmDuty;
    }

    if (pwmDuty == 0)
    {
        return 0;
    }

    if (
        pwmDuty <
        m_config.minimumEffectivePWMDuty)
    {
        return m_config.minimumEffectivePWMDuty;
    }

    return pwmDuty;
}

//====================================================
// Startup Boost
//====================================================

uint32_t TB6612Driver::applyStartupBoost(
    MotorChannel channel,
    uint32_t pwmDuty,
    uint32_t currentTimestampMs)
{
    if (
        !m_config.enableStartupBoost)
    {
        return pwmDuty;
    }

    if (pwmDuty == 0)
    {
        return 0;
    }

    bool startupActive = false;

    uint32_t lastStartup = 0;

    if (
        channel ==
        MotorChannel::Left)
    {
        startupActive = m_memory.leftStartupBoostActive;

        lastStartup =
            m_memory.lastLeftStartupTimestampMs;
    }
    else
    {
        startupActive =
            m_memory.rightStartupBoostActive;

        lastStartup =
            m_memory.lastRightStartupTimestampMs;
    }

    //-----------------------------------------
    // Startup boost duration
    //-----------------------------------------

    if (
        startupActive &&
        (currentTimestampMs -
         lastStartup) <
            m_config.startupBoostDurationMs)
    {
        return std::max(
            pwmDuty,
            m_config.startupBoostPWMDuty);
    }

    return pwmDuty;
}

//====================================================
// Direction Transition Validation
//====================================================

bool TB6612Driver::validateDirectionTransition(
    MotorChannel channel,
    MotorDirection newDirection,
    uint32_t currentTimestampMs)
{
    if (
        !m_config.enableSafeDirectionTransition)
    {
        return true;
    }

    MotorDirection currentDirection =
        MotorDirection::Stop;

    uint32_t lastTransition = 0;

    if (
        channel ==
        MotorChannel::Left)
    {
        currentDirection =
            m_memory.leftMotorDirection;

        lastTransition =
            m_memory
                .lastLeftDirectionChangeTimestampMs;
    }
    else
    {
        currentDirection =
            m_memory.rightMotorDirection;

        lastTransition =
            m_memory
                .lastRightDirectionChangeTimestampMs;
    }

    //-----------------------------------------
    // Forward -> reverse protection
    //-----------------------------------------

    const bool reversing =
        ((currentDirection ==
              MotorDirection::Forward &&
          newDirection ==
              MotorDirection::Reverse) ||
         (currentDirection ==
              MotorDirection::Reverse &&
          newDirection ==
              MotorDirection::Forward));

    if (!reversing)
    {
        return true;
    }

    //-----------------------------------------
    // Enforce delay
    //-----------------------------------------

    return (
        (
            currentTimestampMs -
            lastTransition) >=
        m_config.reverseTransitionDelayMs);
}

//====================================================
// Commit PWM Update
//====================================================

void TB6612Driver::commitPWMUpdate(
    MotorChannel channel)
{
    ledc_update_duty(
        m_config.ledcMode,
        getPWMChannel(channel));
}

//====================================================
// Commit Synchronized PWM Update
//====================================================

void TB6612Driver::commitSynchronizedPWMUpdate()
{
    ledc_update_duty(
        m_config.ledcMode,
        m_config.leftPWMChannel);

    ledc_update_duty(
        m_config.ledcMode,
        m_config.rightPWMChannel);
}

//====================================================
// Stop Single Motor
//====================================================

void TB6612Driver::stopMotor(
    MotorChannel channel)
{
    applyPWMDuty(channel, 0);

    applyMotorDirection(
        channel,
        MotorDirection::Stop);
}

//====================================================
// Emergency Brake
//====================================================

void TB6612Driver::applyEmergencyBrake()
{
    applyMotorBrake(
        MotorChannel::Left,
        BrakeMode::Emergency);

    applyMotorBrake(
        MotorChannel::Right,
        BrakeMode::Emergency);
}

//====================================================
// Update Runtime Memory
//====================================================

void TB6612Driver::updateMotorMemory(
    MotorChannel channel,
    MotorDirection direction,
    uint32_t pwmDuty,
    uint32_t currentTimestampMs)
{
    if (
        channel ==
        MotorChannel::Left)
    {
        m_memory.previousLeftPWMDuty =
            m_memory.currentLeftPWMDuty;

        m_memory.currentLeftPWMDuty =
            pwmDuty;

        m_memory.previousLeftMotorDirection =
            m_memory.leftMotorDirection;

        m_memory.leftMotorDirection =
            direction;

        m_memory
            .lastLeftDirectionChangeTimestampMs =
            currentTimestampMs;
    }
    else
    {
        m_memory.previousRightPWMDuty =
            m_memory.currentRightPWMDuty;

        m_memory.currentRightPWMDuty =
            pwmDuty;

        m_memory.previousRightMotorDirection =
            m_memory.rightMotorDirection;

        m_memory.rightMotorDirection =
            direction;

        m_memory
            .lastRightDirectionChangeTimestampMs =
            currentTimestampMs;
    }

    m_memory.lastUpdateTimestampMs =
        currentTimestampMs;
}

//====================================================
// Timestamp Utility
//====================================================

uint32_t
TB6612Driver::getCurrentTimestampMs() const
{
    return static_cast<uint32_t>(
        esp_timer_get_time() / 1000ULL);
}

//====================================================
// Pin Mapping
//====================================================

gpio_num_t TB6612Driver::getIN1Pin(
    MotorChannel channel) const
{
    return (
               channel ==
               MotorChannel::Left)
               ? m_config.leftMotorIN1Pin
               : m_config.rightMotorIN1Pin;
}

//====================================================
// Pin Mapping
//====================================================

gpio_num_t TB6612Driver::getIN2Pin(
    MotorChannel channel) const
{
    return (
               channel ==
               MotorChannel::Left)
               ? m_config.leftMotorIN2Pin
               : m_config.rightMotorIN2Pin;
}

//====================================================
// PWM Channel Mapping
//====================================================

ledc_channel_t
TB6612Driver::getPWMChannel(
    MotorChannel channel) const
{
    return (
               channel ==
               MotorChannel::Left)
               ? m_config.leftPWMChannel
               : m_config.rightPWMChannel;
}

//====================================================
// Motor Inversion
//====================================================
bool TB6612Driver::isMotorInverted(MotorChannel channel) const
{
    return (
               channel ==
               MotorChannel::Left)
               ? m_config.invertLeftMotor
               : m_config.invertRightMotor;
}

bool TB6612Driver::hasCommandTimedOut(uint32_t currentTimestampMs) const
{
    if (
        !m_config.enableCommandTimeoutProtection)
    {
        return false;
    }

    if (
        m_memory.lastCommandTimestampMs == 0)
    {
        return false;
    }

    return (
        (
            currentTimestampMs -
            m_memory.lastCommandTimestampMs) >
        m_config.commandTimeoutMs);
}

void TB6612Driver::handleCommandTimeout()
{
    ESP_LOGE(
        TAG,
        "Motor command timeout");

    emergencyStop();
}

void TB6612Driver::update(uint32_t currentTimestampMs)
{
    ScopedDriverLock lock(this);

    //-----------------------------------------
    // Watchdog timeout
    //-----------------------------------------

    if (
        hasCommandTimedOut(
            currentTimestampMs))
    {
        handleCommandTimeout();
    }
}

void TB6612Driver::reset()
{
    ScopedDriverLock lock(this);

    stopMotor(MotorChannel::Left);

    stopMotor(MotorChannel::Right);

    m_memory = {};

    m_memory.initialized = true;

    transitionToState(
        MotorDriverState::Ready);
}

bool TB6612Driver::validateStateTransition(
    MotorDriverState currentState,
    MotorDriverState newState) const
{
    switch (currentState)
    {
    case MotorDriverState::Fault:

        return false;

    case MotorDriverState::EmergencyStopped:

        return (
            newState ==
            MotorDriverState::Ready);

    default:

        return true;
    }
}

void TB6612Driver::transitionToState(
    MotorDriverState newState)
{
    if (
        !validateStateTransition(
            m_memory.currentState,
            newState))
    {
        ESP_LOGE(
            TAG,
            "Invalid state transition");

        return;
    }

    m_memory.currentState =
        newState;
}

void TB6612Driver::performSafeReverseSequence(
    MotorChannel channel,
    MotorDirection newDirection)
{
    // Brake
    applyMotorBrake(
        channel,
        BrakeMode::Active);

    vTaskDelay(
        pdMS_TO_TICKS(
            m_config
                .safeReverseBrakeDurationMs));

    // Coast
    applyMotorDirection(
        channel,
        MotorDirection::Stop);

    vTaskDelay(
        pdMS_TO_TICKS(
            m_config
                .safeReverseCoastDurationMs));

    // Apply new direction
    applyMotorDirection(
        channel,
        newDirection);
}

uint32_t TB6612Driver::applyPWMRamping(MotorChannel channel, uint32_t targetPWMDuty)
{
    if (!m_config.enablePWMRamping)
    {
        return targetPWMDuty;
    }

    uint32_t currentDuty = 0;

    if (channel == MotorChannel::Left)
    {
        currentDuty =
            m_memory.currentLeftPWMDuty;
    }
    else
    {
        currentDuty =
            m_memory.currentRightPWMDuty;
    }

    // Ramp up
    if (targetPWMDuty > currentDuty)
    {
        return std::min(
            currentDuty +
                m_config.maximumPWMStepPerUpdate,
            targetPWMDuty);
    }

    // Ramp down
    return std::max(
        static_cast<int32_t>(currentDuty) -
            static_cast<int32_t>(
                m_config.maximumPWMStepPerUpdate),
        static_cast<int32_t>(targetPWMDuty));
}