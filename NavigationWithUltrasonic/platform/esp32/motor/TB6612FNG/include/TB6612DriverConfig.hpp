//====================================================
// File: TB6612DriverConfig.hpp
//====================================================

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

#include <stdint.h>

//====================================================
// TB6612DriverConfig
// Production-grade configuration for
// TB6612FNG dual motor driver.
// Defines:
//  - GPIO pin mapping
//  - PWM configuration
//  - safety constraints
//  - deadzone compensation
//  - startup boost behavior
//  - reverse transition protection
//  - emergency braking behavior
// This configuration is:
//  - hardware-focused
//  - deterministic
//  - real-time safe

struct TB6612DriverConfig
{
    //================================================
    // LEFT MOTOR PINS
    //================================================
    // Direction pin 1
    gpio_num_t leftMotorIN1Pin = GPIO_NUM_NC;

    // Direction pin 2
    gpio_num_t leftMotorIN2Pin = GPIO_NUM_NC;

    // PWM pin
    gpio_num_t leftMotorPWMPin = GPIO_NUM_NC;

    //================================================
    // RIGHT MOTOR PINS
    //================================================

    // Direction pin 1
    gpio_num_t rightMotorIN1Pin = GPIO_NUM_NC;

    // Direction pin 2
    gpio_num_t rightMotorIN2Pin = GPIO_NUM_NC;

    // PWM pin
    gpio_num_t rightMotorPWMPin = GPIO_NUM_NC;

    //================================================
    // SHARED CONTROL PINS
    //================================================

    // Standby pin: HIGH  -> enabled, LOW   -> standby
    gpio_num_t standbyPin = GPIO_NUM_NC;

    //================================================
    // PWM CONFIGURATION
    //================================================

    //-----------------------------------------
    // PWM frequency, Recommended: 20kHz
    // Benefits: - quieter motors,  - smoother control, - less audible noise
    uint32_t pwmFrequencyHz = 20000;

    // PWM resolution
    ledc_timer_bit_t pwmResolution = LEDC_TIMER_10_BIT;

    // PWM timer
    ledc_timer_t pwmTimer = LEDC_TIMER_0;

    // Left motor PWM channel
    ledc_channel_t leftPWMChannel = LEDC_CHANNEL_0;

    // Right motor PWM channel
    ledc_channel_t rightPWMChannel = LEDC_CHANNEL_1;

    // LEDC speed mode
    ledc_mode_t ledcMode = LEDC_LOW_SPEED_MODE;

    //================================================
    // PWM LIMITS
    //================================================

    // Maximum PWM duty: Auto-calculated typically: 1023 for 10-bit
    uint32_t maximumPWMDuty = 1023;

    // Minimum effective PWM
    // Real motors often do not move
    // below a certain PWM threshold.
    uint32_t minimumEffectivePWMDuty = 180;

    // Startup boost PWM
    // Used briefly to overcome:
    //      - static friction
    //      - gearbox resistance
    uint32_t startupBoostPWMDuty = 280;

    // Startup boost duration
    uint32_t startupBoostDurationMs = 80;

    //================================================
    // SAFETY CONFIGURATION
    //================================================

    // Reverse transition protection
    // Prevents:
    //      forward -> reverse instantly
    bool enableSafeDirectionTransition = true;

    // Transition delay
    uint32_t reverseTransitionDelayMs = 50;

    // Enable emergency brake
    bool enableEmergencyBrake = true;

    // Emergency brake duration
    uint32_t emergencyBrakeDurationMs = 200;

    // Enable active braking
    bool enableActiveBraking = true;

    // Brake before reverse
    bool brakeBeforeDirectionChange = true;

    //================================================
    // MOTOR CHARACTERISTICS
    //================================================

    // Left motor inversion, Useful if motor wiring orientation differs.
    bool invertLeftMotor = false;

    // Right motor inversion
    bool invertRightMotor = false;

    // Enable deadzone compensation
    bool enableDeadzoneCompensation = true;

    // Enable startup boost
    bool enableStartupBoost = true;

    //================================================
    // EXECUTION SAFETY
    //================================================

    // Validate PWM range
    bool validatePWMDuty = true;

    // Clamp PWM automatically
    bool enablePWMClamping = true;

    // Reject invalid commands
    bool rejectInvalidCommands = true;

    // Enable fault detection
    bool enableFaultDetection = true;

    // Command watchdog
    bool enableCommandTimeoutProtection = true;
    uint32_t commandTimeoutMs = 500;

    // PWM ramping
    bool enablePWMRamping = true;
    uint32_t maximumPWMStepPerUpdate = 40;

    // RTOS thread safety
    bool enableThreadSafety = true;

    // Safe reverse sequence
    bool enableSafeReverseSequence = true;
    uint32_t safeReverseBrakeDurationMs = 50;
    uint32_t safeReverseCoastDurationMs = 30;

    //================================================
    // REAL-TIME EXECUTION
    //================================================

    // Enable synchronized updates. Important for: differential drive robots
    bool enableSynchronizedDualMotorUpdate = true;

    // Enable deterministic updates
    bool enableDeterministicExecution = true;
};