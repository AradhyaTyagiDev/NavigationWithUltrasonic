//====================================================
// File: MotorDriverTypes.hpp
//====================================================

#pragma once

#include <stdint.h>

//====================================================
// MotorDirection
//====================================================

enum class MotorDirection
{
    Stop,
    Forward,
    Reverse,
    Brake
};

//====================================================
// BrakeMode
//====================================================

enum class BrakeMode
{
    Coast,
    Active,
    Emergency
};

//====================================================
// MotorDriverState
//====================================================

enum class MotorDriverState
{
    Uninitialized,
    Ready,
    Running,
    Braking,
    EmergencyStopped,
    Fault
};

//====================================================
// MotorChannel
//====================================================

enum class MotorChannel
{
    Left,
    Right
};

//====================================================
// MotorHardwareType
//====================================================

enum class MotorHardwareType
{
    Unknown,
    BTS7960,
    TB6612FNG,
    CytronMD,
    L298N,
    VESC,
    Custom
};