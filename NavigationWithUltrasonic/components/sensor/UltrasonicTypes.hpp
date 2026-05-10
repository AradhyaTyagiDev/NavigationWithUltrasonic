#pragma once

#include <stdint.h>

struct UltrasonicSensorData
{
    uint32_t pulseWidthUs = 0;

    uint32_t timestampMs = 0;
};