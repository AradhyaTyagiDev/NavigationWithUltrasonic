#pragma once

#include "interfaces/include/timing/ITimer.hpp"
#include "esp_timer.h"

class ESPTimer final : public ITimer
{
public:
    uint64_t getTimestampUs() const override
    {
        return static_cast<uint64_t>(
            esp_timer_get_time());
    }
};
