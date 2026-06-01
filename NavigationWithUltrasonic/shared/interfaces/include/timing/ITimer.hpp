#pragma once

#include <cstdint>

class ITimer
{
public:
    virtual ~ITimer() = default;

    virtual uint64_t getTimestampUs() const = 0;
};

namespace Timer
{
    inline uint32_t milliseconds(
        const ITimer &timer)
    {
        return static_cast<uint32_t>(
            timer.getTimestampUs() / 1000ULL);
    }

    inline uint32_t seconds(
        const ITimer &timer)
    {
        return static_cast<uint32_t>(
            timer.getTimestampUs() / 1000000ULL);
    }
}

class MockTimer final : public ITimer
{
public:
    uint64_t getTimestampUs() const override
    {
        return m_timestampUs;
    }

    void setTimestampUs(
        uint64_t timestampUs)
    {
        m_timestampUs = timestampUs;
    }

    void advanceUs(
        uint64_t deltaUs)
    {
        m_timestampUs += deltaUs;
    }

    void advanceMs(
        uint32_t deltaMs)
    {
        advanceUs(
            static_cast<uint64_t>(deltaMs) *
            1000ULL);
    }

    void reset()
    {
        m_timestampUs = 0;
    }

private:
    uint64_t m_timestampUs = 0;
};