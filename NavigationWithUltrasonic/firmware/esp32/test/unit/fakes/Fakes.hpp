#pragma once

#include "interfaces/include/logging/ILogger.hpp"
#include "interfaces/include/motor/driver/IMotorDriver.hpp"
#include "interfaces/include/sensor/IUltrasonicSensor.hpp"
#include "interfaces/include/synchronization/IMutex.hpp"
#include "interfaces/include/timing/ITimer.hpp"

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

class FakeMutex final : public IMutex
{
public:
    void lock() override
    {
        m_locked = true;
    }

    bool tryLock() override
    {
        if (m_forceBusy || m_locked)
        {
            return false;
        }

        m_locked = true;

        return true;
    }

    bool lockFor(uint32_t timeoutMs) override
    {
        (void)timeoutMs;

        return tryLock();
    }

    void unlock() override
    {
        m_locked = false;
    }

    void setForceBusy(bool forceBusy)
    {
        m_forceBusy = forceBusy;
    }

private:
    bool m_locked = false;
    bool m_forceBusy = false;
};

class FakeLogger final : public ILogger
{
public:
    void log(
        LogLevel level,
        const char *component,
        const char *message) override
    {
        if (!shouldLog(level))
        {
            return;
        }

        entries.push_back(
            std::string(component) + ": " + message);
    }

    void logV(
        LogLevel level,
        const char *component,
        const char *format,
        va_list args) override
    {
        if (!shouldLog(level))
        {
            return;
        }

        char buffer[256];

        vsnprintf(
            buffer,
            sizeof(buffer),
            format,
            args);

        log(
            level,
            component,
            buffer);
    }

    bool shouldLog(
        LogLevel level) const override
    {
        return static_cast<int>(level) >=
               static_cast<int>(minimumLevel);
    }

    LogLevel minimumLevel = LogLevel::Trace;
    std::vector<std::string> entries;
};

class FakeTimer final : public ITimer
{
public:
    uint64_t getTimestampUs() const override
    {
        return timestampUs;
    }

    void advanceMs(uint32_t deltaMs)
    {
        timestampUs +=
            static_cast<uint64_t>(deltaMs) *
            1000ULL;
    }

    void advanceUs(uint32_t deltaUs)
    {
        timestampUs += deltaUs;
    }

    void setMs(uint32_t timestampMs)
    {
        timestampUs =
            static_cast<uint64_t>(timestampMs) *
            1000ULL;
    }

private:
    uint64_t timestampUs = 0;
};

class FakeUltrasonicSensor final : public IUltrasonicSensor
{
public:
    bool initialize() override
    {
        initialized = true;

        return initializeResult;
    }

    bool start() override
    {
        running = startResult;

        return startResult;
    }

    void shutdown() override
    {
        running = false;
        initialized = false;
    }

    bool fetchLatestData(
        UltrasonicSensorData &outData) override
    {
        if (!hasData)
        {
            return false;
        }

        outData = latestData;
        hasData = false;
        fetchCount++;

        return true;
    }

    bool isHealthy() const override
    {
        return healthy;
    }

    bool isRunning() const override
    {
        return running;
    }

    void publishPulse(
        uint32_t pulseWidthUs,
        uint32_t timestampMs)
    {
        latestData.pulseWidthUs = pulseWidthUs;
        latestData.timestampMs = timestampMs;
        hasData = true;
    }

    bool initializeResult = true;
    bool startResult = true;
    bool initialized = false;
    bool running = false;
    bool healthy = true;
    bool hasData = false;
    uint32_t fetchCount = 0;
    UltrasonicSensorData latestData;
};

class FakeMotorDriver final : public IMotorDriver
{
public:
    bool initialize() override
    {
        initialized = initializeResult;
        ready = initializeResult;

        return initializeResult;
    }

    void shutdown() override
    {
        ready = false;
    }

    void executeCommand(
        const MotorDriverCommand &command) override
    {
        lastCommand = command;
        commandCount++;
    }

    void executeDualCommand(
        const MotorDriverCommand &leftCommand,
        const MotorDriverCommand &rightCommand) override
    {
        if (timerToAdvance != nullptr)
        {
            timerToAdvance->advanceUs(
                advanceOnDualCommandUs);
        }

        lastLeftCommand = leftCommand;
        lastRightCommand = rightCommand;
        dualCommandCount++;
    }

    void stopAllMotors() override
    {
        stopped = true;
    }

    void emergencyStop() override
    {
        emergencyStopActive = true;
    }

    void clearEmergencyStop() override
    {
        emergencyStopActive = false;
    }

    void applyBrakeMode(
        BrakeMode brakeMode) override
    {
        lastBrakeMode = brakeMode;
    }

    void setMotorDirection(
        MotorChannel channel,
        MotorDirection direction) override
    {
        (void)channel;

        lastDirection = direction;
    }

    void setPWMDuty(
        MotorChannel channel,
        uint32_t pwmDuty) override
    {
        (void)channel;

        lastPwmDuty = pwmDuty;
    }

    void enableMotor(
        MotorChannel channel) override
    {
        (void)channel;

        enabled = true;
    }

    void disableMotor(
        MotorChannel channel) override
    {
        (void)channel;

        enabled = false;
    }

    bool isReady() const override
    {
        return ready;
    }

    bool hasFault() const override
    {
        return fault;
    }

    MotorDriverCapabilities getCapabilities() const override
    {
        return capabilities;
    }

    MotorDriverStatus getStatus() const override
    {
        MotorDriverStatus status;
        status.state = state;
        status.faultDetected = fault;
        status.emergencyStopActive = emergencyStopActive;

        return status;
    }

    MotorHardwareType getHardwareType() const override
    {
        return MotorHardwareType::Custom;
    }

    void update(
        uint32_t currentTimestampMs) override
    {
        lastUpdateTimestampMs = currentTimestampMs;
        updateCount++;
    }

    void reset() override
    {
        fault = false;
        emergencyStopActive = false;
        stopped = false;
    }

    MotorDriverState getDriverState() const override
    {
        return state;
    }

    bool initializeResult = true;
    bool initialized = false;
    bool ready = true;
    bool fault = false;
    bool stopped = false;
    bool enabled = false;
    bool emergencyStopActive = false;
    uint32_t commandCount = 0;
    uint32_t dualCommandCount = 0;
    uint32_t updateCount = 0;
    uint32_t lastUpdateTimestampMs = 0;
    uint32_t lastPwmDuty = 0;
    MotorDirection lastDirection = MotorDirection::Stop;
    BrakeMode lastBrakeMode = BrakeMode::Coast;
    MotorDriverState state = MotorDriverState::Ready;
    MotorDriverCapabilities capabilities;
    MotorDriverCommand lastCommand;
    MotorDriverCommand lastLeftCommand;
    MotorDriverCommand lastRightCommand;
    FakeTimer *timerToAdvance = nullptr;
    uint32_t advanceOnDualCommandUs = 0;
};
