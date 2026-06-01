#pragma once

#include "interfaces/include/logging/ILogger.hpp"

#include <cstdarg>

class ESPLogger final : public ILogger
{
public:
    explicit ESPLogger(
        LogLevel minimumLogLevel =
            LogLevel::Info);

    ~ESPLogger() override = default;

    ESPLogger(
        const ESPLogger &) = delete;

    ESPLogger &operator=(
        const ESPLogger &) = delete;

    ESPLogger(
        ESPLogger &&) = delete;

    ESPLogger &operator=(
        ESPLogger &&) = delete;

public:
    // Plain text logging
    void log(
        LogLevel level,
        const char *component,
        const char *message) override;

    // Formatted logging
    void logV(
        LogLevel level,
        const char *component,
        const char *format,
        va_list args) override;

    // Runtime filtering
    bool shouldLog(
        LogLevel level) const override;

    // Runtime configuration
    void setMinimumLogLevel(
        LogLevel level);

    LogLevel getMinimumLogLevel() const;

private:
    LogLevel m_minimumLogLevel;
};