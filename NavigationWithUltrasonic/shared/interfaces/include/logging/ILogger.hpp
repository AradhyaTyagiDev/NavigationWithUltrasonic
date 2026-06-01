#pragma once

#include <string_view>
#include <cstdarg>

enum class LogLevel
{
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical
};

class ILogger
{
public:
    virtual ~ILogger() = default;

    // Generic logging entry point
    virtual void log(
        LogLevel level,
        const char *component,
        const char *message) = 0;

    virtual void logV(
        LogLevel level,
        const char *component,
        const char *format,
        va_list args) = 0;

    // Runtime filtering
    virtual bool shouldLog(
        LogLevel level) const = 0;
};