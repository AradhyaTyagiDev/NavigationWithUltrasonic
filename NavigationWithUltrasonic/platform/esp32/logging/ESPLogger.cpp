#include "ESPLogger.hpp"

#include "esp_log.h"

namespace
{
    // Convert abstraction -> ESP-IDF
    esp_log_level_t toESPLogLevel(
        LogLevel level)
    {
        switch (level)
        {
        case LogLevel::Trace:
            return ESP_LOG_VERBOSE;

        case LogLevel::Debug:
            return ESP_LOG_DEBUG;

        case LogLevel::Info:
            return ESP_LOG_INFO;

        case LogLevel::Warning:
            return ESP_LOG_WARN;

        case LogLevel::Error:
            return ESP_LOG_ERROR;

        case LogLevel::Critical:
            return ESP_LOG_ERROR;

        default:
            return ESP_LOG_INFO;
        }
    }
}

ESPLogger::ESPLogger(
    LogLevel minimumLogLevel)
    : m_minimumLogLevel(minimumLogLevel)
{
}

bool ESPLogger::shouldLog(
    LogLevel level) const
{
    return static_cast<int>(level) >=
           static_cast<int>(m_minimumLogLevel);
}

void ESPLogger::setMinimumLogLevel(
    LogLevel level)
{
    m_minimumLogLevel = level;
}

LogLevel ESPLogger::getMinimumLogLevel() const
{
    return m_minimumLogLevel;
}

void ESPLogger::log(
    LogLevel level,
    const char *component,
    const char *message)
{
    if (!shouldLog(level))
    {
        return;
    }

    esp_log_write(
        toESPLogLevel(level),
        component,
        "%s\n",
        message);
}

void ESPLogger::logV(
    LogLevel level,
    const char *component,
    const char *format,
    va_list args)
{
    if (!shouldLog(level))
    {
        return;
    }

    esp_log_writev(
        toESPLogLevel(level),
        component,
        format,
        args);
}