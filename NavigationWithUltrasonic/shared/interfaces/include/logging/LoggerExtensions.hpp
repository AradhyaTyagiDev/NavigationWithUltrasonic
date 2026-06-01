#pragma once

#include "interfaces/include/logging/ILogger.hpp"
#include "interfaces/include/logging/LoggerConfig.hpp"

#include <cstdarg>

namespace Logger
{
    // Internal helper
    constexpr bool isCompileTimeEnabled(
        LogLevel level)
    {
        return static_cast<int>(level) >=
               static_cast<int>(LoggerConfig::MinimumLogLevel);
    }

    //------------------------------------------------
    // Internal helper
    //------------------------------------------------
    inline void writeV(
        ILogger &logger,
        LogLevel level,
        const char *component,
        const char *format,
        va_list args)
    {
        if (!logger.shouldLog(level))
        {
            return;
        }

        logger.logV(
            level,
            component,
            format,
            args);
    }

    //------------------------------------------------
    // Internal helper
    //------------------------------------------------

    inline void write(
        ILogger &logger,
        LogLevel level,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(level))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            level,
            component,
            format,
            args);

        va_end(args);
    }

    //------------------------------------------------
    // Trace
    //------------------------------------------------

    inline void trace(
        ILogger &logger,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(LogLevel::Trace))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            LogLevel::Trace,
            component,
            format,
            args);

        va_end(args);
    }

    //------------------------------------------------
    // Debug
    //------------------------------------------------

    inline void debug(
        ILogger &logger,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(LogLevel::Debug))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            LogLevel::Debug,
            component,
            format,
            args);

        va_end(args);
    }

    //------------------------------------------------
    // Info
    //------------------------------------------------

    inline void info(
        ILogger &logger,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(LogLevel::Info))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            LogLevel::Info,
            component,
            format,
            args);

        va_end(args);
    }

    //------------------------------------------------
    // Warning
    //------------------------------------------------

    inline void warning(
        ILogger &logger,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(LogLevel::Warning))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            LogLevel::Warning,
            component,
            format,
            args);

        va_end(args);
    }

    //------------------------------------------------
    // Error
    //------------------------------------------------

    inline void error(
        ILogger &logger,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(LogLevel::Error))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            LogLevel::Error,
            component,
            format,
            args);

        va_end(args);
    }

    //------------------------------------------------
    // Critical
    //------------------------------------------------

    inline void critical(
        ILogger &logger,
        const char *component,
        const char *format,
        ...)
    {
        if (!logger.shouldLog(LogLevel::Critical))
        {
            return;
        }

        va_list args;

        va_start(
            args,
            format);

        logger.logV(
            LogLevel::Critical,
            component,
            format,
            args);

        va_end(args);
    }
}