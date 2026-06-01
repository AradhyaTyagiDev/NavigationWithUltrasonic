#pragma once

class IRobotRuntime
{
public:
    virtual ~IRobotRuntime() = default;

    virtual bool initialize() = 0;

    virtual bool start() = 0;

    virtual void stop() = 0;

    virtual void shutdown() = 0;

    virtual bool isRunning() const = 0;
};
