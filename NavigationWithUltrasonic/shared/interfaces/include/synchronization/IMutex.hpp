#pragma once

/*
 * NOTE: NOT ISR safe unless explicitly documented
 * FreeRTOS mutexes are task-only. Blocking inside ISR is catastrophic
 */

#include <cstdint>

class IMutex
{
public:
    virtual ~IMutex() = default;

    /*
     * Blocks until mutex acquired.
     * Must be deterministic and thread-safe.
     */
    virtual void lock() = 0;

    /*
     * Attempts immediate lock without blocking.
     *
     * Returns:
     * true  -> lock acquired
     * false -> mutex already locked
     */
    virtual bool tryLock() = 0;

    /*
     * Attempts lock with timeout.
     *
     * timeoutMs:
     *   0     -> immediate return
     *   UINT32_MAX -> infinite wait
     *
     * Returns:
     * true  -> lock acquired
     * false -> timeout occurred
     */
    virtual bool lockFor(uint32_t timeoutMs) = 0;

    /*
     * Releases mutex.
     */
    virtual void unlock() = 0;
};