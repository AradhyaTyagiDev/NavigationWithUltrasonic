#pragma once

#include "interfaces/include/synchronization/IMutex.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class FreeRTOSMutex final : public IMutex
{
public:
    FreeRTOSMutex();

    ~FreeRTOSMutex() override;

    FreeRTOSMutex(const FreeRTOSMutex &) = delete;
    FreeRTOSMutex &operator=(const FreeRTOSMutex &) = delete;

    FreeRTOSMutex(FreeRTOSMutex &&) = delete;
    FreeRTOSMutex &operator=(FreeRTOSMutex &&) = delete;

    /*
     * Blocks indefinitely until mutex acquired.
     *
     * NOT ISR-safe.
     */
    void lock() override;

    /*
     * Attempts immediate mutex acquisition.
     *
     * NOT ISR-safe.
     */
    bool tryLock() override;

    /*
     * Attempts mutex acquisition with timeout.
     *
     * NOT ISR-safe.
     */
    bool lockFor(uint32_t timeoutMs) override;

    /*
     * Releases mutex.
     *
     * NOT ISR-safe.
     */
    void unlock() override;

private:
    SemaphoreHandle_t m_mutexHandle{nullptr};

private:
    static bool isInISR();
};