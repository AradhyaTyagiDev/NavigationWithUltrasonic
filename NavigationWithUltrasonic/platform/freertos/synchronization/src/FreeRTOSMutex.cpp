#include "synchronization/include/FreeRTOSMutex.hpp"

#include <cassert>
#include <climits>

#if defined(__XTENSA__)
#include "freertos/portmacro.h"
#endif

FreeRTOSMutex::FreeRTOSMutex()
{
    m_mutexHandle = xSemaphoreCreateMutex();

    assert(m_mutexHandle != nullptr);
}

FreeRTOSMutex::~FreeRTOSMutex()
{
    /*
     * IMPORTANT:
     * Mutex must not be deleted while owned by another task.
     *
     * Production systems should ensure lifecycle ownership externally.
     */

    if (m_mutexHandle != nullptr)
    {
        vSemaphoreDelete(m_mutexHandle);
        m_mutexHandle = nullptr;
    }
}

void FreeRTOSMutex::lock()
{
    assert(!isInISR());
    assert(m_mutexHandle != nullptr);

    const BaseType_t result =
        xSemaphoreTake(m_mutexHandle, portMAX_DELAY);

    assert(result == pdTRUE);
}

bool FreeRTOSMutex::tryLock()
{
    assert(!isInISR());
    assert(m_mutexHandle != nullptr);

    return xSemaphoreTake(m_mutexHandle, 0) == pdTRUE;
}

bool FreeRTOSMutex::lockFor(uint32_t timeoutMs)
{
    assert(!isInISR());
    assert(m_mutexHandle != nullptr);

    TickType_t timeoutTicks;

    if (timeoutMs == UINT32_MAX)
    {
        timeoutTicks = portMAX_DELAY;
    }
    else
    {
        timeoutTicks = pdMS_TO_TICKS(timeoutMs);
    }

    return xSemaphoreTake(m_mutexHandle, timeoutTicks) == pdTRUE;
}

void FreeRTOSMutex::unlock()
{
    assert(!isInISR());
    assert(m_mutexHandle != nullptr);

    const BaseType_t result =
        xSemaphoreGive(m_mutexHandle);

    assert(result == pdTRUE);
}

bool FreeRTOSMutex::isInISR()
{
#if defined(__XTENSA__)

    /*
     * ESP32 Xtensa ISR detection
     */
    return xPortInIsrContext();

#else

    return false;

#endif
}