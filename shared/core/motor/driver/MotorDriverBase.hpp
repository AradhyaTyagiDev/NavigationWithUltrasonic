#pragma once

#include "IMotorDriver.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MotorDriverBase : public IMotorDriver
{
public:
    virtual ~MotorDriverBase();

protected:
    MotorDriverBase();

    //------------------------------------------------
    // Driver lock
    //------------------------------------------------
    void lockDriver();

    //------------------------------------------------
    // Driver unlock
    //------------------------------------------------
    void unlockDriver();

protected:
    //------------------------------------------------
    // Scoped RTOS lock guard
    //------------------------------------------------

    class ScopedDriverLock
    {
    public:
        explicit ScopedDriverLock(
            MotorDriverBase *driver)
            : m_driver(driver)
        {
            if (m_driver != nullptr)
            {
                m_driver->lockDriver();
            }
        }

        ~ScopedDriverLock()
        {
            if (m_driver != nullptr)
            {
                m_driver->unlockDriver();
            }
        }

    private:
        MotorDriverBase *m_driver =
            nullptr;
    };

protected:
    SemaphoreHandle_t m_driverMutex =
        nullptr;
};