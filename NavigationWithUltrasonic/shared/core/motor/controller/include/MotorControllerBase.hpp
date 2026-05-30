#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MotorControllerBase public IMotorDriver
{
public:
    virtual ~MotorControllerBase();

protected:
    MotorControllerBase();

    // RTOS lock
    void lockController();

    // RTOS unlock
    void unlockController();

protected:
    // Scoped controller lock
    class ScopedControllerLock
    {
    public:
        explicit ScopedControllerLock(
            MotorControllerBase *controller)
            : m_controller(controller)
        {
            if (m_controller != nullptr)
            {
                m_controller->lockController();
            }
        }

        ~ScopedControllerLock()
        {
            if (m_controller != nullptr)
            {
                m_controller->unlockController();
            }
        }

    private:
        MotorControllerBase *m_controller = nullptr;
    };

protected:
    SemaphoreHandle_t m_controllerMutex = nullptr;
};