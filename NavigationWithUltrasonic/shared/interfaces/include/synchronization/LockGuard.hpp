#pragma once

#include "interfaces/include/synchronization/IMutex.hpp"

/**
 * @brief RAII mutex lock helper.
 *
 * Locks the mutex on construction and automatically
 * unlocks it when leaving scope.
 *
 * Usage:
 *   LockGuard guard(mutex());
 */
// Prevents forgotten unlocks.
class LockGuard
{
public:
    explicit LockGuard(IMutex &mutex)
        : m_mutex(mutex)
    {
        m_mutex.lock();
    }

    ~LockGuard()
    {
        m_mutex.unlock();
    }

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

private:
    IMutex &m_mutex;
};