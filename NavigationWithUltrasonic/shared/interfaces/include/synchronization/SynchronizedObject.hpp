#pragma once

#include "interfaces/include/synchronization/IMutex.hpp"

/**
 * @brief Base class for thread-safe objects.
 * Stores a platform-independent mutex and provides
 * protected access to derived classes.
 * Usage:
 *   class MotorController
 *       : protected SynchronizedObject
 *   {
 *       ...
 *   };
 */
class SynchronizedObject
{
public:
    explicit SynchronizedObject(
        IMutex &mutex)
        : m_mutex(mutex)
    {
    }

    virtual ~SynchronizedObject() = default;

    SynchronizedObject(
        const SynchronizedObject &) = delete;

    SynchronizedObject &operator=(
        const SynchronizedObject &) = delete;

    SynchronizedObject(
        SynchronizedObject &&) = delete;

    SynchronizedObject &operator=(
        SynchronizedObject &&) = delete;

protected:
    [[nodiscard]]
    IMutex &mutex() noexcept
    {
        return m_mutex;
    }

private:
    IMutex &m_mutex;
};