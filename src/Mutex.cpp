
#include "Mutex.h"

namespace PowerFeather
{

    void Mutex::init()
    {
        _sem = xSemaphoreCreateRecursiveMutex();
    }

    bool Mutex::lock()
    {
        return xSemaphoreTakeRecursive(_sem, pdMS_TO_TICKS(_timeout));
    }

    void Mutex::unlock()
    {
        xSemaphoreGiveRecursive(_sem);
    }
}

