#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace PowerFeather
{
    class Mutex
    {
    public:
        class Lock
        {
        public:
            Lock(Mutex& mutex) : _mutex(mutex) { _locked = _mutex.lock(); }
            ~Lock() { _mutex.unlock(); }

            bool isLocked() { return _locked; }
        private:
            Mutex& _mutex;
            bool _locked;
        };

        Mutex(uint32_t timeout) : _timeout(timeout) {}

        void init();
        bool lock();
        void unlock();
    private:
        SemaphoreHandle_t _sem;
        uint32_t _timeout;
    };
}
