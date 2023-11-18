#include <freertos/FreeRTOS.h>

namespace PowerFeather
{
    class Mutex
    {
    public:
        class Lock
        {
        public:
            Lock(Mutex& mutex) : _mutex(mutex) { _mutex.lock(); }
            ~Lock() { _mutex.unlock(); }
        private:
            Mutex& _mutex;
        };

        void init() {}
        void lock() {}
        void unlock() {}
    };
}
