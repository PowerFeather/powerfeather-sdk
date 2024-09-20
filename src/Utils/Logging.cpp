#include "Logging.h"

Logging &Log = Logging::get();

void Logging::Buffer(const char *tag, const uint8_t *buf, uint32_t len, Level level)
{
    // TODO
}

void Logging::Verbose(const char *tag, const char *format, ...)
{
    // TODO
}

void Logging::Debug(const char *tag, const char *format, ...)
{
    // TODO
}

void Logging::Info(const char *tag, const char *format, ...)
{
    // TODO
}

void Logging::Warning(const char *tag, const char *format, ...)
{
    // TODO
}

void Logging::Error(const char *tag, const char *format, ...)
{
    // TODO
}

/*static*/ Logging &Logging::get()
{
    static Logging log;
    return log;
}