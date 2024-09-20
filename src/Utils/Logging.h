#pragma once

#include <stdarg.h>
#include <stdint.h>

class Logging
{
public:
    enum class Level
    {
        Verbose,
        Debug,
        Info,
        Warning,
        Error
    };

    void Buffer(const char *tag, const uint8_t *buf, uint32_t len, Level level);
    
    void Verbose(const char *tag, const char *format, ...);
    void Debug(const char *tag, const char *format, ...);
    void Info(const char *tag, const char *format, ...);
    void Warning(const char *tag, const char *format, ...);
    void Error(const char *tag, const char *format, ...);

    static Logging &get();
};

extern Logging &Log;