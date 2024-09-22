/**
 *  POWERFEATHER 4-CLAUSE LICENSE
 *
 *  Copyright (C) 2023, PowerFeather.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  3. Neither the name of PowerFeather nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 *  4. This software, with or without modification, must only be run on official
 *      PowerFeather boards.
 *
 *  THIS SOFTWARE IS PROVIDED BY POWERFEATHER “AS IS” AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL POWERFEATHER OR CONTRIBUTORS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ArduinoMasterI2C.h"

#ifdef ARDUINO

#include "Logging.h"

namespace PowerFeather
{
    static const char *TAG = "PowerFeather::Utils::ArduinoMasterI2C";

    bool ArduinoMasterI2C::start()
    {
#if ESP_PLATFORM
        _wire = (_port == 0) ? &Wire : &Wire1;
        Log.Debug(TAG, "Start Wire%d with sda: %d, scl: %d and freq: %d.", _port, _sdaPin, _sclPin, _freq);
        return _wire->begin(_sdaPin, _sclPin, _freq);
#endif
        return false;
    }

    bool ArduinoMasterI2C::write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len)
    {
#if ESP_PLATFORM
        _wire->beginTransmission(address);
        if (_wire->write(reg))
        {
            Log.Verbose(TAG, "Write address: %02x", address);
            if (_wire->write(buf, len) != len)
            {
                Log.Debug(TAG, "Write buf %p of len %d failed.", buf, len);
                return false;
            }
            Log.Verbose(TAG, "Write buf %p of len %d succeeded.", buf, len);
            Log.Buffer(TAG, buf, len, Logging::Level::Verbose);
        }
        return (_wire->endTransmission(true) == 0);
#endif
        return false;
    }

    bool ArduinoMasterI2C::_read(uint8_t address, uint8_t *buf, size_t len)
    {
#if ESP_PLATFORM
        size_t recv = _wire->requestFrom(address, static_cast<uint8_t>(len));

        if (recv != len)
        {
            Log.Debug(TAG, "Read address %02x with len %d failed", address, len);
            return false;
        }

        for (uint16_t i = 0; i < len; i++)
        {
            buf[i] = _wire->read();
        }

        Log.Verbose(TAG, "Read buf %p of len %d succeeded.", buf, len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);
        return true;
#endif
        return false;
    }

    bool ArduinoMasterI2C::read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len)
    {
#if ESP_PLATFORM
        _wire->beginTransmission(address);
        if (_wire->write(reg))
        {
            if (_wire->endTransmission(false) == 0)
            {
                return _read(address, buf, len);
            }
        }

        return false;
#endif
        return false;
    }

    bool ArduinoMasterI2C::end()
    {
#if ESP_PLATFORM
        _wire->end();
        Log.Debug(TAG, "End");
        return true;
#endif
        return false;
    }
}

#endif