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

#include <math.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "MAX17260.h"

namespace PowerFeather
{
    static const char *TAG = "PowerFeather::Mainboard::MAX17260";

    bool MAX17260::_readReg(Register reg, uint16_t &value)
    {
        assert(reg.start <= reg.end);
        assert(reg.end <= (_regSize * CHAR_BIT) - 1);

        uint16_t data = 0;
        if (_i2c.read(_i2cAddress, reg.address, reinterpret_cast<uint8_t*>(&data), _regSize))
        {
            int left = (((sizeof(data) * CHAR_BIT) - 1) - reg.end);
            data <<= left;
            data >>= left + reg.start;
            value = data;
            ESP_LOGD(TAG, "Read bit%d to bit%d on register %02x, value = %04x.", reg.start, reg.end, reg.address, value);
            return true;
        }

        ESP_LOGD(TAG, "Read bit%d to bit%d on register %02x failed.", reg.start, reg.end, reg.address);
        return false;
    }

    bool MAX17260::_writeReg(Register reg, uint16_t value)
    {
        uint8_t last = (_regSize * CHAR_BIT) - 1;

        assert(reg.start <= reg.end);
        assert(reg.end <= last);

        uint16_t data = 0;

        if (_readReg(Register{reg.address, 0, last}, data))
        {
            uint8_t bits = reg.end - reg.start + 1;
            uint16_t mask = ((1 << bits) - 1) << reg.start;
            data = (data & ~mask) | ((value << reg.start) & mask);
            bool res = _i2c.write(_i2cAddress, reg.address, reinterpret_cast<uint8_t *>(&data), _regSize);
            if (res)
            {
                ESP_LOGD(TAG, "Write bit%d to bit%d on register %02x, value = %04x.", reg.start, reg.end, reg.address, data);
            }
            else
            {
                ESP_LOGD(TAG, "Write bit%d to bit%d on register %02x failed.", reg.start, reg.end, reg.address);
            }
            return res;
        }

        return false;
    }

    bool MAX17260::init()
    {
        uint16_t stat = 0;

        if (_readReg(Status_POR, stat)) // POR occured if true
        {
            uint16_t fstat = 0;
            while (_readReg(FStat_DNR, fstat)) vTaskDelay(pdMS_TO_TICKS(10));

        }
        else
        {
        }

        return false;
    }

    bool MAX17260::getEnabled(bool &enabled)
    {
        return false;
    }

    bool MAX17260::getCellVoltage(uint16_t &voltage)
    {
        return false;
    }

    bool MAX17260::getRSOC(uint8_t &percent)
    {
        return false;
    }

    bool MAX17260::getTimeToEmpty(uint16_t &minutes)
    {
        return false;
    }

    bool MAX17260::getTimeToFull(uint16_t &minutes)
    {
        return false;
    }

    bool MAX17260::getCellTemperature(float &temperature)
    {
        return false;
    }

    bool MAX17260::getCycles(uint16_t &cycles)
    {
        return false;
    }

    bool MAX17260::getSOH(uint8_t &percent)
    {
        return false;
    }

    bool MAX17260::getInitialized(bool& state)
    {
        return false;
    }

    bool MAX17260::setEnabled(bool enable)
    {
        return false;
    }

    bool MAX17260::setCellTemperature(float temperature)
    {
        return false;
    }

    bool MAX17260::enableTSENSE(bool enableTsense1, bool enableTsense2)
    {
        return false;
    }

    bool MAX17260::setLowVoltageAlarm(uint16_t voltage)
    {
        return false;
    }

    bool MAX17260::setHighVoltageAlarm(uint16_t voltage)
    {
        return false;
    }

    bool MAX17260::setLowRSOCAlarm(uint8_t percent)
    {
        return false;
    }

    bool MAX17260::setTerminationFactor(float factor)
    {
        return false;
    }

    bool MAX17260::setInitialized()
    {
        return false;
    }

    bool MAX17260::clearLowVoltageAlarm()
    {
        return false;
    }

    bool MAX17260::clearHighVoltageAlarm()
    {
        return false;
    }

    bool MAX17260::clearLowRSOCAlarm()
    {
        return false;
    }
}