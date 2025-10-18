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

#include <algorithm>
#include <cmath>

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
        uint16_t por = 0;
        if (!_readReg(Status_POR, por))
        {
            return false;
        }

        if (por != 0)
        {
            constexpr uint16_t maxAttempts = 100;
            uint16_t attempts = 0;
            uint16_t dnr = 0;

            do
            {
                if (!_readReg(FStat_DNR, dnr))
                {
                    return false;
                }

                if ((dnr & 0x1) == 0)
                {
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(_fStatDNRWaitTime));
            } while (++attempts < maxAttempts);

            if ((dnr & 0x1) != 0)
            {
                ESP_LOGW(TAG, "FSTAT.DNR remained set after initialization wait.");
                return false;
            }
        }

        return true;
    }

    bool MAX17260::getEnabled(bool &enabled)
    {
        uint16_t value = 0;
        if (_readReg(Config_SHDN, value))
        {
            enabled = (value == 0);
            return true;
        }
        return false;
    }

    bool MAX17260::getCellVoltage(uint16_t &voltage)
    {
        uint16_t raw = 0;
        if (_readReg(VCell_Reg, raw))
        {
            uint32_t mv = static_cast<uint32_t>(raw) * 5u;
            voltage = static_cast<uint16_t>((mv + 32u) >> 6); // divide by 64 with rounding
            return true;
        }
        return false;
    }

    bool MAX17260::getRSOC(uint8_t &percent)
    {
        uint16_t raw = 0;
        if (_readReg(RepSOC_Reg, raw))
        {
            percent = static_cast<uint8_t>(std::min<uint16_t>(100, (raw + 0x80) >> 8));
            return true;
        }
        return false;
    }

    bool MAX17260::getTimeToEmpty(uint16_t &minutes)
    {
        uint16_t raw = 0;
        if (_readReg(TTE_Reg, raw))
        {
            uint32_t value = (static_cast<uint32_t>(raw) * 3u + 16u) >> 5;
            minutes = static_cast<uint16_t>(std::min<uint32_t>(value, 0xFFFFu));
            return true;
        }
        return false;
    }

    bool MAX17260::getTimeToFull(uint16_t &minutes)
    {
        uint16_t raw = 0;
        if (_readReg(TTF_Reg, raw))
        {
            uint32_t value = (static_cast<uint32_t>(raw) * 3u + 16u) >> 5;
            minutes = static_cast<uint16_t>(std::min<uint32_t>(value, 0xFFFFu));
            return true;
        }
        return false;
    }

    bool MAX17260::getCellTemperature(float &temperature)
    {
        uint16_t raw = 0;
        if (_readReg(Temp_Reg, raw))
        {
            int16_t signedRaw = static_cast<int16_t>(raw);
            temperature = static_cast<float>(signedRaw) / 256.0f;
            return true;
        }
        return false;
    }

    bool MAX17260::getCycles(uint16_t &cycles)
    {
        uint16_t raw = 0;
        if (_readReg(Cycles_Reg, raw))
        {
            cycles = static_cast<uint16_t>((raw + 50u) / 100u);
            return true;
        }
        return false;
    }

    bool MAX17260::getSOH(uint8_t &percent)
    {
        uint16_t fullCap = 0;
        uint16_t designCap = 0;
        if (_readReg(FullCapRep_Reg, fullCap) && _readReg(DesignCap_Reg, designCap) && designCap != 0)
        {
            uint32_t value = (static_cast<uint32_t>(fullCap) * 100u + (designCap / 2u)) / designCap;
            percent = static_cast<uint8_t>(std::min<uint32_t>(value, 100u));
            return true;
        }
        return false;
    }

    bool MAX17260::getInitialized(bool& state)
    {
        uint16_t por = 0;
        if (_readReg(Status_POR, por))
        {
            state = (por == 0);
            return true;
        }
        return false;
    }

    bool MAX17260::setEnabled(bool enable)
    {
        return _writeReg(Config_SHDN, enable ? 0 : 1);
    }

    bool MAX17260::setCellTemperature(float temperature)
    {
        int32_t raw = static_cast<int32_t>(std::lround(temperature * 256.0f));
        if (raw < -32768)
        {
            raw = -32768;
        }
        else if (raw > 32767)
        {
            raw = 32767;
        }
        uint16_t encoded = static_cast<uint16_t>(static_cast<int16_t>(raw));
        return _writeReg(Temp_Reg, encoded);
    }

    bool MAX17260::enableTSENSE(bool enableTsense1, bool enableTsense2)
    {
        uint16_t config = 0;
        if (!_readReg(Config_All, config))
        {
            return false;
        }

        uint16_t newConfig = config;
        const uint16_t bitTSel = static_cast<uint16_t>(1u << 15);
        const uint16_t bitTEn = static_cast<uint16_t>(1u << 9);
        const uint16_t bitTEx = static_cast<uint16_t>(1u << 8);
        const uint16_t bitETHRM = static_cast<uint16_t>(1u << 4);
        const uint16_t bitFTHRM = static_cast<uint16_t>(1u << 3);

        newConfig &= ~bitTEn;
        newConfig &= ~bitTEx;
        newConfig &= ~bitTSel;
        newConfig &= ~bitETHRM;
        newConfig &= ~bitFTHRM;

        if (enableTsense2)
        {
            newConfig |= bitTEn;
            newConfig &= ~bitTEx;
            newConfig |= bitTSel;
            newConfig |= bitETHRM;
        }
        else if (enableTsense1)
        {
            newConfig |= bitTEn;
            newConfig &= ~bitTEx;
        }
        else
        {
            newConfig |= bitTEn;
            newConfig |= bitTEx;
        }

        if (newConfig == config)
        {
            return true;
        }

        return _writeReg(Config_All, newConfig);
    }

    bool MAX17260::setLowVoltageAlarm(uint16_t voltage)
    {
        uint16_t current = 0;
        if (!_readReg(VAlrtTh_All, current))
        {
            return false;
        }

        uint8_t high = static_cast<uint8_t>(current >> 8);
        uint8_t raw = 0;
        if (voltage != 0)
        {
            uint32_t value = (static_cast<uint32_t>(voltage) + 10u) / 20u;
            raw = static_cast<uint8_t>(std::min<uint32_t>(value, 0xFFu));
        }

        uint16_t updated = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | raw);
        if (updated == current)
        {
            return true;
        }
        return _writeReg(VAlrtTh_All, updated);
    }

    bool MAX17260::setHighVoltageAlarm(uint16_t voltage)
    {
        uint16_t current = 0;
        if (!_readReg(VAlrtTh_All, current))
        {
            return false;
        }

        uint8_t low = static_cast<uint8_t>(current & 0xFF);
        uint8_t raw = 0xFF;
        if (voltage != 0)
        {
            uint32_t value = (static_cast<uint32_t>(voltage) + 10u) / 20u;
            raw = static_cast<uint8_t>(std::min<uint32_t>(value, 0xFFu));
        }

        uint16_t updated = static_cast<uint16_t>((static_cast<uint16_t>(raw) << 8) | low);
        if (updated == current)
        {
            return true;
        }
        return _writeReg(VAlrtTh_All, updated);
    }

    bool MAX17260::setLowRSOCAlarm(uint8_t percent)
    {
        uint16_t current = 0;
        if (!_readReg(SAlrtTh_All, current))
        {
            return false;
        }

        uint8_t high = static_cast<uint8_t>(current >> 8);
        uint8_t raw = 0;
        if (percent != 0)
        {
            raw = std::min<uint8_t>(percent, 100);
        }

        uint16_t updated = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | raw);
        if (updated == current)
        {
            return true;
        }
        return _writeReg(SAlrtTh_All, updated);
    }

    bool MAX17260::setTerminationFactor(float factor)
    {
        if (factor <= 0.0f)
        {
            return false;
        }

        uint16_t designCap = 0;
        if (!_readReg(DesignCap_Reg, designCap) || designCap == 0)
        {
            return false;
        }

        float scaled = static_cast<float>(designCap) * factor * 3.2f;
        uint32_t raw = static_cast<uint32_t>(std::lround(scaled));
        raw = std::min<uint32_t>(raw, 0xFFFFu);
        return _writeReg(IChgTerm_Reg, static_cast<uint16_t>(raw));
    }

    bool MAX17260::setInitialized()
    {
        return _writeReg(Status_POR, 0);
    }

    bool MAX17260::clearLowVoltageAlarm()
    {
        return _writeReg(Status_Vmn, 0);
    }

    bool MAX17260::clearHighVoltageAlarm()
    {
        return _writeReg(Status_Vmx, 0);
    }

    bool MAX17260::clearLowRSOCAlarm()
    {
        return _writeReg(Status_Smn, 0);
    }
}
