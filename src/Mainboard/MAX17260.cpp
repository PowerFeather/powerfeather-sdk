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

    bool MAX17260::readRegister(uint8_t address, uint16_t &value)
    {
        if (_i2c.read(_i2cAddress, address, reinterpret_cast<uint8_t *>(&value), registerSizeBytes()))
        {
            ESP_LOGD(TAG, "Read register %02x, value = %04x.", address, value);
            return true;
        }

        ESP_LOGD(TAG, "Read register %02x failed.", address);
        return false;
    }

    bool MAX17260::writeRegister(uint8_t address, uint16_t value)
    {
        bool res = _i2c.write(_i2cAddress, address, reinterpret_cast<uint8_t *>(&value), registerSizeBytes());
        if (res)
        {
            ESP_LOGD(TAG, "Write register %02x, value = %04x.", address, value);
        }
        else
        {
            ESP_LOGD(TAG, "Write register %02x failed.", address);
        }
        return res;
    }

    bool MAX17260::_waitForDNRClear()
    {
        constexpr uint16_t maxAttempts = 100;
        uint16_t attempts = 0;
        uint16_t dnr = 0;

        do
        {
            if (!readField(Fields::FStat::DNR, dnr))
            {
                return false;
            }

            if (dnr == 0)
            {
                return true;
            }

            vTaskDelay(pdMS_TO_TICKS(_fStatDNRWaitTime));
        } while (++attempts < maxAttempts);

        ESP_LOGW(TAG, "FSTAT.DNR remained set after waiting.");
        return false;
    }

    bool MAX17260::probe()
    {
        uint16_t value = 0;
        return readField(Fields::Status::POR, value);
    }

    bool MAX17260::init()
    {
        uint16_t por = 0;
        if (!readField(Fields::Status::POR, por))
        {
            return false;
        }

        if (por != 0)
        {
            return _waitForDNRClear();
        }

        return true;
    }

bool MAX17260::setModelID(uint8_t modelId)
{
    uint16_t cfg = 0;
    if (!readRegister(Register::ModelCfg, cfg))
    {
        return false;
        }

        cfg &= static_cast<uint16_t>(~(0x7u << 5));
        cfg |= static_cast<uint16_t>((modelId & 0x7u) << 5);
        cfg |= static_cast<uint16_t>(1u << 15); // Refresh

        if (!writeRegister(Register::ModelCfg, cfg))
        {
            return false;
        }

    return _waitForDNRClear();
}

bool MAX17260::loadModel(const Model &model)
{
    if (!_waitForDNRClear())
    {
        return false;
    }

    uint16_t hibCfg = 0;
    if (!readRegister(Register::HibCfg, hibCfg))
    {
        return false;
    }

    if (!writeRegister(Register::Command, HibernateExit_Command1)) return false;
    if (!writeRegister(Register::HibCfg, HibernateExit_Command2)) return false;
    if (!writeRegister(Register::Command, HibernateExit_Command2)) return false;

    if (!writeRegister(Register::UnlockModel1, UnlockKey1)) return false;
    if (!writeRegister(Register::UnlockModel2, UnlockKey2)) return false;

    const uint8_t modelTableBase = static_cast<uint8_t>(Register::ModelTableStart);
    for (size_t i = 0; i < 16; ++i)
    {
        if (!writeRegister(static_cast<uint8_t>(modelTableBase + i), model.modelTable[i]))
        {
            return false;
        }
    }

    for (size_t i = 0; i < 16; ++i)
    {
        if (!writeRegister(static_cast<uint8_t>(modelTableBase + 16 + i), model.modelTable[16 + i]))
        {
            return false;
        }
    }

    if (!writeRegister(Register::RCompSeg, model.rCompSeg)) return false;

    if (!writeRegister(Register::UnlockModel1, 0x0000)) return false;
    if (!writeRegister(Register::UnlockModel2, 0x0000)) return false;

    if (!writeRegister(Register::RepCap, 0x0000)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!writeRegister(Register::DesignCap, model.designCap)) return false;

    const uint16_t fullCapNom = model.designCap;
    const uint16_t fullCapRep = model.designCap;

    if (!writeRegister(Register::FullCapRep, fullCapRep)) return false;

    const uint16_t dQAcc = static_cast<uint16_t>(fullCapNom / 2);
    constexpr uint16_t dPAcc = 0x0C80;
    bool verified = false;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        if (!writeRegister(Register::dQAcc, dQAcc)) return false;
        if (!writeRegister(Register::dPAcc, dPAcc)) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!writeRegister(Register::FullCapNom, fullCapNom)) return false;

        uint16_t checkFullCapNom = 0;
        uint16_t checkDQAcc = 0;
        uint16_t checkDPAcc = 0;
        if (!readRegister(Register::FullCapNom, checkFullCapNom) ||
            !readRegister(Register::dQAcc, checkDQAcc) ||
            !readRegister(Register::dPAcc, checkDPAcc))
        {
            return false;
        }

        if (checkFullCapNom == fullCapNom && checkDQAcc == dQAcc && checkDPAcc == dPAcc)
        {
            verified = true;
            break;
        }
    }

    if (!verified)
    {
        return false;
    }

    uint16_t vfsoc = 0;
    if (!readRegister(Register::VFSOC, vfsoc))
    {
        return false;
    }

    uint32_t updateCapacity = (static_cast<uint32_t>(vfsoc) * fullCapNom) / 25600u;
    uint16_t updateCapacity16 = static_cast<uint16_t>(std::min<uint32_t>(updateCapacity, 0xFFFFu));
    if (!writeRegister(Register::MixCap, updateCapacity16)) return false;
    if (!writeRegister(Register::AvCap, updateCapacity16)) return false;

    vTaskDelay(pdMS_TO_TICKS(200));

    if (!writeRegister(Register::IChgTerm, model.ichgTerm)) return false;
    if (!writeRegister(Register::VEmpty, model.vEmpty)) return false;
    if (!writeRegister(Register::RComp0, model.rComp0)) return false;
    if (!writeRegister(Register::TempCo, model.tempCo)) return false;

    const uint8_t qrAddresses[4] = {
        static_cast<uint8_t>(Register::QRTable00),
        static_cast<uint8_t>(Register::QRTable10),
        static_cast<uint8_t>(Register::QRTable20),
        static_cast<uint8_t>(Register::QRTable30),
    };
    for (size_t i = 0; i < model.qrTable.size(); ++i)
    {
        if (!writeRegister(qrAddresses[i], model.qrTable[i]))
        {
            return false;
        }
    }

    if (!writeRegister(Register::LearnCfg, model.learnCfg)) return false;
    if (!writeRegister(Register::RelaxCfg, model.relaxCfg)) return false;
    if (!writeRegister(Register::Config, model.config)) return false;
    if (!writeRegister(Register::MiscCfg, model.miscCfg)) return false;
    if (!writeRegister(Register::FullSOCThr, model.fullSocThr)) return false;
    if (!writeRegister(Register::TGain, model.tGain)) return false;
    if (!writeRegister(Register::TOff, model.tOff)) return false;
    if (!writeRegister(Register::Curve, model.curve)) return false;

    if (!writeRegister(Register::ModelCfg, model.modelCfg)) return false;

    for (int i = 0; i < 50; ++i)
    {
        uint16_t cfg = 0;
        if (!readRegister(Register::ModelCfg, cfg))
        {
            return false;
        }

        if ((cfg & 0x8000u) == 0)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        if (i == 49)
        {
            return false;
        }
    }

    uint16_t config2 = model.config2;
    if (!writeRegister(Register::Config2, static_cast<uint16_t>(config2 | 0x0020))) return false;

    for (int i = 0; i < 200; ++i)
    {
        uint16_t cfg2 = 0;
        if (!readRegister(Register::Config2, cfg2))
        {
            return false;
        }

        if ((cfg2 & 0x0020u) == 0)
        {
            break;
        }

        if (!writeRegister(Register::Current, 0x0000)) return false;
        if (!writeRegister(Register::AvgCurrent, 0x0000)) return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        if (i == 199)
        {
            return false;
        }
    }

    if (!writeRegister(Register::Config2, config2)) return false;

    if (!writeRegister(Register::QRTable20, model.qrTable[2])) return false;
    if (!writeRegister(Register::QRTable30, model.qrTable[3])) return false;
    if (!writeRegister(Register::Cycles, 0x0000)) return false;

    if (!writeRegister(Register::HibCfg, hibCfg)) return false;

    return _waitForDNRClear();
}

    bool MAX17260::getEnabled(bool &enabled)
    {
        uint16_t value = 0;
        if (readField(Fields::Config::SHDN, value))
        {
            enabled = (value == 0);
            return true;
        }
        return false;
    }

    bool MAX17260::getCellVoltage(uint16_t &voltage)
    {
        uint16_t raw = 0;
        if (readRegister(Register::VCell, raw))
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
        if (readRegister(Register::RepSOC, raw))
        {
            percent = static_cast<uint8_t>(std::min<uint16_t>(100, (raw + 0x80) >> 8));
            return true;
        }
        return false;
    }

    bool MAX17260::getTimeToEmpty(uint16_t &minutes)
    {
        uint16_t raw = 0;
        if (readRegister(Register::TTE, raw))
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
        if (readRegister(Register::TTF, raw))
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
        if (readRegister(Register::Temp, raw))
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
        if (readRegister(Register::Cycles, raw))
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
        if (readRegister(Register::FullCapRep, fullCap) &&
            readRegister(Register::DesignCap, designCap) &&
            designCap != 0)
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
        if (readField(Fields::Status::POR, por))
        {
            state = (por == 0);
            return true;
        }
        return false;
    }

    bool MAX17260::setEnabled(bool enable)
    {
        return writeField(Fields::Config::SHDN, enable ? 0 : 1);
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
        return writeRegister(Register::Temp, encoded);
    }

    bool MAX17260::enableTSENSE(bool enableTsense1, bool enableTsense2)
    {
        uint16_t config = 0;
        if (!readRegister(Register::Config, config))
        {
            return false;
        }

        uint16_t newConfig = config;
        newConfig &= static_cast<uint16_t>(~ConfigBit_TEn);
        newConfig &= static_cast<uint16_t>(~ConfigBit_TEx);
        newConfig &= static_cast<uint16_t>(~ConfigBit_TSel);
        newConfig &= static_cast<uint16_t>(~ConfigBit_ETHRM);
        newConfig &= static_cast<uint16_t>(~ConfigBit_FTHRM);

        if (enableTsense2)
        {
            newConfig |= ConfigBit_TEn;
            newConfig &= static_cast<uint16_t>(~ConfigBit_TEx);
            newConfig |= ConfigBit_TSel;
            newConfig |= ConfigBit_ETHRM;
        }
        else if (enableTsense1)
        {
            newConfig |= ConfigBit_TEn;
            newConfig &= static_cast<uint16_t>(~ConfigBit_TEx);
        }
        else
        {
            newConfig |= ConfigBit_TEn;
            newConfig |= ConfigBit_TEx;
        }

        return (newConfig == config) ? true : writeRegister(Register::Config, newConfig);
    }

    bool MAX17260::setLowVoltageAlarm(uint16_t voltage)
    {
        uint16_t current = 0;
        if (!readRegister(Register::VAlrtTh, current))
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
        return (updated == current) ? true : writeRegister(Register::VAlrtTh, updated);
    }

    bool MAX17260::setHighVoltageAlarm(uint16_t voltage)
    {
        uint16_t current = 0;
        if (!readRegister(Register::VAlrtTh, current))
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
        return (updated == current) ? true : writeRegister(Register::VAlrtTh, updated);
    }

    bool MAX17260::setLowRSOCAlarm(uint8_t percent)
    {
        uint16_t current = 0;
        if (!readRegister(Register::SAlrtTh, current))
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
        return (updated == current) ? true : writeRegister(Register::SAlrtTh, updated);
    }

    bool MAX17260::setTerminationFactor(float factor)
    {
        if (factor <= 0.0f)
        {
            return false;
        }

        uint16_t designCap = 0;
        if (!readRegister(Register::DesignCap, designCap) || designCap == 0)
        {
            return false;
        }

        float scaled = static_cast<float>(designCap) * factor * 3.2f;
        uint32_t raw = static_cast<uint32_t>(std::lround(scaled));
        raw = std::min<uint32_t>(raw, 0xFFFFu);
        return writeRegister(Register::IChgTerm, static_cast<uint16_t>(raw));
    }

    bool MAX17260::setInitialized()
    {
        return writeField(Fields::Status::POR, 0);
    }

    bool MAX17260::clearLowVoltageAlarm()
    {
        return writeField(Fields::Status::Vmn, 0);
    }

    bool MAX17260::clearHighVoltageAlarm()
    {
        return writeField(Fields::Status::Vmx, 0);
    }

    bool MAX17260::clearLowRSOCAlarm()
    {
        return writeField(Fields::Status::Smn, 0);
    }
}
