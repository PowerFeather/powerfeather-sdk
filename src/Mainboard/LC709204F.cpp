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

#include <Utils/Util.h>
#include "LC709204F.h"

namespace PowerFeather
{
    static const char *TAG = "PowerFeather::Mainboard::LC709204F";

    bool LC709204F::readRegister(uint8_t address, uint16_t &data)
    {
        uint8_t reply[6];
        reply[0] = _i2cAddress << 1;          // write byte
        reply[1] = address;                   // register
        reply[2] = reply[0] | 0x1;            // read byte

        if (!_i2c.read(_i2cAddress, reply[1], &(reply[3]), 3))
        {
            ESP_LOGD(TAG, "Read register %02x failed.", reply[1]);
            return false;
        }

        uint8_t crc = _computeCRC8(reply, 5);

        if (crc != reply[5]) // crc failed
        {
            ESP_LOGD(TAG, "CRC %02x different from expected %02x.", reply[5], crc);
            return false;
        }

        data = reply[4];
        data <<= 8;
        data |= reply[3];

        ESP_LOGD(TAG, "Read register %02x succeeded, crc: %02x  value: %04x", reply[1], crc, data);
        return true;
    }

    bool LC709204F::writeRegister(uint8_t address, uint16_t data)
    {
        uint8_t send[5];
        send[0] = _i2cAddress << 1;
        send[1] = address;
        send[2] = data & 0xFF;
        send[3] = data >> 8;
        send[4] = _computeCRC8(send, 4);
        ESP_LOGD(TAG, "Write register %02x, crc: %02x  value: %04x", send[1], send[4], data);
        return _i2c.write(_i2cAddress, send[1], &(send[2]), 3);
    }

    bool LC709204F::probe()
    {
        uint16_t value = 0;
        return readRegister(static_cast<uint8_t>(Register::IC_Power_Mode), value);
    }

    uint8_t LC709204F::_computeCRC8(uint8_t *data, int len)
    {
        const uint8_t polynomial = 0x07;
        uint8_t crc = 0x00;

        for (int j = len; j; --j)
        {
            crc ^= *data++;

            for (int i = 8; i; --i)
            {
                crc = (crc & 0x80) ? (crc << 1) ^ polynomial : (crc << 1);
            }
        }
        return crc;
    }

    bool LC709204F::_setVoltageAlarm(Register reg, uint16_t voltage)
    {
        if (voltage == 0 || (voltage >= LC709204F::MinVoltageAlarm && voltage <= LC709204F::MaxVoltageAlarm))
        {
            return writeRegister(static_cast<uint8_t>(reg), voltage);
        }
        return false;
    }

    bool LC709204F::_clearAlarm(const Field &alarmField)
    {
        return writeField(alarmField, 0);
    }

    bool LC709204F::getEnabled(bool &enabled)
    {
        uint16_t value = 0;
        bool res = readRegister(static_cast<uint8_t>(Register::IC_Power_Mode), value);
        if (res)
        {
            enabled = (value == static_cast<uint16_t>(OperationMode::OperationalMode));
        }
        return res;
    }

    bool LC709204F::getCellVoltage(uint16_t &voltage)
    {
        return readRegister(static_cast<uint8_t>(Register::Cell_Voltage), voltage);
    }

    bool LC709204F::getRSOC(uint8_t &percent)
    {
        uint16_t value = 0;
        if (readRegister(static_cast<uint8_t>(Register::RSOC), value))
        {
            percent = static_cast<uint8_t>(value);
            return true;
        }
        return false;
    }

    bool LC709204F::getTimeToEmpty(uint16_t &minutes)
    {
        return readRegister(static_cast<uint8_t>(Register::TimeToEmpty), minutes);
    }

    bool LC709204F::getTimeToFull(uint16_t &minutes)
    {
        return readRegister(static_cast<uint8_t>(Register::TimeToFull), minutes);
    }

    bool LC709204F::getCellTemperature(float& temperature)
    {
        uint16_t value = 0;
        if (readRegister(static_cast<uint8_t>(Register::TSENSE1), value))
        {
            temperature = Util::fromRaw(value, 0.1, 0x0DCC) + 80.0f;
            return true;
        }
        return false;
    }

    bool LC709204F::getCycles(uint16_t &cycles)
    {
        return readRegister(static_cast<uint8_t>(Register::Cycle_Count), cycles);
    }

    bool LC709204F::getSOH(uint8_t &percent)
    {
        uint16_t value = 0;
        if (readRegister(static_cast<uint8_t>(Register::State_Of_Health), value))
        {
            percent = static_cast<uint8_t>(value);
            return true;
        }
        return false;
    }

    bool LC709204F::getInitialized(bool& state)
    {
        uint16_t bit = 0;
        if (readField(Fields::BatteryStatus::Initialized, bit))
        {
            state = (bit == 0);
            return true;
        }
        return false;
    }

    bool LC709204F::setEnabled(bool enable)
    {
        uint16_t value = static_cast<uint16_t>(enable ? OperationMode::OperationalMode : OperationMode::SleepMode);
        return writeRegister(static_cast<uint8_t>(Register::IC_Power_Mode), value);
    }

    bool LC709204F::setAPA(uint16_t capacity, ChangeOfParameter changeOfParam)
    {
        if (capacity >= MinBatteryCapacity && capacity <= MaxBatteryCapacity)
        {
            if (changeOfParam == ChangeOfParameter::ICR18650_26H)
            {
                return writeRegister(static_cast<uint8_t>(Register::APA), 0x0606);
            }
            else if (changeOfParam == ChangeOfParameter::UR18650ZY)
            {
                return writeRegister(static_cast<uint8_t>(Register::APA), 0x1010);
            }
            else
            {
                auto prev = _apaTable[0];
                for (int i = 0; i < sizeof(_apaTable) / sizeof(prev); i++)
                {
                    auto cur = _apaTable[i];
                    uint16_t cap = std::get<0>(cur);
                    uint16_t apa = 0;

                    if (capacity == cap)
                    {
                        apa = (std::get<1>(cur) << 8) | std::get<1>(cur);
                    }
                    else
                    {
                        auto prev = _apaTable[i - 1];
                        uint16_t prev_cap = std::get<0>(prev);
                        if (capacity < cap && (prev != cur && cap > prev_cap))
                        {
                            float res = round(std::get<1>(prev) + (std::get<1>(cur) - std::get<1>(prev)) * ((static_cast<float>(capacity) - prev_cap) / (cap - prev_cap)));
                            apa = (static_cast<uint8_t>(res) << 8) | static_cast<uint8_t>(res);
                        }
                    }

                    if (apa)
                    {
                        return writeRegister(static_cast<uint8_t>(Register::APA), apa);
                    }

                    prev = cur;
                }
            }
        }
        return false;
    }

    bool LC709204F::setChangeOfParameter(ChangeOfParameter changeOfParam)
    {
        return writeRegister(static_cast<uint8_t>(Register::Change_Of_The_Parameter), static_cast<uint16_t>(changeOfParam));
    }

    bool LC709204F::setCellTemperature(float temperature)
    {
        uint16_t value = Util::toRaw(temperature - 80.0f, 0.1f, 0xDCC);
        return writeRegister(static_cast<uint8_t>(Register::TSENSE1), value);
    }

    bool LC709204F::enableTSENSE(bool enableTsense1, bool enableTsense2)
    {
        uint16_t status = enableTsense1 << 0 | enableTsense2 << 1;
        return writeRegister(static_cast<uint8_t>(Register::Status_Bit), status);
    }

    bool LC709204F::setLowVoltageAlarm(uint16_t voltage)
    {
        return _setVoltageAlarm(Register::Alarm_Low_Cell_Voltage, voltage);
    }

    bool LC709204F::setHighVoltageAlarm(uint16_t voltage)
    {
        return _setVoltageAlarm(Register::Alarm_High_Cell_Voltage, voltage);
    }

    bool LC709204F::setLowRSOCAlarm(uint8_t percent)
    {
        if (percent <= 100)
        {
            return writeRegister(static_cast<uint8_t>(Register::Alarm_Low_RSOC), static_cast<uint16_t>(percent));
        }
        return false;
    }

    bool LC709204F::setTerminationFactor(float factor)
    {
        if (factor >= LC709204F::MinTerminationFactor && factor <= LC709204F::MaxTerminationFactor)
        {
            return writeRegister(static_cast<uint8_t>(Register::Termination_Current_Rate), static_cast<uint16_t>(factor * 100));
        }
        return false;
    }

    bool LC709204F::setInitialized()
    {
        return writeField(Fields::BatteryStatus::Initialized, 0);
    }

    bool LC709204F::clearLowVoltageAlarm()
    {
        return _clearAlarm(Fields::BatteryStatus::LowVoltage);
    }

    bool LC709204F::clearHighVoltageAlarm()
    {
        return _clearAlarm(Fields::BatteryStatus::HighVoltage);
    }

    bool LC709204F::clearLowRSOCAlarm()
    {
        return _clearAlarm(Fields::BatteryStatus::LowRSOC);
    }

}
