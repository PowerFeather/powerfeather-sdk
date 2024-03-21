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

#include "LC709204F.h"

namespace PowerFeather
{
    bool LC709204F::readReg(Registers reg, uint16_t &data)
    {
        uint8_t reply[6];
        reply[0] = _i2cAddress << 1;          // write byte
        reply[1] = static_cast<uint8_t>(reg); // register
        reply[2] = reply[0] | 0x1;            // read byte

        if (!_i2c.read(_i2cAddress, reply[1], reply + 3, 3))
        {
            return false;
        }

        uint8_t crc = computeCRC8(reply, 5);

        // CRC failure?
        if (crc != reply[5])
            return false;

        data = reply[4];
        data <<= 8;
        data |= reply[3];

        return true;
    }

    bool LC709204F::writeReg(Registers reg, uint16_t data)
    {
        uint8_t send[5];
        send[0] = _i2cAddress << 1;
        send[1] = static_cast<uint8_t>(reg);
        send[2] = data & 0xFF;
        send[3] = data >> 8;
        send[4] = computeCRC8(send, 4);

        return _i2c.write(_i2cAddress, send[1], send + 2, 4);
    }

    uint8_t LC709204F::computeCRC8(uint8_t *data, int len)
    {
        const uint8_t POLYNOMIAL(0x07);
        uint8_t crc(0x00);

        for (int j = len; j; --j)
        {
            crc ^= *data++;

            for (int i = 8; i; --i)
            {
                crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
            }
        }
        return crc;
    }

    bool LC709204F::getCellVoltage(uint16_t &mV)
    {
        return readReg(Registers::Cell_Voltage, mV);
    }

    bool LC709204F::getRSOC(uint8_t &rsoc)
    {
        uint16_t val = 0;
        if (readReg(Registers::RSOC, val))
        {
            rsoc = val;
            return true;
        }
        return false;
    }

    bool LC709204F::getCycles(uint16_t &cycles)
    {
        uint16_t val = 0;
        if (readReg(Registers::Cycle_Count, val))
        {
            cycles = val;
            return true;
        }
        return false;
    }

    bool LC709204F::getSOH(uint8_t &soh)
    {
        uint16_t val = 0;
        if (readReg(Registers::State_Of_Health, val))
        {
            soh = val;
            return true;
        }
        return false;
    }

    bool LC709204F::getOperation(bool &enabled)
    {
        uint16_t val = 0;
        bool res = readReg(Registers::IC_Power_Mode, val);
        enabled = val == 0x0001;
        return res;
    }

    bool LC709204F::getTimeToEmpty(uint16_t &minutes)
    {
        return readReg(Registers::TimeToEmpty, minutes);
    }

    bool LC709204F::getTimeToFull(uint16_t &minutes)
    {
        return readReg(Registers::TimeToFull, minutes);
    }

    bool LC709204F::setAPA(uint16_t mAh, ChangeOfParameter param)
    {
        if (param == ChangeOfParameter::ICR18650_26H)
        {
            return writeReg(Registers::APA, 0x0606);
        }
        else if (param == ChangeOfParameter::UR18650ZY)
        {
            return writeReg(Registers::APA, 0x1010);
        }
        else
        {
            auto prev = _apaTable[0];
            for (int i = 0; i < sizeof(_apaTable) / sizeof(prev); i++)
            {
                auto cur = _apaTable[i];
                uint16_t cap = std::get<0>(cur);
                uint16_t apa = 0;

                if (mAh == cap)
                {
                    apa = (std::get<1>(cur) << 8) | std::get<1>(cur);
                }
                else
                {
                    auto prev = _apaTable[i - 1];
                    uint16_t prev_cap = std::get<0>(prev);
                    if (mAh < cap && (prev != cur && cap > prev_cap))
                    {
                        float val = round(std::get<1>(prev) + (std::get<1>(cur) - std::get<1>(prev)) * ((static_cast<float>(mAh) - prev_cap) / (cap - prev_cap)));
                        apa = (static_cast<uint8_t>(val) << 8) | static_cast<uint8_t>(val);
                    }
                }

                if (apa)
                {
                    return writeReg(Registers::APA, apa);
                }

                prev = cur;
            }
        }
        return false;
    }

    bool LC709204F::setVoltageAlarm(Registers reg, uint16_t mV)
    {
        uint16_t val = 0x09c4 + (mV - 2500);
        return writeReg(reg, val);
    }

    bool LC709204F::setLowVoltageAlarm(uint16_t mV)
    {
        return setVoltageAlarm(Registers::Alarm_Low_Cell_Voltage, mV);
    }

    bool LC709204F::setHighVoltageAlarm(uint16_t mV)
    {
        return setVoltageAlarm(Registers::Alarm_High_Cell_Voltage, mV);
    }

    bool LC709204F::setLowRSOCAlarm(uint16_t rsoc)
    {
        uint16_t val = 0x01 + (rsoc - 0x01);
        return writeReg(Registers::Alarm_Low_RSOC, val);
    }

    bool LC709204F::setChangeOfParameter(ChangeOfParameter param)
    {
        return writeReg(Registers::Change_Of_The_Parameter, static_cast<uint16_t>(param));
    }

    bool LC709204F::enableTSENSE(bool tsense1, bool tsense2)
    {
        uint16_t status = tsense1 << 0 | tsense2 << 1;
        return writeReg(Registers::Status_Bit, status);
    }

    bool LC709204F::enableOperation(bool enable)
    {
        return writeReg(Registers::IC_Power_Mode, enable ? 0x0001 : 0x0002);
    }

    bool LC709204F::clearLowVoltageAlarm()
    {
        return clearAlarm(11);
    }

    bool LC709204F::clearHighVoltageAlarm()
    {
        return clearAlarm(15);
    }

    bool LC709204F::clearLowRSOCAlarm()
    {
        return clearAlarm(9);
    }

    bool LC709204F::clearAlarm(uint8_t bit)
    {
        uint16_t value = 0;
        if (readReg(Registers::BatteryStatus, value))
        {
            value &= ~(0b1 << bit);
            return writeReg(Registers::BatteryStatus, value);
        }
        return false;
    }
}