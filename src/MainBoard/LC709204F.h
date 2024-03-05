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

#pragma once

#include <tuple>

#include "Utils/MasterI2C.h"

namespace PowerFeather
{
    class LC709204F
    {
    public:

        enum class Registers
        {
            Cell_Voltage = 0x09,
            RSOC = 0x0d,
            State_Of_Health = 0x32,
            TimeToEmpty = 0x03,
            TimeToFull = 0x05,

            APA = 0x0b,
            Change_Of_The_Parameter = 0x12,
            Status_Bit = 0x16,
            Alarm_Low_RSOC = 0x13,
            Alarm_Low_Cell_Voltage = 0x14,
            IC_Power_Mode = 0x15,
            Cycle_Count = 0x17,
            Alarm_High_Cell_Voltage = 0x1f
        };

        enum class ChangeOfParameter
        {
            Nominal_3V7_Charging_4V2 = 0x00,
            UR18650ZY = 0x01,
            ICR18650_26H = 0x02,
            Nominal_3V8_Charging_4V35= 0x03,
            Nominal_3V85_Charging_4V4= 0x04,
        };

        LC709204F(MasterI2C& i2c):_i2c(i2c) {}

        bool getCellVoltage(uint16_t& mV);
        bool getRSOC(uint8_t& rsoc);
        bool getSOH(uint8_t& rsoh);
        bool getTimeToEmpty(uint16_t& minutes);
        bool getTimeToFull(uint16_t& minutes);
        bool getCycles(uint16_t& cycles);
        bool getOperation(bool& enabled);

        bool setAPA(uint16_t mAh, ChangeOfParameter param);
        bool setChangeOfParameter(ChangeOfParameter param);
        bool setLowVoltageAlarm(uint16_t mV);
        bool setHighVoltageAlarm(uint16_t mV);
        bool setLowRSOCAlarm(uint16_t rsoc);
        bool enableTSENSE(bool tsense1, bool tsense2);
        bool enableOperation(bool enable);

    private:
        const std::tuple<uint16_t, uint8_t> _apaTable[10] =
        {
            { 50, 0x13 },
            { 100, 0x15 },
            { 200, 0x18 },
            { 500, 0x21 },
            { 1000, 0x2D },
            { 2000, 0x3A },
            { 3000, 0x3F },
            { 4000, 0x42 },
            { 5000, 0x44 },
            { 6000, 0x45 },
        };

        static constexpr uint8_t _i2cAddress = 0x0b;

        MasterI2C& _i2c;

        bool setVoltageAlarm(Registers reg, uint16_t mV);

        bool readReg(Registers reg, uint16_t& data);
        bool writeReg(Registers reg, uint16_t data);
        uint8_t computeCRC8(uint8_t *data, int len);
    };
}