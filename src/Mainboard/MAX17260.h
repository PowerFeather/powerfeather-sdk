/**
 *  POWERFEATHER 4-CLAUSE LICENSE
 *
 *  Copyright (C) 2025, PowerFeather.
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
#include <climits>

#include "FuelGauge.h"

namespace PowerFeather
{
    class MAX17260 : public FuelGauge
    {
    private:
        static constexpr uint16_t _fStatDNRWaitTime = 10; // from software implementation guide

        struct Register
        {
            uint8_t address;
            uint8_t start;
            uint8_t end;
        };

        static constexpr uint8_t _i2cAddress = 0x6c;
        static constexpr uint8_t _regSize = 2;
        static constexpr uint8_t _lastBit = (_regSize * CHAR_BIT) - 1;

        const Register Status_POR =            { 0x00, 1, 1 };
        const Register FStat_DNR =             { 0x3D, 0, 0 };

        const Register Status_All =            { 0x00, 0, _lastBit };
        const Register Status_Vmn =            { 0x00, 8, 8 };
        const Register Status_Vmx =            { 0x00, 12, 12 };
        const Register Status_Smn =            { 0x00, 10, 10 };

        const Register Config_All =            { 0x1D, 0, _lastBit };
        const Register Config_TSel =           { 0x1D, 15, 15 };
        const Register Config_TEn =            { 0x1D, 9, 9 };
        const Register Config_TEx =            { 0x1D, 8, 8 };
        const Register Config_SHDN =           { 0x1D, 7, 7 };
        const Register Config_ETHRM =          { 0x1D, 4, 4 };
        const Register Config_FTHRM =          { 0x1D, 3, 3 };

        const Register VAlrtTh_All =           { 0x01, 0, _lastBit };
        const Register SAlrtTh_All =           { 0x03, 0, _lastBit };

        const Register RepSOC_Reg =            { 0x06, 0, _lastBit };
        const Register Temp_Reg =              { 0x08, 0, _lastBit };
        const Register VCell_Reg =             { 0x09, 0, _lastBit };
        const Register FullCapRep_Reg =        { 0x10, 0, _lastBit };
        const Register TTE_Reg =               { 0x11, 0, _lastBit };
        const Register Cycles_Reg =            { 0x17, 0, _lastBit };
        const Register DesignCap_Reg =         { 0x18, 0, _lastBit };
        const Register IChgTerm_Reg =          { 0x1E, 0, _lastBit };
        const Register TTF_Reg =               { 0x20, 0, _lastBit };

        bool _readReg(Register reg, uint16_t &value);
        bool _writeReg(Register reg, uint16_t value);

    public:
        MAX17260(MasterI2C &i2c) : FuelGauge(i2c) {}

        bool init();

        bool getEnabled(bool &enabled) override;
        bool getCellVoltage(uint16_t &voltage) override;
        bool getRSOC(uint8_t &percent) override;
        bool getTimeToEmpty(uint16_t &minutes) override;
        bool getTimeToFull(uint16_t &minutes) override;
        bool getCellTemperature(float &temperature) override;
        bool getCycles(uint16_t &cycles) override;
        bool getSOH(uint8_t &percent) override;
        bool getInitialized(bool& state) override;
        bool setEnabled(bool enable) override;
        bool setCellTemperature(float temperature) override;
        bool enableTSENSE(bool enableTsense1, bool enableTsense2) override;
        bool setLowVoltageAlarm(uint16_t voltage) override;
        bool setHighVoltageAlarm(uint16_t voltage) override;
        bool setLowRSOCAlarm(uint8_t percent) override;
        bool setTerminationFactor(float factor) override;
        bool setInitialized() override;
        bool clearLowVoltageAlarm() override;
        bool clearHighVoltageAlarm() override;
        bool clearLowRSOCAlarm() override;
    };
}
