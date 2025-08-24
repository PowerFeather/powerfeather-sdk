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

        const Register Status_POR =            { 0x00, 1, 1 };
        const Register FStat_DNR =             { 0x3D, 0, 0 };

        bool _readReg(Register reg, uint16_t &value);
        bool _writeReg(Register reg, uint16_t value);

        static constexpr uint8_t _i2cAddress = 0x6c;
        static constexpr uint8_t _regSize = 2;

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
