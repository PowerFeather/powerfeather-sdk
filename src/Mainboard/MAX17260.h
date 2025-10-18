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

#include "FuelGauge.h"

namespace PowerFeather
{
    class MAX17260 : public RegisterFuelGauge
    {
    private:
        using Field = RegisterFuelGauge::RegisterField;

        static constexpr uint8_t RegisterSize = 2;
        static constexpr uint16_t _fStatDNRWaitTime = 10; // from software implementation guide
        static constexpr uint16_t MinVoltageAlarm = 0;
        static constexpr uint16_t MaxVoltageAlarm = 5120;
        static constexpr float MinTemperature = -128.0f;
        static constexpr float MaxTemperature = 127.996f;
        static constexpr float MinTermination = 0.01f;
        static constexpr float MaxTermination = 1.0f;

        static constexpr uint8_t _i2cAddress = 0x6c;
        static constexpr uint8_t Config_Register = 0x1D;
        static constexpr uint16_t ConfigBit_TSel = 1u << 15;
        static constexpr uint16_t ConfigBit_TEn = 1u << 9;
        static constexpr uint16_t ConfigBit_TEx = 1u << 8;
        static constexpr uint16_t ConfigBit_ETHRM = 1u << 4;
        static constexpr uint16_t ConfigBit_FTHRM = 1u << 3;

        static constexpr uint8_t VAlrtTh_Register = 0x01;
        static constexpr uint8_t SAlrtTh_Register = 0x03;
        static constexpr uint8_t RepSOC_Register = 0x06;
        static constexpr uint8_t Temp_Register = 0x08;
        static constexpr uint8_t VCell_Register = 0x09;
        static constexpr uint8_t FullCapRep_Register = 0x10;
        static constexpr uint8_t TTE_Register = 0x11;
        static constexpr uint8_t Cycles_Register = 0x17;
        static constexpr uint8_t DesignCap_Register = 0x18;
        static constexpr uint8_t IChgTerm_Register = 0x1E;
        static constexpr uint8_t TTF_Register = 0x20;

        const Field Status_POR =            { 0x00, 1, 1 };
        const Field FStat_DNR =             { 0x3D, 0, 0 };
        const Field Status_Vmn =            { 0x00, 8, 8 };
        const Field Status_Vmx =            { 0x00, 12, 12 };
        const Field Status_Smn =            { 0x00, 10, 10 };
        const Field Config_SHDN =           { 0x1D, 7, 7 };

        bool readRegister(uint8_t address, uint16_t &value) override;
        bool writeRegister(uint8_t address, uint16_t value) override;

    public:
        MAX17260(MasterI2C &i2c) : RegisterFuelGauge(i2c, RegisterSize) {}

        bool init();

        bool probe() override;
        const char *name() const override { return "MAX17260"; }
        bool voltageAlarmRange(uint16_t &minMv, uint16_t &maxMv) const override
        {
            minMv = MinVoltageAlarm;
            maxMv = MaxVoltageAlarm;
            return true;
        }
        bool temperatureRange(float &minC, float &maxC) const override
        {
            minC = MinTemperature;
            maxC = MaxTemperature;
            return true;
        }
        float minTerminationFactor() const override { return MinTermination; }
        float maxTerminationFactor() const override { return MaxTermination; }

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
