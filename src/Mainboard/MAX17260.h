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

#include <array>

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
        static constexpr uint8_t VAlrtTh_Register = 0x01;
        static constexpr uint8_t SAlrtTh_Register = 0x03;
        static constexpr uint8_t RepCap_Register = 0x05;
        static constexpr uint8_t RepSOC_Register = 0x06;
        static constexpr uint8_t Temp_Register = 0x08;
        static constexpr uint8_t VCell_Register = 0x09;
        static constexpr uint8_t Current_Register = 0x0A;
        static constexpr uint8_t AvgCurrent_Register = 0x0B;
        static constexpr uint8_t MixCap_Register = 0x0F;
        static constexpr uint8_t FullCapRep_Register = 0x10;
        static constexpr uint8_t TTE_Register = 0x11; // 0x11 is TTE
        static constexpr uint8_t QRTable00_Register = 0x12;
        static constexpr uint8_t FullSOCThr_Register = 0x13;
        static constexpr uint8_t Cycles_Register = 0x17;
        static constexpr uint8_t DesignCap_Register = 0x18;
        static constexpr uint8_t Config_Register = 0x1D;
        static constexpr uint8_t IChgTerm_Register = 0x1E;
        static constexpr uint8_t AvCap_Register = 0x1F;
        static constexpr uint8_t TTF_Register = 0x20;
        static constexpr uint8_t DevName_Register = 0x21;
        static constexpr uint8_t QRTable10_Register = 0x22;
        static constexpr uint8_t FullCapNom_Register = 0x23;
        static constexpr uint8_t LearnCfg_Register = 0x28;
        static constexpr uint8_t RelaxCfg_Register = 0x2A;
        static constexpr uint8_t MiscCfg_Register = 0x2B;
        static constexpr uint8_t TGain_Register = 0x2C;
        static constexpr uint8_t TOff_Register = 0x2D;
        static constexpr uint8_t QRTable20_Register = 0x32;
        static constexpr uint8_t RComp0_Register = 0x38;
        static constexpr uint8_t TempCo_Register = 0x39;
        static constexpr uint8_t VEmpty_Register = 0x3A;
        static constexpr uint8_t QRTable30_Register = 0x42;
        static constexpr uint8_t dQAcc_Register = 0x45;
        static constexpr uint8_t dPAcc_Register = 0x46;
        static constexpr uint8_t Command_Register = 0x60;
        static constexpr uint8_t UnlockModel1_Register = 0x62;
        static constexpr uint8_t UnlockModel2_Register = 0x63;
        static constexpr uint8_t ModelTableStart_Register = 0x80;
        static constexpr uint8_t RCompSeg_Register = 0xAF;
        static constexpr uint8_t Curve_Register = 0xB9;
        static constexpr uint8_t HibCfg_Register = 0xBA;
        static constexpr uint8_t Config2_Register = 0xBB;
        static constexpr uint8_t ModelCfg_Register = 0xDB;

        static constexpr uint16_t DevName_MAX17260 = 0x4031;
        static constexpr uint16_t UnlockKey1 = 0x0059;
        static constexpr uint16_t UnlockKey2 = 0x00C4;
        static constexpr uint16_t HibernateExit_Command1 = 0x0090;
        static constexpr uint16_t HibernateExit_Command2 = 0x0000;

        static constexpr uint16_t ConfigBit_TSel = 1u << 15;
        static constexpr uint16_t ConfigBit_TEn = 1u << 9;
        static constexpr uint16_t ConfigBit_TEx = 1u << 8;
        static constexpr uint16_t ConfigBit_ETHRM = 1u << 4;
        static constexpr uint16_t ConfigBit_FTHRM = 1u << 3;

    public:
        static constexpr uint8_t ModelID_LiCoO2 = 2;
        static constexpr uint8_t ModelID_LFP = 6;
        const Field FStat_DNR =             { 0x3D, 0, 0 };
        const Field Status_POR =            { 0x00, 1, 1 };
        const Field Status_Vmn =            { 0x00, 8, 8 };
        const Field Status_Vmx =            { 0x00, 12, 12 };
        const Field Status_Smn =            { 0x00, 10, 10 };
        const Field Config_SHDN =           { 0x1D, 7, 7 };

        bool readRegister(uint8_t address, uint16_t &value) override;
        bool writeRegister(uint8_t address, uint16_t value) override;
        bool _waitForDNRClear();

    public:
        struct Model
        {
            std::array<uint16_t, 32> modelTable{};
            uint16_t rComp0{0};
            uint16_t tempCo{0};
            uint16_t rCompSeg{0};
            uint16_t designCap{0};
            uint16_t ichgTerm{0};
            uint16_t vEmpty{0};
            uint16_t modelCfg{0};
            uint16_t learnCfg{0};
            uint16_t relaxCfg{0};
            uint16_t config{0};
            uint16_t config2{0};
            uint16_t miscCfg{0};
            uint16_t fullSocThr{0};
            uint16_t tGain{0};
            uint16_t tOff{0};
            uint16_t curve{0};
            std::array<uint16_t, 4> qrTable{{0, 0, 0, 0}};
            uint16_t chargeVoltageMv{0};
        };

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

        bool setModelID(uint8_t modelId);
        bool loadModel(const Model &model);

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
