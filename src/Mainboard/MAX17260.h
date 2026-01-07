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
    public:

        static constexpr uint16_t MinVoltageAlarm = 0;
        static constexpr uint16_t MaxVoltageAlarm = 5120;
        static constexpr float MinTemperature = -128.0f;
        static constexpr float MaxTemperature = 127.996f;
        static constexpr float MinTermination = 0.01f;
        static constexpr float MaxTermination = 1.0f;

        enum class Register : uint8_t
        {
            Status = 0x00,
            VAlrtTh = 0x01,
            SAlrtTh = 0x03,
            RepCap = 0x05,
            RepSOC = 0x06,
            Temp = 0x08,
            VCell = 0x09,
            Current = 0x0A,
            AvgCurrent = 0x0B,
            MixCap = 0x0F,
            FullCapRep = 0x10,
            TTE = 0x11,
            QRTable00 = 0x12,
            FullSOCThr = 0x13,
            Cycles = 0x17,
            DesignCap = 0x18,
            Config = 0x1D,
            IChgTerm = 0x1E,
            AvCap = 0x1F,
            TTF = 0x20,
            DevName = 0x21,
            QRTable10 = 0x22,
            FullCapNom = 0x23,
            LearnCfg = 0x28,
            RelaxCfg = 0x2A,
            MiscCfg = 0x2B,
            TGain = 0x2C,
            TOff = 0x2D,
            QRTable20 = 0x32,
            RComp0 = 0x38,
            TempCo = 0x39,
            VEmpty = 0x3A,
            FStat = 0x3D,
            QRTable30 = 0x42,
            dQAcc = 0x45,
            dPAcc = 0x46,
            Command = 0x60,
            UnlockModel1 = 0x62,
            UnlockModel2 = 0x63,
            ModelTableStart = 0x80,
            RCompSeg = 0xAF,
            Curve = 0xB9,
            HibCfg = 0xBA,
            Config2 = 0xBB,
            ModelCfg = 0xDB,
            VFSOC = 0xFF
        };

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

        static constexpr uint8_t RegisterSize = 2;
        static constexpr uint8_t ModelID_LiCoO2 = 0;
        static constexpr uint8_t ModelID_LFP = 6;

        MAX17260(MasterI2C &i2c) : RegisterFuelGauge(i2c, RegisterSize) {}

        bool init();

        bool probe() override;
        const char *getName() const override { return "MAX17260"; }
        void getVoltageAlarmRange(uint16_t &minMv, uint16_t &maxMv) const override
        {
            minMv = MinVoltageAlarm;
            maxMv = MaxVoltageAlarm;
        }
        void getTemperatureRange(float &minC, float &maxC) const override
        {
            minC = MinTemperature;
            maxC = MaxTemperature;
        }
        void getTerminationFactorRange(float &minFactor, float &maxFactor) const override
        {
            minFactor = MinTermination;
            maxFactor = MaxTermination;
        }

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

    private:
        using Field = RegisterFuelGauge::RegisterField;

        static constexpr uint8_t _i2cAddress = 0x36;
        static constexpr uint16_t _fStatDNRWaitTime = 10; // from software implementation guide
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

        struct Fields
        {
            struct FStat
            {
                static constexpr Field DNR = { static_cast<uint8_t>(Register::FStat), 0, 0 };
            };
            struct Status
            {
                static constexpr Field POR = { static_cast<uint8_t>(Register::Status), 1, 1 };
                static constexpr Field Vmn = { static_cast<uint8_t>(Register::Status), 8, 8 };
                static constexpr Field Smn = { static_cast<uint8_t>(Register::Status), 10, 10 };
                static constexpr Field Vmx = { static_cast<uint8_t>(Register::Status), 12, 12 };
            };
            struct Config
            {
                static constexpr Field SHDN = { static_cast<uint8_t>(Register::Config), 7, 7 };
            };
        };

        bool readRegister(uint8_t address, uint16_t &value) override;
        bool writeRegister(uint8_t address, uint16_t value) override;
        bool readRegister(Register address, uint16_t &value)
        {
            return readRegister(static_cast<uint8_t>(address), value);
        }
        bool writeRegister(Register address, uint16_t value)
        {
            return writeRegister(static_cast<uint8_t>(address), value);
        }
        bool _waitForDNRClear();
    };
}
