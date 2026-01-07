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

#include "FuelGauge.h"

namespace PowerFeather
{
    class LC709204F : public RegisterFuelGauge
    {
    public:

        static constexpr uint16_t MinVoltageAlarm = 2500;
        static constexpr uint16_t MaxVoltageAlarm = 5000;
        static constexpr uint16_t MaxBatteryCapacity = 6000;
        static constexpr uint16_t MinBatteryCapacity = 50;
        static constexpr float MaxTerminationFactor = 0.3f;
        static constexpr float MinTerminationFactor = 0.02f;

        static constexpr float MinTemperature = -30.0f;
        static constexpr float MaxTemperature = 80.0f;

        static constexpr uint16_t MinTemperatureRaw = 0x0980;
        static constexpr uint16_t MaxTemperatureRaw = 0x0DCC;

        enum class Register : uint8_t
        {
            TimeToEmpty = 0x03,
            TimeToFull = 0x05,
            TSENSE1 = 0x08,
            Cell_Voltage = 0x09,
            APA = 0x0b,
            RSOC = 0x0d,
            Change_Of_The_Parameter = 0x12,
            Alarm_Low_RSOC = 0x13,
            Alarm_Low_Cell_Voltage = 0x14,
            IC_Power_Mode = 0x15,
            Status_Bit = 0x16,
            Cycle_Count = 0x17,
            BatteryStatus = 0x19,
            Termination_Current_Rate = 0x1c,
            Alarm_High_Cell_Voltage = 0x1f,
            State_Of_Health = 0x32
        };

        enum class ChangeOfParameter : uint8_t
        {
            Nominal_3V7_Charging_4V2 = 0x00,
            UR18650ZY = 0x01,
            ICR18650_26H = 0x02,
            Nominal_3V8_Charging_4V35 = 0x03,
            Nominal_3V85_Charging_4V4 = 0x04,
        };

        static constexpr uint8_t RegisterSize = 2;

        LC709204F(MasterI2C &i2c) : RegisterFuelGauge(i2c, RegisterSize) {}

        bool probe() override;
        const char *name() const override { return "LC709204F"; }
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
        float minTerminationFactor() const override { return MinTerminationFactor; }
        float maxTerminationFactor() const override { return MaxTerminationFactor; }

        bool getEnabled(bool &enabled) override;
        bool getCellVoltage(uint16_t &voltage) override;
        bool getRSOC(uint8_t &percent) override;
        bool getTimeToEmpty(uint16_t &minutes) override;
        bool getTimeToFull(uint16_t &minutes) override;
        bool getCellTemperature(float &temperature) override;
        bool getCycles(uint16_t &cycles) override;
        bool getSOH(uint8_t &percent) override;
        bool getInitialized(bool& state);
        bool setEnabled(bool enable) override;
        bool setAPA(uint16_t capacity, ChangeOfParameter changeOfParam);
        bool setChangeOfParameter(ChangeOfParameter changeOfParam);
        bool setCellTemperature(float temperature) override;
        bool enableTSENSE(bool enableTsense1, bool enableTsense2) override;
        bool setLowVoltageAlarm(uint16_t voltage) override;
        bool setHighVoltageAlarm(uint16_t voltage) override;
        bool setLowRSOCAlarm(uint8_t percent) override;
        bool setTerminationFactor(float factor);
        bool setInitialized();
        bool clearLowVoltageAlarm() override;
        bool clearHighVoltageAlarm() override;
        bool clearLowRSOCAlarm() override;

    private:
        enum class BatteryStatus : uint8_t
        {
            LowRSOC = 9,
            LowCellVoltage = 11,
            HighCellVoltage = 15,
            Initialized = 7
        };

        enum class OperationMode : uint16_t
        {
            OperationalMode = 0x0001,
            SleepMode = 0x0002
        };

        const std::tuple<uint16_t, uint8_t> _apaTable[10] =
        {
            {50, 0x13},
            {100, 0x15},
            {200, 0x18},
            {500, 0x21},
            {1000, 0x2D},
            {2000, 0x3A},
            {3000, 0x3F},
            {4000, 0x42},
            {5000, 0x44},
            {6000, 0x45},
        };

        static constexpr uint8_t _i2cAddress = 0x0b;

        using Field = RegisterFuelGauge::RegisterField;

        struct Fields
        {
            struct BatteryStatus
            {
                static constexpr Field Initialized = {
                    static_cast<uint8_t>(Register::BatteryStatus),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::Initialized),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::Initialized)
                };
                static constexpr Field LowVoltage = {
                    static_cast<uint8_t>(Register::BatteryStatus),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::LowCellVoltage),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::LowCellVoltage)
                };
                static constexpr Field HighVoltage = {
                    static_cast<uint8_t>(Register::BatteryStatus),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::HighCellVoltage),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::HighCellVoltage)
                };
                static constexpr Field LowRSOC = {
                    static_cast<uint8_t>(Register::BatteryStatus),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::LowRSOC),
                    static_cast<uint8_t>(LC709204F::BatteryStatus::LowRSOC)
                };
            };
        };

        bool readRegister(uint8_t address, uint16_t &data) override;
        bool writeRegister(uint8_t address, uint16_t data) override;

        uint8_t _computeCRC8(uint8_t *data, int len);

        bool _setVoltageAlarm(Register reg, uint16_t voltage);
        bool _clearAlarm(const Field &alarmField);
    };
}
