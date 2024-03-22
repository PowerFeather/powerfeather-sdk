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

#include "Utils/MasterI2C.h"

namespace PowerFeather
{
    class BQ2562x
    {
    public:

        static constexpr uint16_t MaxChargingCurrent = 2000;
        static constexpr uint16_t MaxVINDPMVoltage = 16800;

        struct Register
        {
            uint8_t address;
            uint8_t size;
            uint8_t start;
            uint8_t end;
        };

        enum class VBUSStat
        {
            None,
            Adapter
        };

        enum class ChargeStat
        {
            Terminated,
            Trickle,
            Taper,
            TopOff
        };

        enum class ADCRate
        {
            Oneshot,
            Continuous,
        };

        enum class ADCSampling
        {
            Bits_12,
            Bits_11,
            Bits_10,
            Bits_9,
        };

        enum class ADCAverage
        {
            Single,
            Running,
        };

        enum class ADCAverageInit
        {
            Existing,
            New,
        };

        enum class BATFETControl
        {
            Normal,
            ShutdownMode,
            ShipMode,
            SystemPowerReset
        };

        enum class BATFETDelay
        {
            Delay20ms,
            Delay10s,
        };

        enum class Interrupt : uint8_t
        {
            VBUS
        };

        enum class Adc : uint8_t
        {
            IBUS = 7,
            IBAT = 6,
            VBUS = 5,
            VBAT = 4,
            VSYS = 3,
            TS = 2,
            TDIE = 1,
            VPMID = 0
        };

        BQ2562x(MasterI2C &i2c) : _i2c(i2c) {}

        bool getWD(bool &enabled);
        bool getVBUS(uint16_t &voltage);
        bool getIBUS(int16_t &current);
        bool getVBAT(uint16_t &voltage);
        bool getIBAT(int16_t &current);
        bool getADCDone(bool &done);
        bool getTS(bool &enabled);
        bool getTS_ADC(float &celsius);
        bool getVBUSStat(VBUSStat &stat);
        bool getChargeStat(ChargeStat &stat);
        bool getPartInformation(uint8_t &info);

        bool enableWD(bool enable);
        bool enableCharging(bool enable);
        bool enableTS(bool enable);
        bool enableHIZ(bool enable);
        bool enableInterrupts(bool enable);
        bool enableInterrupt(Interrupt mask, bool en);
        bool enableWVBUS(bool enable);
        bool enableADC(Adc adc, bool enable);
        bool setChargeCurrent(uint16_t current);
        bool setBATFETControl(BATFETControl control);
        bool setBATFETDelay(BATFETDelay delay);
        bool setVINDPM(uint32_t voltage);
        bool setIINDPM(uint32_t current);
        bool setupADC(bool enable, ADCRate rate = ADCRate::Continuous, ADCSampling sampling = ADCSampling::Bits_9,
                      ADCAverage average = ADCAverage::Single, ADCAverageInit averageInit = ADCAverageInit::Existing);

    private:
        const Register Charge_Current_Limit_ICHG =            { 0x02, 2, 5, 11 };

        const Register Input_Current_Limit_IINDPM =           { 0x06, 2, 4, 11 };
        const Register Input_Current_Limit_VINDPM =           { 0x08, 2, 5, 13 };

        const Register Charger_Control_0_EN_CHG =             { 0x16, 1, 5, 5 };
        const Register Charger_Control_0_WATCHDOG =           { 0x16, 1, 0, 1 };
        const Register Charger_Control_0_EN_HIZ =             { 0x16, 1, 4, 4 };

        const Register Charger_Control_2 =                    { 0x18, 1, 0, 7 };
        const Register Charger_Control_2_BATFET_CTRL =        { 0x18, 1, 0, 1 };
        const Register Charger_Control_2_BATFET_DLY =         { 0x18, 1, 2, 2 };
        const Register Charger_Control_2_WVBUS =              { 0x18, 1, 3, 3 };

        const Register NTC_Control_0_TS_IGNORE =              { 0x1a, 1, 7, 7 };

        const Register Charger_Status_0 =                     { 0x1d, 1, 0, 7 };
        const Register Charger_Status_1 =                     { 0x1e, 1, 0, 7 };
        const Register Charger_Status_1_VBUS_STAT =           { 0x1e, 1, 0, 2 };
        const Register Charger_Status_1_CHG_STAT =            { 0x1e, 1, 3, 4 };
        const Register FAULT_Status_0 =                       { 0x1f, 1, 0, 7 };
        const Register Charger_Flag_0 =                       { 0x20, 1, 0, 7 };
        const Register Charger_Flag_1 =                       { 0x21, 1, 0, 7 };
        const Register FAULT_Flag_0 =                         { 0x22, 1, 0, 7 };
        const Register Charger_Mask_0 =                       { 0x23, 1, 0, 7 };
        const Register Charger_Mask_1 =                       { 0x24, 1, 0, 7 };
        const Register FAULT_Mask_0 =                         { 0x25, 1, 2, 2 };

        const Register ADC_Control =                          { 0x26, 1, 0, 7 };

        const Register ADC_Function_Disable_0 =               { 0x27, 1, 0, 7 };

        const Register IBUS_ADC =                             { 0x28, 2, 1, 15 };
        const Register IBAT_ADC =                             { 0x2a, 2, 2, 15 };
        const Register VBUS_ADC =                             { 0x2c, 2, 2, 14 };
        const Register VBAT_ADC =                             { 0x30, 2, 1, 12 };
        const Register TS_ADC =                               { 0x34, 2, 0, 11 };

        const Register Part_Information =                     { 0x38, 1, 0, 7 };
        const Register Part_Information_PN =                  { 0x38, 1, 3, 5 };
        const Register Part_Information_DEV_REV =             { 0x38, 1, 0, 2 };

        static constexpr uint8_t _i2cAddress = 0x6a;

        MasterI2C &_i2c;

        template <typename T>
        bool _readReg(Register reg, T &value);
        template <typename T>
        bool _writeReg(Register reg, T value);
    };
}