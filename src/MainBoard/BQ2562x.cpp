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

#include "BQ2562x.h"

namespace PowerFeather
{
    template <typename T>
    bool BQ2562x::writeReg(Register reg, T value)
    {
        uint8_t last = (reg.size * CHAR_BIT) - 1;

        assert(reg.size <= sizeof(value));
        assert(reg.start <= reg.end);
        assert(reg.end <= last);

        uint16_t data = 0;

        if (readReg(Register{reg.address, reg.size, 0, last}, data))
        {
            uint8_t bits = reg.end - reg.start + 1;
            uint16_t mask = ((1 << bits) - 1) << reg.start;
            data = (data & ~mask) | ((value << reg.start) & mask);
            return _i2c.write(_i2cAddress, reg.address, reinterpret_cast<uint8_t *>(&data), reg.size);
        }

        return false;
    }

    template <typename T>
    bool BQ2562x::readReg(Register reg, T &value)
    {
        assert(reg.size <= sizeof(value));
        assert(reg.start <= reg.end);
        assert(reg.end <= (reg.size * CHAR_BIT) - 1);

        uint16_t data = 0;
        if (_i2c.read(_i2cAddress, reg.address, reinterpret_cast<uint8_t *>(&data), reg.size))
        {
            int left = (((sizeof(data) * CHAR_BIT) - 1) - reg.end);
            data <<= left;
            data >>= left + reg.start;
            value = data;
            return true;
        }
        return false;
    }

    bool BQ2562x::setChargeCurrent(uint16_t current)
    {
        current /= 40;
        if (current >= 0x1 && current <= 0x32)
        {
            return writeReg(Charge_Current_Limit_ICHG, current);
        }
        return false;
    }

    bool BQ2562x::enableWD(bool enable)
    {
        return writeReg(Charger_Control_0_WATCHDOG, enable);
    }

    bool BQ2562x::getWD(bool &enabled)
    {
        return readReg(Charger_Control_0_WATCHDOG, enabled);
    }

    bool BQ2562x::getTS(bool &enabled)
    {
        bool tsIgnore = false, tsDis = false;
        if (readReg(NTC_Control_0_TS_IGNORE, tsIgnore))
        {
            Register reg = ADC_Function_Disable_0;
            reg.start = reg.end = static_cast<uint8_t>(Adc::TS);
            if (readReg(reg, tsDis))
            {
                enabled = (!tsIgnore) && (!tsDis);
                return true;
            }
        }
        return false;
    }

    bool BQ2562x::enableHIZ(bool enable)
    {
        return writeReg(Charger_Control_0_EN_HIZ, enable);
    }

    bool BQ2562x::enableTS(bool enable)
    {
        return writeReg(NTC_Control_0_TS_IGNORE, !enable) && enableADC(Adc::TS, enable);
    }

    bool BQ2562x::enableADC(Adc adc, bool enable)
    {
        Register reg = ADC_Function_Disable_0;
        reg.start = reg.end = static_cast<uint8_t>(adc);
        return writeReg(reg, !enable);
    }

    bool BQ2562x::getPartInformation(uint8_t &value)
    {
        return readReg(Part_Information, value);
    }

    bool BQ2562x::enableInterrupts(bool enable)
    {
        uint8_t data = enable ? 0x00 : 0xFF;
        if (writeReg(Charger_Mask_0, data))
        {
            return writeReg(Charger_Mask_1, data);
        }
        return false;
    }

    bool BQ2562x::enableInterrupt(Interrupt num, bool en)
    {
        if (num == Interrupt::VBUS) // only uses Charger_Mask_1 for now
        {
            Register reg = Charger_Mask_1;
            reg.start = reg.end = static_cast<uint8_t>(num);
            return writeReg(reg, !en);
        }
        return false;
    }

    bool BQ2562x::enableCharging(bool state)
    {
        return writeReg(Charger_Control_0_EN_CHG, state);
    }

    bool BQ2562x::getVBUS(uint16_t &value)
    {
        uint16_t data = 0;
        if (readReg(VBUS_ADC, data))
        {
            value = data * 3.97f;
            return true;
        }

        return false;
    }

    bool BQ2562x::getIBAT(int16_t &value)
    {
        uint16_t data = 0;
        if (readReg(IBAT_ADC, data))
        {
            if (data != 0x2000)
            {
                if (data >= 0x38AD && data <= 0x3FFF)
                {
                    data = ((0x3FFF - data + 1) * 4) * -1;
                }
                else
                {
                    data *= 4;
                }

                value = data;
                return true;
            }
        }
        return false;
    }

    bool BQ2562x::getIBUS(int16_t &value)
    {
        uint16_t data = 0;
        if (readReg(IBUS_ADC, data))
        {
            if (data >= 0x7830 && data <= 0x7FFF)
            {
                data = (0x7FFF - data + 1) * -2;
            }
            else
            {
                data *= 2;
            }

            value = data;
            return true;
        }
        return false;
    }

    bool BQ2562x::getVBAT(uint16_t &value)
    {
        uint16_t res = 0;
        if (readReg(VBAT_ADC, res))
        {
            value = (res * 1.99f);
            return true;
        }
        return false;
    }

    bool BQ2562x::setVINDPM(uint32_t mV)
    {
        return writeReg(Input_Current_Limit_VINDPM, static_cast<uint16_t>((mV) / 40));
    }

    bool BQ2562x::setIINDPM(uint32_t mA)
    {
        return writeReg(Input_Current_Limit_IINDPM, static_cast<uint16_t>((mA) / 40));
    }

    bool BQ2562x::setupADC(bool enable, ADCRate rate, ADCSampling sampling, ADCAverage average, ADCAverageInit averageInit)
    {
        uint8_t value = enable << 7;

        if (enable)
        {
            value |= (rate == ADCRate::Oneshot) << 6;

            switch (sampling)
            {
            case ADCSampling::Bits_12:
                value |= 0 << 4;
                break;

            case ADCSampling::Bits_11:
                value |= 1 << 4;
                break;

            case ADCSampling::Bits_10:
                value |= 2 << 4;
                break;

            case ADCSampling::Bits_9:
                value |= 3 << 4;
                break;

            default:
                break;
            }

            value |= (average == ADCAverage::Running) << 3;
            value |= (averageInit == ADCAverageInit::New) << 2;
        }

        return writeReg(ADC_Control, value);
    }

    bool BQ2562x::getADCDone(bool &done)
    {
        uint8_t data;
        if (readReg(Charger_Status_0, data))
        {
            done = data & (1 << 6);
            return true;
        }
        return false;
    }

    bool BQ2562x::getBatteryVoltage(float &value)
    {
        uint16_t data = 0;
        if (readReg(VBAT_ADC, data))
        {
            value = data / 1000.0f;
            return true;
        }
        return false;
    }

    bool BQ2562x::getVBUSStat(VBUSStat &stat)
    {
        uint8_t data = 0;
        if (readReg(Charger_Status_1_VBUS_STAT, data))
        {
            if (data == 0b100)
            {
                stat = VBUSStat::Adapter;
            }
            else
            {
                stat = VBUSStat::None;
            }
            return true;
        }
        return false;
    }

    bool BQ2562x::getChargeStat(ChargeStat &stat)
    {
        uint8_t value = 0;
        if (readReg(Charger_Status_1_CHG_STAT, value))
        {
            ChargeStat res = ChargeStat::Terminated;

            switch (value)
            {
            case 0x01:
                res = ChargeStat::Trickle;
                break;

            case 0x02:
                res = ChargeStat::Taper;
                break;

            case 0x03:
                res = ChargeStat::TopOff;
                break;

            case 0x00:
            default:

                break;
            }

            stat = res;
            return true;
        }

        return false;
    }

    bool BQ2562x::setBATFETControl(BATFETControl control)
    {
        uint8_t value = 0x0;
        switch (control)
        {
        case BATFETControl::ShutdownMode:
            value = 0x01;
            break;
        case BATFETControl::ShipMode:
            value = 0x02;
            break;
        case BATFETControl::SystemPowerReset:
            value = 0x03;
            break;
        case BATFETControl::Normal:
        default:
            break;
        }

        return writeReg(Charger_Control_2_BATFET_CTRL, value);
    }

    bool BQ2562x::setBATFETDelay(BATFETDelay delay)
    {
        bool value = false;
        switch (delay)
        {
        case BATFETDelay::Delay10s:
            value = true;
            break;
        case BATFETDelay::Delay20ms:
        default:
            break;
        }

        return writeReg(Charger_Control_2_BATFET_DLY, value);
    }

    bool BQ2562x::enableWVBUS(bool enable)
    {
        return writeReg(Charger_Control_2_WVBUS, enable);
    }

    bool BQ2562x::getTS_ADC(float &value)
    {
        uint16_t res = 0;
        if (readReg(TS_ADC, res))
        {
            value = (res * 0.0961f) / 100.0f;
            return true;
        }
        return false;
    }
}
