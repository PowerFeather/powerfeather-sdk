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

#include <cassert>
#include <climits>
#include <cstdint>

#include "Utils/Result.h"
#include "Utils/MasterI2C.h"

namespace PowerFeather
{
    class FuelGauge
    {
    protected:
        MasterI2C &_i2c;

    public:
        FuelGauge(MasterI2C &i2c) : _i2c(i2c) {}

        virtual bool getEnabled(bool &enabled) = 0;
        virtual bool getCellVoltage(uint16_t &voltage) = 0;
        virtual bool getRSOC(uint8_t &percent) = 0;
        virtual bool getTimeToEmpty(uint16_t &minutes) = 0;
        virtual bool getTimeToFull(uint16_t &minutes) = 0;
        virtual bool getCellTemperature(float &temperature) = 0;
        virtual bool getCycles(uint16_t &cycles) = 0;
        virtual bool getSOH(uint8_t &percent) = 0;
        virtual bool getInitialized(bool& state) = 0;
        virtual bool setEnabled(bool enable) = 0;
        virtual bool setCellTemperature(float temperature) = 0;
        virtual bool enableTSENSE(bool enableTsense1, bool enableTsense2) = 0;
        virtual bool setLowVoltageAlarm(uint16_t voltage) = 0;
        virtual bool setHighVoltageAlarm(uint16_t voltage) = 0;
        virtual bool setLowRSOCAlarm(uint8_t percent) = 0;
        virtual bool setTerminationFactor(float factor) = 0;
        virtual bool setInitialized() = 0;
        virtual bool clearLowVoltageAlarm() = 0;
        virtual bool clearHighVoltageAlarm() = 0;
        virtual bool clearLowRSOCAlarm() = 0;

        virtual bool probe() = 0;
        virtual const char *getName() const = 0;
        virtual void getVoltageAlarmRange(uint16_t &minMv, uint16_t &maxMv) const = 0;
        virtual void getTemperatureRange(float &minC, float &maxC) const = 0;
        virtual void getTerminationFactorRange(float &minFactor, float &maxFactor) const = 0;
    };
}

namespace PowerFeather
{
    class RegisterFuelGauge : public FuelGauge
    {
    protected:
        struct RegisterField
        {
            uint8_t address;
            uint8_t start;
            uint8_t end;
        };

        RegisterFuelGauge(MasterI2C &i2c, uint8_t registerSizeBytes)
            : FuelGauge(i2c), _registerSizeBytes(registerSizeBytes)
        {
            assert(_registerSizeBytes > 0);
        }

        bool readField(const RegisterField &field, uint16_t &value);
        bool writeField(const RegisterField &field, uint16_t value);

        uint8_t registerSizeBytes() const { return _registerSizeBytes; }
        uint8_t registerBitWidth() const { return static_cast<uint8_t>(_registerSizeBytes * CHAR_BIT); }

        virtual bool readRegister(uint8_t address, uint16_t &value) = 0;
        virtual bool writeRegister(uint8_t address, uint16_t value) = 0;

    private:
        static constexpr uint16_t _maskForWidth(uint8_t width)
        {
            return (width >= 16) ? 0xFFFFu : static_cast<uint16_t>((1u << width) - 1u);
        }

        uint8_t _registerSizeBytes;
    };

    inline bool RegisterFuelGauge::readField(const RegisterField &field, uint16_t &value)
    {
        uint8_t lastBit = static_cast<uint8_t>(registerBitWidth() - 1);
        assert(field.start <= field.end);
        assert(field.end <= lastBit);

        uint16_t data = 0;
        if (!readRegister(field.address, data))
        {
            return false;
        }

        uint8_t width = static_cast<uint8_t>(field.end - field.start + 1);
        uint16_t mask = _maskForWidth(width);
        value = static_cast<uint16_t>((data >> field.start) & mask);
        return true;
    }

    inline bool RegisterFuelGauge::writeField(const RegisterField &field, uint16_t value)
    {
        uint8_t lastBit = static_cast<uint8_t>(registerBitWidth() - 1);
        assert(field.start <= field.end);
        assert(field.end <= lastBit);

        uint16_t data = 0;
        if (!readRegister(field.address, data))
        {
            return false;
        }

        uint8_t width = static_cast<uint8_t>(field.end - field.start + 1);
        uint16_t valueMask = _maskForWidth(width);
        uint16_t maskedValue = static_cast<uint16_t>(value & valueMask);
        uint16_t fieldMask = static_cast<uint16_t>(valueMask << field.start);
        data = static_cast<uint16_t>((data & ~fieldMask) | ((maskedValue << field.start) & fieldMask));
        return writeRegister(field.address, data);
    }
}
