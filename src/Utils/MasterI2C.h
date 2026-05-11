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

#include <cstddef>
#include <cstdint>
#include <cstring>

#ifndef ARDUINO
#include <driver/i2c_master.h>
#endif

namespace PowerFeather
{
    class MasterI2C
    {
    public:
#ifndef ARDUINO
        MasterI2C(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq) :
                _port(static_cast<i2c_port_num_t>(port)), _sdaPin(sdaPin), _sclPin(sclPin), _freq(freq) {};
#else
        MasterI2C(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq) :
                _port(port), _sdaPin(sdaPin), _sclPin(sclPin), _freq(freq) {};
#endif

        virtual bool start();
        virtual bool end();
        virtual bool write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len);
        virtual bool read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len);

        static constexpr int TransferTimeoutMs = 50;

    protected:
#ifndef ARDUINO
        i2c_port_num_t _port;
#else
        uint8_t _port;
#endif
        uint8_t _sdaPin;
        uint8_t _sclPin;
        uint32_t _freq;

#ifndef ARDUINO
    private:
        struct Device
        {
            uint8_t address{0};
            i2c_master_dev_handle_t handle{nullptr};
        };

        static constexpr uint8_t _firstInvalid7BitAddress = 0x80;
        static constexpr size_t _maxDevices = 8;

        bool _getDevice(uint8_t address, i2c_master_dev_handle_t &device);

        i2c_master_bus_handle_t _bus{nullptr};
        Device _devices[_maxDevices]{};
#endif
    };
}
