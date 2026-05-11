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

#include <esp_log.h>

#include "MasterI2C.h"

#ifndef ARDUINO

namespace PowerFeather
{
    static const char *TAG = "PowerFeather::Utils::MasterI2C";

    bool MasterI2C::start()
    {
        if (_bus)
        {
            return true;
        }

        i2c_master_bus_config_t conf = {};
        conf.i2c_port = _port;
        conf.sda_io_num = static_cast<gpio_num_t>(_sdaPin);
        conf.scl_io_num = static_cast<gpio_num_t>(_sclPin);
        conf.clk_source = I2C_CLK_SRC_DEFAULT;
        conf.glitch_ignore_cnt = 7;
        conf.flags.enable_internal_pullup = false;

        esp_err_t result = i2c_new_master_bus(&conf, &_bus);
        ESP_LOGD(TAG, "Start with port: %d, sda: %d, scl: %d, freq: %d, result: %s.",
                 static_cast<int>(_port), _sdaPin, _sclPin, static_cast<int>(_freq), esp_err_to_name(result));
        return result == ESP_OK;
    }

    bool MasterI2C::_getDevice(uint8_t address, i2c_master_dev_handle_t &device)
    {
        if (!_bus || address >= _firstInvalid7BitAddress)
        {
            return false;
        }

        Device *availableDevice = nullptr;
        for (Device &registeredDevice : _devices)
        {
            if (!registeredDevice.handle)
            {
                if (!availableDevice)
                {
                    availableDevice = &registeredDevice;
                }
                continue;
            }

            if (registeredDevice.address == address)
            {
                device = registeredDevice.handle;
                return true;
            }
        }

        if (!availableDevice)
        {
            ESP_LOGD(TAG, "No free device slot for address: %02x.", address);
            return false;
        }

        i2c_device_config_t conf = {};
        conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        conf.device_address = address;
        conf.scl_speed_hz = _freq;

        esp_err_t result = i2c_master_bus_add_device(_bus, &conf, &availableDevice->handle);
        ESP_LOGD(TAG, "Add device address: %02x, result: %s.", address, esp_err_to_name(result));
        if (result != ESP_OK)
        {
            availableDevice->handle = nullptr;
            return false;
        }

        availableDevice->address = address;
        device = availableDevice->handle;
        return true;
    }

    bool MasterI2C::write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len)
    {
        i2c_master_dev_handle_t device = nullptr;
        if (!_getDevice(address, device))
        {
            return false;
        }

        uint8_t buf2[len + sizeof(reg)];
        memcpy(buf2, &reg, sizeof(reg));
        memcpy(&(buf2[sizeof(reg)]), buf, len);
        ESP_LOGV(TAG, "Write address: %02x, reg: %02x, buf: %p, len: %d.", address, reg, buf, len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);
        return i2c_master_transmit(device, buf2, sizeof(buf2), TransferTimeoutMs) == ESP_OK;
    }

    bool MasterI2C::read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len)
    {
        i2c_master_dev_handle_t device = nullptr;
        if (!_getDevice(address, device))
        {
            return false;
        }

        ESP_LOGV(TAG, "Read address: %02x, reg: %02x, buf: %p, len: %d.", address, reg, buf, len);
        esp_err_t res = i2c_master_transmit_receive(device, &reg, sizeof(reg), buf, len, TransferTimeoutMs);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);
        return res == ESP_OK;
    }

    bool MasterI2C::end()
    {
        if (!_bus)
        {
            return true;
        }

        bool success = true;
        for (Device &registeredDevice : _devices)
        {
            if (registeredDevice.handle)
            {
                esp_err_t result = i2c_master_bus_rm_device(registeredDevice.handle);
                if (result != ESP_OK)
                {
                    ESP_LOGD(TAG, "Remove device address: %02x failed: %s.",
                             registeredDevice.address, esp_err_to_name(result));
                    success = false;
                }
                registeredDevice.handle = nullptr;
            }
        }

        esp_err_t result = i2c_del_master_bus(_bus);
        ESP_LOGD(TAG, "End, result: %s.", esp_err_to_name(result));
        _bus = nullptr;
        return success && result == ESP_OK;
    }
}

#endif
