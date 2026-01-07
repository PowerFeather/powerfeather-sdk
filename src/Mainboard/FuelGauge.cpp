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

#include <esp_log.h>

#include "FuelGauge.h"

namespace PowerFeather
{
    // Comment: Is not not a fully abstract class?
    static const char *TAG = "PowerFeather::Mainboard::FuelGauge";

    bool FuelGauge::getEnabled(bool &enabled)
    {
        (void)enabled;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getCellVoltage(uint16_t &voltage)
    {
        (void)voltage;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getRSOC(uint8_t &percent)
    {
        (void)percent;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getTimeToEmpty(uint16_t &minutes)
    {
        (void)minutes;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getTimeToFull(uint16_t &minutes)
    {
        (void)minutes;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getCellTemperature(float &temperature)
    {
        (void)temperature;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getCycles(uint16_t &cycles)
    {
        (void)cycles;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getSOH(uint8_t &percent)
    {
        (void)percent;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::getInitialized(bool &state)
    {
        (void)state;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setEnabled(bool enable)
    {
        (void)enable;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setCellTemperature(float temperature)
    {
        (void)temperature;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::enableTSENSE(bool enableTsense1, bool enableTsense2)
    {
        (void)enableTsense1;
        (void)enableTsense2;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setLowVoltageAlarm(uint16_t voltage)
    {
        (void)voltage;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setHighVoltageAlarm(uint16_t voltage)
    {
        (void)voltage;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setLowRSOCAlarm(uint8_t percent)
    {
        (void)percent;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setTerminationFactor(float factor)
    {
        (void)factor;
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::setInitialized()
    {
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::clearLowVoltageAlarm()
    {
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::clearHighVoltageAlarm()
    {
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }

    bool FuelGauge::clearLowRSOCAlarm()
    {
        ESP_LOGE(TAG, "%s not implemented.", __func__);
        return false;
    }
}
