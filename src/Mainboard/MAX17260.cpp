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

#include <math.h>

#include <esp_log.h>

#include "MAX17260.h"

namespace PowerFeather
{
    static const char *TAG = "PowerFeather::Mainboard::MAX17260";

    bool MAX17260::getEnabled(bool &enabled)
    {
        return false;
    }

    bool MAX17260::getCellVoltage(uint16_t &voltage)
    {
        return false;
    }

    bool MAX17260::getRSOC(uint8_t &percent)
    {
        return false;
    }

    bool MAX17260::getTimeToEmpty(uint16_t &minutes)
    {
        return false;
    }

    bool MAX17260::getTimeToFull(uint16_t &minutes)
    {
        return false;
    }

    bool MAX17260::getCellTemperature(float &temperature)
    {
        return false;
    }

    bool MAX17260::getCycles(uint16_t &cycles)
    {
        return false;
    }

    bool MAX17260::getSOH(uint8_t &percent)
    {
        return false;
    }

    bool MAX17260::getInitialized(bool& state)
    {
        return false;
    }

    bool MAX17260::setEnabled(bool enable)
    {
        return false;
    }

    bool MAX17260::setCellTemperature(float temperature)
    {
        return false;
    }

    bool MAX17260::enableTSENSE(bool enableTsense1, bool enableTsense2)
    {
        return false;
    }

    bool MAX17260::setLowVoltageAlarm(uint16_t voltage)
    {
        return false;
    }

    bool MAX17260::setHighVoltageAlarm(uint16_t voltage)
    {
        return false;
    }

    bool MAX17260::setLowRSOCAlarm(uint8_t percent)
    {
        return false;
    }

    bool MAX17260::setTerminationFactor(float factor)
    {
        return false;
    }

    bool MAX17260::setInitialized()
    {
        return false;
    }

    bool MAX17260::clearLowVoltageAlarm()
    {
        return false;
    }

    bool MAX17260::clearHighVoltageAlarm()
    {
        return false;
    }

    bool MAX17260::clearLowRSOCAlarm()
    {
        return false;
    }
}