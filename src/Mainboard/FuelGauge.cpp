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

#include "FuelGauge.h"

namespace PowerFeather
{
    class FuelGauge
    {

        LC709204F getFuelGauge()
        {
            return _fuelGauge;
        }

        bool FuelGauge::isEnabled()
        {
            bool enabled = false;
            getFuelGauge().getOperationMode(enabled); // failure here means false is returned
            return enabled;
        }

        Result FuelGauge::_init()
        {
            bool inited = false;
            RET_IF_FALSE(getFuelGauge().getInitialized(inited), Result::Failure); // check if already initialized

            if (!inited)
            {
                LC709204F::ChangeOfParameter param = _batteryType == BatteryType::ICR18650_26H ? LC709204F::ChangeOfParameter::ICR18650_26H :
                                                    _batteryType == BatteryType::UR18650ZY ? LC709204F::ChangeOfParameter::UR18650ZY :
                                                    LC709204F::ChangeOfParameter::Nominal_3V7_Charging_4V2;
                RET_IF_FALSE(getFuelGauge().setAPA(_batteryCapacity, param), Result::Failure);
                RET_IF_FALSE(getFuelGauge().setChangeOfParameter(param), Result::Failure);

                const float minFactor = LC709204F::MinTerminationFactor;
                const float maxFactor = LC709204F::MaxTerminationFactor;
                float terminationFactor = _terminationCurrent/static_cast<float>(_batteryCapacity);
                terminationFactor = std::min(std::max(terminationFactor, minFactor), maxFactor);
                RET_IF_FALSE(getFuelGauge().setTerminationFactor(terminationFactor), Result::Failure);

                RET_IF_FALSE(getFuelGauge().enableTSENSE(false, false), Result::Failure);
                RET_IF_FALSE(getFuelGauge().setOperationMode(true), Result::Failure);
                RET_IF_FALSE(getFuelGauge().setInitialized(), Result::Failure);
                ESP_LOGD(TAG, "Fuel gauge initialized.");
            }
            else
            {
                ESP_LOGD(TAG, "Fuel gauge already initialized.");
            }

            return Result::Ok;
        }
    }
}