#include <BQ2562x.h>

namespace PowerFeather
{
    // Initialize the board
    // batteryCapacity - advertised capacity of the battery
    // chargeRate - charge rate of the battery expressed as fraction of the capacity,
    //              ex. 2000mAh, 0.5 charge rate = 1000mA charge current
    //
    // event handler - when significant events occur
    //      - change in external power source
    //      - enable pin pulled down low
    //      - charging started/stopped
    //      - temperature too high or too low (hardware set)
    //      - battery state of charge low
    //      - input current exceeded
    //      - charge current exceeded
    //
    //  charger status
    //      - charging changed
    //      - vbus changed
    //  charger fault
    //      - bat overvoltage or overcurrent
    //      - sys overvoltage or short
    //      - tshut thermal shutdown
    //
    // take OCV for gauge, then enable charging
    // enable 3V3 by default
    // enable 5V by default

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
            uint16_t mask = ((0b1 << bits) - 1) << reg.start;
            data = (data & ~mask) | ((value << reg.start) & mask);
            return _i2c.write(_i2cAddress, reg.address, reinterpret_cast<uint8_t*>(&data), reg.size);
        }

        return false;
    }

    template <typename T>
    bool BQ2562x::readReg(Register reg, T& value)
    {
        assert(reg.size <= sizeof(value));
        assert(reg.start <= reg.end);
        assert(reg.end <= (reg.size * CHAR_BIT) - 1);

        uint16_t data = 0;
        if (_i2c.read(_i2cAddress, reg.address, reinterpret_cast<uint8_t*>(&data), reg.size))
        {
            int left = (((sizeof(data) * CHAR_BIT) - 1) - reg.end);
            data <<= left;
            data >>= left + reg.start;
            value = data;
            return true;
        }
        return false;
    }

    template <typename T>
    bool BQ2562x::writeReg(T address, uint8_t start, uint8_t end, T value)
    {
        // T data = 0;

        // if (readReg(address, data))
        // {
        // 	uint8_t bits = end - start + 1;
        // 	T mask = ((0b1 << bits) - 1) << start;
        // 	value <<= start;
        // 	data = (data & ~mask) | (mask & value);

        // 	return _i2c.write(_i2cAddress,  reinterpret_cast<uint8_t*>(&data), sizeof(data));
        // }

        return false;
    }

    template <typename T>
    bool BQ2562x::writeReg(T address, uint8_t bit, bool value)
    {
        return writeReg(address, bit, bit, static_cast<T>(value));
    }

    template <typename T>
    bool BQ2562x::writeReg(T address, T value)
    {
        return writeReg(address, 0, ((sizeof(value) * CHAR_BIT) - 1), value);
    }

    template <typename T>
    bool BQ2562x::readReg(T address, uint8_t start, uint8_t end, T& value)
    {
        value = 0;
        // static_assert(sizeof(value) == sizeof(uint8_t) || sizeof(value) == sizeof(uint16_t));
        // assert(end < (sizeof(value) * CHAR_BIT));
        // assert(start <= end);
        // assert((end - start) < (sizeof(T) * CHAR_BIT));

        // if (!_i2c.read(_i2cAddress, nt8_t*>(&address), 1, reinterpret_cast<uint8_t*>(&value), sizeof(value)))
        // {
        //     return false;
        // }

        // int left = (((sizeof(value) * CHAR_BIT) - 1) - end);
        // value <<= left;
        // value >>= left + start;
        return true;
    }

    template <typename T>
    bool BQ2562x::readReg(T address, uint8_t bit, bool &value)
    {
        T data = 0;
        bool res = readReg(address, bit, bit, data);
        value = data;
        return res;
    }

    template <typename T>
    bool BQ2562x::readReg(T address, T& value)
    {
        return readReg(static_cast<T>(address), 0, (sizeof(value) * CHAR_BIT) - 1, value);
    }

    bool BQ2562x::setChargeCurrent(uint16_t current)
    {
        current /= 40;
        if (current >= 0x1 && current <= 0x32)
        {
            return writeReg(Registers::Charge_Current_Limit_ICHG, current);
        }
        return false;
    }

    bool BQ2562x::enableWD(bool enable)
    {
        return writeReg(Registers::Charger_Control_0_WATCHDOG, enable);
    }

    bool BQ2562x::enableTS(bool enable)
    {
        return writeReg(Registers::NTC_Control_0_TS_IGNORE, !enable);
    }

    bool BQ2562x::getPartInformation(uint8_t& value)
    {
        return readReg(Registers::Part_Information, value);
    }

    bool BQ2562x::enableCharging(bool state)
    {
        return writeReg(Registers::Charger_Control_0_EN_CHG, state);
    }

    float BQ2562x::getVBUS()
    {
        uint16_t value = 0;
        readReg(SHORT(0x2c), 2, 14, value);
        return (value * 3.97f) / 1000.0f;
    }

    float BQ2562x::getIBAT()
    {
        uint16_t value = 0;
        readReg(SHORT(0x2a), 2, 15, value);

        float partial = 0.0f;

        if (value >= 0x38AD && value <= 0x3FFF)
        {
            partial = (0x3FFF - value + 1) * -4.0f;
        }
        else
        {
            partial = value * 4.0f;
        }

        return partial / 1000.0f;
    }

    float BQ2562x::getIBUS()
    {
        uint16_t value = 0;
        readReg(SHORT(0x28), 1, 15, value);

        float partial = 0.0f;

        if (value >= 0x7830 && value <= 0x7FFF)
        {
            partial = (0x7FFF - value + 1) * -2.0f;
        }
        else
        {
            partial = value * 4.0f;
        }

        return partial / 1000.0f;
    }

    bool BQ2562x::getVBAT(float& value)
    {
        uint16_t res = 0;
        if (readReg(Registers::VBAT_ADC, res))
        {
            value = (res * 1.99f) / 1000.0f;
            return true;
        }
        return false;
    }

    void BQ2562x::setVINDPM(uint32_t mV)
    {
        writeReg(Registers::Input_Current_Limit_VINDPM, static_cast<uint16_t>((mV) / 40));
        // uint16_t value = 0;
        // readReg(Registers::Input_Current_Limit_VINDPM, value);
        // printf("vindpm: %d\n", value * 40);
    }

    bool BQ2562x::setIINDPM(uint32_t mA)
    {
        return writeReg(Registers::Input_Current_Limit_IINDPM, static_cast<uint16_t>((mA) / 40));
        // uint16_t value = 0;
        // readReg(Registers::Input_Current_Limit_IINDPM, value);
        // printf("vindpm: %d\n", value * 40);
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
                value |= 0b00 << 4;
                break;

            case ADCSampling::Bits_11:
                value |= 0b01 << 4;
                break;

            case ADCSampling::Bits_10:
                value |= 0b10 << 4;
                break;

            case ADCSampling::Bits_9:
                value |= 0b11 << 4;
                break;

            default:
                break;
            }

            value |= (average == ADCAverage::Running) << 3;
            value |= (averageInit == ADCAverageInit::New) << 2;
        }

        return writeReg(Registers::ADC_Control, value);
    }

    bool BQ2562x::getBatteryVoltage(float& value)
    {
        uint16_t data = 0;
        if (readReg(Registers::VBAT_ADC, data))
        {
            value = data / 1000.0f;
            return true;
        }
        return false;
    }

    bool BQ2562x::getVBUSStat(VBUSStat& stat)
    {
        uint8_t data = 0;
        if (readReg(Registers::Charger_Status_1_VBUS_STAT, data))
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

    bool BQ2562x::getChargeStat(ChargeStat& stat)
    {
        uint8_t value = 0;
        if (readReg(Registers::Charger_Status_1_CHG_STAT, value))
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

        return writeReg(Registers::Charger_Control_2_BATFET_CTRL, value);
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

        return writeReg(Registers::Charger_Control_2_BATFET_DLY, value);
    }

    bool BQ2562x::enableWVBUS(bool enable)
    {
        return writeReg(Registers::Charger_Control_2_WVBUS, enable);
    }

    bool BQ2562x::getBatteryTemperature(float& value)
    {
        uint16_t res = 0;
        if (readReg(Registers::TS_ADC, res))
        {
            value = res * 0.0961f;
            return true;
        }
        return false;
    }

    void BQ2562x::displayInfo()
    {
        uint8_t chargerStatus0 = 0;
        uint8_t chargerStatus1 = 0;
        uint8_t faultStatus0 = 0;
        uint8_t chargerFlag0 = 0;
        uint8_t chargerFlag1 = 0;
        uint8_t faultFlag0 = 0;
        uint8_t chargerMask0 = 0;
        uint8_t chargerMask1 = 0;
        uint8_t faultMask0 = 0;

        readReg(BQ2562x::Registers::Charger_Status_0, chargerStatus0);
        readReg(BQ2562x::Registers::Charger_Status_1, chargerStatus1);
        readReg(BQ2562x::Registers::FAULT_Status_0, faultStatus0);
        readReg(BQ2562x::Registers::Charger_Flag_0, chargerFlag0);
        readReg(BQ2562x::Registers::Charger_Flag_1, chargerFlag1);
        readReg(BQ2562x::Registers::FAULT_Flag_0, faultFlag0);
        readReg(BQ2562x::Registers::Charger_Mask_0, chargerMask0);
        readReg(BQ2562x::Registers::Charger_Mask_1, chargerMask1);
        readReg(BQ2562x::Registers::FAULT_Mask_0, faultMask0);

        printf("chargerStatus0: 0x%01x, chargerStatus1: 0x%01x, faultStatus0: 0x%01x, chargerFlag0: 0x%01x, chargerFlag1: 0x%01x, faultFlag0: 0x%01x, chargerMask0: 0x%01x, chargerMask1: 0x%01x, faultMask0: 0x%01x\n",
                chargerStatus0, chargerStatus1, faultStatus0, chargerFlag0,
                chargerFlag1, faultFlag0, chargerMask0, chargerMask1, faultMask0);
    }
}