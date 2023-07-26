#include <climits>

#include <driver/i2c.h>
#include <SparkFunBQ27441.h>

#include "BQ2562x.hpp"
#include "Board.hpp"


static_assert(CHAR_BIT == 8, "Unsupported architecture");


static constexpr int I2C_NUM = 0;
static constexpr uint32_t I2C_SPEED = 400000;
static constexpr uint32_t I2C_TIMEOUT = 1000;

static constexpr uint8_t BQ2562x_ADDR = 0x6a;

namespace PowerFeather
{
    bool Board::_readI2C(uint8_t address, uint8_t reg, uint8_t &data)
    {
        return i2c_master_write_read_device(I2C_NUM, address, &reg, sizeof(reg), &data, 1, pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    bool Board::_writeI2C(uint8_t address, uint8_t reg, uint8_t data)
    {
        uint8_t data2[] = {reg, data};
        return i2c_master_write_to_device(I2C_NUM, address, data2, sizeof(data2), pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    bool Board::_readI2C(uint8_t address, uint8_t reg, uint16_t &data)
    {
        uint8_t *data2 = reinterpret_cast<uint8_t*>(&data);
        return i2c_master_write_read_device(I2C_NUM, address, &reg, sizeof(reg), data2, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    bool Board::_writeI2C(uint8_t address, uint8_t reg, uint16_t data)
    {
        uint8_t *data2 = reinterpret_cast<uint8_t*>(&data);
        uint8_t data3[] = {reg, data2[0], data2[1]};
        return i2c_master_write_to_device(I2C_NUM, address, data3, sizeof(data3), pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    Board::Board(uint16_t batteryCapacity, bool useTSPin)
    {
        this->_batteryCapacity = batteryCapacity;
        this->_useTSPin = useTSPin;
    }

    bool Board::setChargeFactor(float factor)
    {
        uint16_t current = (_batteryCapacity * factor) / 40;
        if (current >= 0x1 && current <= 0x32)
        {
            return _setRegisterValue(0x02, 5, 11, current);
        }
        return false;
    }

    bool Board::setBatteryModeHeader5V(Board::BatteryModeHeader5V mode)
    {
        bool res = false;
        switch (mode)
        {
        case Board::BatteryModeHeader5V::None:
            res = _setRegisterValue(0x18, 6, 0b0 & _setRegisterValue(0x18, 7, 0b0));
            break;

        case Board::BatteryModeHeader5V::Bypass:
            res = _setRegisterValue(0x18, 6, 0b0) & _setRegisterValue(0x18, 7, 0b1);
            break;

        case Board::BatteryModeHeader5V::Reg:
            res = _setRegisterValue(0x18, 6, 0b1) & _setRegisterValue(0x18, 7, 0b1);
            break;
        
        default:
            break;
        }
        return res;
    }

    bool Board::setVoltageHeader5V(float voltage)
    {
        uint16_t volts = voltage * 1000;
        if (volts >= 3840 && volts <= 5040)
        {
            uint16_t volts2 = volts / 80;
            return _setRegisterValue(0x0C, 6, 12, volts2);
        }
        return false;
    }

    bool Board::_setRegisterValue(uint8_t address, uint8_t bit, bool value)
    {
        return bit < CHAR_BIT ? _setRegisterValue(address, bit, bit, static_cast<uint8_t>(value)) 
                              : _setRegisterValue(address, bit, bit, static_cast<uint16_t>(value));
    }

    template <typename T>
    bool Board::_setRegisterValue(uint8_t address, uint8_t start, uint8_t end, T value)
    {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2);
        assert(end < sizeof(T) * CHAR_BIT);
        assert(start <= end);
        T data = 0;
        bool res = this->_readI2C(BQ2562x_ADDR, address, data);
        if (res)
        {
            uint8_t bits = end - start + 1;
            T mask = ((0b1 << bits) - 1) << start;
            value <<= start;
            data = (data & ~mask) | (mask & value);
            res = this->_writeI2C(BQ2562x_ADDR, address, data);
        }
        return res;
    }

    bool Board::_setRegisterValue(uint8_t address, uint16_t value)
    {
        return _setRegisterValue(address, 0, (sizeof(value) * CHAR_BIT) - 1, value);
    }

    bool Board::_enableChargerStatLed(bool enable)
    {
        return _setRegisterValue(0x15, 7, !enable);
    }

    bool Board::_enableChargerWd(bool enable)
    {
        return _setRegisterValue(0x16, 0, 1, static_cast<uint8_t>(enable ? 0x1 : 0x0));
    }

    bool Board::_enableChargerTS(bool enable)
    {
        return _setRegisterValue(0x1a, 7, !enable);
    }

    bool Board::_initRTCPin(int pin, rtc_gpio_mode_t mode)
    {
        rtc_gpio_init(static_cast<gpio_num_t>(pin));
        rtc_gpio_set_direction(static_cast<gpio_num_t>(pin), mode);
        rtc_gpio_set_direction_in_sleep(static_cast<gpio_num_t>(pin), mode);
        return true;
    }

    bool Board::_initDigitalPin(int pin, gpio_mode_t mode)
    {
        gpio_config_t io_conf = {};
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = mode;
        io_conf.pin_bit_mask = (0b1 << pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        return true;
    }

    uint8_t Board::_getChargerFault()
    {
        uint8_t data;
        this->_readI2C(BQ2562x_ADDR, 0x1f, data);
        return data;
    }

    bool Board::init()
    {
        // Initialize I2C bus
        int i2c_master_port = I2C_NUM;

        i2c_config_t i2c_conf;
        memset(&i2c_conf, 0, sizeof(i2c_conf));
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = Board::SDA0Pin;
        i2c_conf.scl_io_num = Board::SCL0Pin;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.master.clk_speed = I2C_SPEED;

        esp_err_t res;
        if ((res = i2c_param_config(i2c_master_port, &i2c_conf)) != ESP_OK)
        {
            return false;
        }
        
        if ((res = i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0)) != ESP_OK)
        {
            return false;
        }

        // Prioritize charging disable after I2C initialization
        enableCharging(false);

        // Initialize IO pins
        _initDigitalPin(Board::ChargerIntPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::GaugeAlarmPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::GaugeRegPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::VDDTypePin, GPIO_MODE_INPUT);

        _initRTCPin(Board::EnableHeader3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);
        _initRTCPin(Board::EnableStemma3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);

        _initRTCPin(Board::EnablePin, RTC_GPIO_MODE_INPUT_OUTPUT_OD);

        // Rest of initialization
        _enableChargerTS(_useTSPin);
        _enableChargerWd(false);
        setChargeFactor(0.5f);

        setBatteryModeHeader5V(Board::BatteryModeHeader5V::None);
        setVoltageHeader5V(5.04);

        enableHeader3V3(true);
        enableStemma3V3(true);

        return true;
    }

    void Board::_setRTCPin(gpio_num_t pin, bool value)
    {
        if (value)
        {
            // Set the pin high.
            rtc_gpio_set_level(pin, value);
            // Hold the pin high, even in deep sleep.
            rtc_gpio_hold_en(pin);
        }
        else
        {
            // Disable pin hold during deep sleep
            rtc_gpio_hold_dis(pin);

            if (pin == Board::EnablePin)
            {
                // The enable pin is in open-drain configuration with external pull-up
                // resistor. Setting the pin to 0 means pulling it down.
                rtc_gpio_set_level(pin, 0);
            }
            else
            {
                // The rest of the output RTC pins is by default pulled down by an
                // external resistor, this means setting them to 0 entails just
                // just disconnecting the pin from the chip.
                rtc_gpio_isolate(pin);
            }
        }
    }

    void Board::enableHeader3V3(bool enable)
    {
        _setRTCPin(Board::EnableHeader3V3Pin, enable);
    }

    void Board::enableStemma3V3(bool enable)
    {
        _setRTCPin(Board::EnableStemma3V3Pin, enable);
    }

    bool Board::getEnablePin()
    {
        return rtc_gpio_get_level(Board::EnablePin);
    }

    void Board::setEnablePin(bool value)
    {
        _setRTCPin(Board::EnablePin, value);
    }

    void Board::enableCharging(bool state)
    {
        _setRegisterValue(0x16, 5, state);
    }

    void Board::enableGauge(bool enable)
    {

    }
}
