#include <climits>

#include <soc/reset_reasons.h>
#include <SparkFunBQ27441.h>

#include "BQ2562x.hpp"
#include "Board.hpp"

static_assert(CHAR_BIT == 8, "Unsupported architecture");

static constexpr int I2C_NUM = 0;
static constexpr uint32_t I2C_SPEED = 400000;
static constexpr uint32_t I2C_TIMEOUT = 1000;

namespace PowerFeather
{
    bool Board::MasterI2C::init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq)
    {
        _port = port;

        // Initialize I2C bus
        i2c_config_t i2c_conf;
        memset(&i2c_conf, 0, sizeof(i2c_conf));
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = sda;
        i2c_conf.scl_io_num = scl;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
        i2c_conf.master.clk_speed = freq;

        esp_err_t res;
        if ((res = i2c_param_config(_port, &i2c_conf)) != ESP_OK)
        {
            return false;
        }

        if ((res = i2c_driver_install(_port, i2c_conf.mode, 0, 0, 0)) != ESP_OK)
        {
            return false;
        }

        return true;
    }

    template <typename T>
    bool Board::MasterI2C::read(uint8_t address, uint8_t reg, T &data)
    {
        uint8_t *data2 = reinterpret_cast<uint8_t*>(&data);
        return i2c_master_write_read_device(_port, address, &reg, sizeof(reg), data2, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    template <typename T>
    bool Board::MasterI2C::write(uint8_t address, uint8_t reg, T data)
    {
        uint8_t data2[sizeof(reg) + sizeof(data)] = { reg };
        memcpy(&data2[sizeof(reg)], &data, sizeof(data));
        return i2c_master_write_to_device(_port, address, data2, sizeof(data2), pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    template <typename T>
    bool Board::Charger::writeReg(uint8_t reg, uint8_t start, uint8_t end, T value)
    {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2);
        assert(end < sizeof(T) * CHAR_BIT);
        assert(start <= end);
        T data = 0;
        bool res = _i2c.read(_address, reg, data);
        if (res)
        {
            uint8_t bits = end - start + 1;
            T mask = ((0b1 << bits) - 1) << start;
            value <<= start;
            data = (data & ~mask) | (mask & value);
            res = _i2c.write(_address, reg, data);
        }
        return res;
    }

    bool Board::Charger::writeReg(uint8_t address, uint8_t bit, bool value)
    {
        return bit < CHAR_BIT ? writeReg(address, bit, bit, static_cast<uint8_t>(value))
                              : writeReg(address, bit, bit, static_cast<uint16_t>(value));
    }

    template <typename T>
    bool Board::Charger::readReg(uint8_t address, uint8_t start, uint8_t end, T &value)
    {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2);
        assert(end < sizeof(T) * CHAR_BIT);
        assert(start <= end);
        T data = 0;
        bool res = _i2c.read(address, address, data);
        if (res)
        {
            value = (data << (((sizeof(value) * CHAR_BIT) - 1) - end)) >> start;
        }
        return res;
    }

    bool Board::Charger::readReg(uint8_t address, uint8_t bit, bool &value)
    {
        uint8_t value1 = 0;
        uint16_t value2 = 0;
        bool res = bit < CHAR_BIT ? readReg(address, bit, bit, value1)
                              : readReg(address, bit, bit, value2);
        value = value1 | value2;
        return res;
    }

    template <typename T>
    bool Board::Charger::readReg(uint8_t address, T& value)
    {
        return readReg(address, 0, (sizeof(value) * CHAR_BIT) - 1, value);
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
            return _charger.writeReg(0x02, 5, 11, current);
        }
        return false;
    }

    bool Board::enableHeader5VOnBattery(bool enable)
    {
        return _charger.writeReg(0x18, 6, enable);
    }

    bool Board::_enableChargerWd(bool enable)
    {
        return _charger.writeReg(0x16, 0, 1, static_cast<uint8_t>(enable ? 0x1 : 0x0));
    }

    bool Board::_enableChargerTS(bool enable)
    {
        return _charger.writeReg(0x1a, 7, !enable);
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
        _charger.readReg(0x1f, data);
        return data;
    }

    bool Board::init()
    {
        soc_reset_reason_t reset_reason = esp_rom_get_reset_reason(0);

        _masterI2C.init(I2C_NUM, Board::SDA0Pin, Board::SCL0Pin, I2C_SPEED);

        if (reset_reason == RESET_REASON_CHIP_POWER_ON)
        {
            enableCharging(false);
            _enableChargerTS(_useTSPin);
        }

        if (reset_reason == RESET_REASON_CHIP_POWER_ON || 
            reset_reason == RESET_REASON_CHIP_BROWN_OUT ||
            reset_reason == RESET_REASON_CHIP_SUPER_WDT ||
            reset_reason == RESET_REASON_SYS_RTC_WDT || 
            reset_reason == RESET_REASON_SYS_SUPER_WDT ||
            reset_reason == RESET_REASON_SYS_CLK_GLITCH)
        {
            _initRTCPin(Board::EnableHeader3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initRTCPin(Board::EnableStemma3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initRTCPin(Board::EnablePin, RTC_GPIO_MODE_INPUT_OUTPUT_OD);

            enableHeader3V3(true);
            enableStemma3V3(true);
        }

        // Initialize IO pins
        _initDigitalPin(Board::ChargerIntPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::GaugeAlarmPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::GaugeRegPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::VDDTypePin, GPIO_MODE_INPUT);

        if (reset_reason == RESET_REASON_CHIP_POWER_ON)
        {
            _enableChargerWd(false);
            setChargeFactor(0.5f);
            enableHeader5VOnBattery(false);
        }

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
        _charger.writeReg(0x16, 5, state);
    }

    void Board::enableGauge(bool enable)
    {

    }
}
