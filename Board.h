#include <driver/rtc_io.h>

#include <BQ2562x.h>
#include <LC709204F.h>
#include <MasterI2C.h>

namespace PowerFeather
{
    class Board
    {
    public:
        enum class PowerInput
        {
            USB,
            DC,
            Battery,
        };

        class Pin
        {
        public:
            static constexpr gpio_num_t A0 =   GPIO_NUM_5;
            static constexpr gpio_num_t A1 =   GPIO_NUM_5;
            static constexpr gpio_num_t A2 =   GPIO_NUM_5;
            static constexpr gpio_num_t A3 =   GPIO_NUM_5;
            static constexpr gpio_num_t A4 =   GPIO_NUM_5;
            static constexpr gpio_num_t A5 =   GPIO_NUM_5;
            static constexpr gpio_num_t A6 =   GPIO_NUM_5;
            static constexpr gpio_num_t D6 =   GPIO_NUM_5;
            static constexpr gpio_num_t D7 =   GPIO_NUM_5;
            static constexpr gpio_num_t D8 =   GPIO_NUM_5;
            static constexpr gpio_num_t D9 =   GPIO_NUM_5;
            static constexpr gpio_num_t D10 =  GPIO_NUM_5;
            static constexpr gpio_num_t D11 =  GPIO_NUM_5;
            static constexpr gpio_num_t D12 =  GPIO_NUM_5;
            static constexpr gpio_num_t D13 =  GPIO_NUM_5;
            static constexpr gpio_num_t SCL =  GPIO_NUM_5;
            static constexpr gpio_num_t SDA =  GPIO_NUM_5;
            static constexpr gpio_num_t TX =   GPIO_NUM_5;
            static constexpr gpio_num_t RX =   GPIO_NUM_5;
            static constexpr gpio_num_t MISO = GPIO_NUM_5;
            static constexpr gpio_num_t MOSI = GPIO_NUM_5;
            static constexpr gpio_num_t SCK =  GPIO_NUM_5;
        };

        class Signal
        {
        public:
            static constexpr gpio_num_t EN =      GPIO_NUM_5; // Board EN, Input
            static constexpr gpio_num_t REGN =    GPIO_NUM_4; // FuelGauge REGN, Input
            static constexpr gpio_num_t ALARM =   GPIO_NUM_21; // FuelGauge ALARM, Input
            static constexpr gpio_num_t INT =     GPIO_NUM_5; // Charger INT, Input
            static constexpr gpio_num_t VDDTYPE = GPIO_NUM_4; // Board VDDTYPE, Input
            static constexpr gpio_num_t LED =     GPIO_NUM_6; // Board LED, Output
            static constexpr gpio_num_t BTN =     GPIO_NUM_0; // Board BTN, Input
            static constexpr gpio_num_t SQT_SCL = GPIO_NUM_5; // Board STEMMA QT SCL
            static constexpr gpio_num_t SQT_SDA = GPIO_NUM_5; // Board STEMMA QT SDA
        };

        bool init();
        void enableHeader3V3(bool enable);
        void enableStemma3V3(bool enable);
        void setEnablePin(bool value);
        PowerInput getPowerInput();

        BQ2562x& getCharger() { return _charger; }
        LC70924F& getFuelGauge() { return _fuelGauge; }

    private:
        class _Signal
        {
        public:
            static constexpr gpio_num_t SQT_3V3 = GPIO_NUM_14;
            static constexpr gpio_num_t HDR_3V3 = GPIO_NUM_4;
            static constexpr gpio_num_t EN2 =     GPIO_NUM_0;
            static constexpr gpio_num_t SCL0 =    GPIO_NUM_47;
            static constexpr gpio_num_t SDA0 =    GPIO_NUM_4;
        };

        static constexpr int _i2c_port = 0;
        static constexpr uint32_t _i2c_freq = 400000;
        static constexpr uint32_t _i2c_timeout = 1000;

        MasterI2C _masterI2C {};
        BQ2562x _charger {_masterI2C};
        LC70924F _fuelGauge {_masterI2C};

        bool _initRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        bool _initDigitalPin(gpio_num_t pin, gpio_mode_t mode);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
