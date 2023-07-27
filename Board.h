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

        enum class Pin
        {
            A0 =   static_cast<gpio_num_t>(5),
            A1 =   static_cast<gpio_num_t>(5),
            A2 =   static_cast<gpio_num_t>(5),
            A3 =   static_cast<gpio_num_t>(5),
            A4 =   static_cast<gpio_num_t>(5),
            A5 =   static_cast<gpio_num_t>(5),
            A6 =   static_cast<gpio_num_t>(5),
            D6 =   static_cast<gpio_num_t>(5),
            D7 =   static_cast<gpio_num_t>(5),
            D8 =   static_cast<gpio_num_t>(5),
            D9 =   static_cast<gpio_num_t>(5),
            D10 =  static_cast<gpio_num_t>(5),
            D11 =  static_cast<gpio_num_t>(5),
            D12 =  static_cast<gpio_num_t>(5),
            D13 =  static_cast<gpio_num_t>(5),
            SCL =  static_cast<gpio_num_t>(5),
            SDA =  static_cast<gpio_num_t>(5),
            EN =   static_cast<gpio_num_t>(5),
            LED =  static_cast<gpio_num_t>(6),
            BTN =  static_cast<gpio_num_t>(0),
        };

        bool init();
        void enableHeader3V3(bool enable);
        void enableStemma3V3(bool enable);
        bool getEnablePin();
        void setEnablePin(bool value);
        PowerInput getPowerInput();

        BQ2562x& getCharger() { return _charger; }
        LC70924F& getFuelGauge() {return _fuelGauge; }

    protected:

        enum class InternalPin
        {
            SQT_3V3 = static_cast<gpio_num_t>(14),
            HDR_3V3 = static_cast<gpio_num_t>(4),
            REGN = static_cast<gpio_num_t>(4),
            ALARM = static_cast<gpio_num_t>(21),
            INT = static_cast<gpio_num_t>(5),
            SCL0 = static_cast<gpio_num_t>(47),
            SDA0 = static_cast<gpio_num_t>(48),
            EN2 = static_cast<gpio_num_t>(0)
        };

        // Input
        static constexpr gpio_num_t ChargerIntPin = static_cast<gpio_num_t>(5);
        static constexpr gpio_num_t GaugeAlarmPin = static_cast<gpio_num_t>(21);
        static constexpr gpio_num_t GaugeRegPin = static_cast<gpio_num_t>(7);
        static constexpr gpio_num_t VDDTypePin = static_cast<gpio_num_t>(38);

        // Output
        static constexpr gpio_num_t EnableHeader3V3Pin = static_cast<gpio_num_t>(4);
        static constexpr gpio_num_t EnableStemma3V3Pin = static_cast<gpio_num_t>(14);

        // Input + Output
        static constexpr gpio_num_t EnablePin = static_cast<gpio_num_t>(13);

        static constexpr gpio_num_t SCL0Pin = static_cast<gpio_num_t>(47);
        static constexpr gpio_num_t SDA0Pin = static_cast<gpio_num_t>(48);

    private:
        static constexpr int _i2c_port = 0;
        static constexpr uint32_t _i2c_freq = 400000;
        static constexpr uint32_t _i2c_timeout = 1000;

        MasterI2C _masterI2C {};
        BQ2562x _charger {_masterI2C};
        LC70924F _fuelGauge {_masterI2C};

        bool _initRTCPin(int pin, rtc_gpio_mode_t mode);
        bool _initDigitalPin(int pin, gpio_mode_t mode);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
