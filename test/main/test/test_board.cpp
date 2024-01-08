#include <time.h>
#include <string>

#include <esp_private/system_internal.h>
#include <esp_sleep.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_timer.h>
#include <esp_mac.h>

#include <unity.h>

#include <PowerFeather.h>

using namespace PowerFeather;

ESP_EVENT_DEFINE_BASE(TEST_EVENTS);

#define CHARGER_INTERRUPT           1
#define FUEL_GAUGE_ALARM            2

MainBoard& board = Board;

static constexpr char MODULE_NAME[] = "[MainBoard]";
static inline size_t MS_TO_US(size_t ms) { return ms * 1000; }


static void display_charger_status(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    // Board.getCharger().displayInfo();
}

static void display_fuel_gauge_status(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    // printf("fuel gauge alarm!\n");
}

static void wait_for_battery()
{
    bool connected = true;
    while (connected)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        board.getSupplyStatus(connected);
    }
    gpio_set_level(MainBoard::Pin::LED, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

static void display_voltages_and_currents()
{
    bool gsup = false;
    int16_t isup = 0, ibat = 0;
    uint16_t vsup = 0, vbat = 0;

    TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyStatus(gsup));
    TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyVoltage(vsup));
    TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyCurrent(isup));
    TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryVoltage(vbat));
    TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCurrent(ibat));

    printf("supply good: %d\tsupply voltage:%d mV\tsupply current: %d mA\tbattery voltage: %d mV\tbattery current:%d mA\n",
            gsup, vsup, isup, vbat, ibat);
}

TEST_CASE("test_EN", MODULE_NAME)
{
    // Tie potentiometer to temperature sense
    // Check interrupt, may be combined with another test
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (uint64_t)0b1 << MainBoard::Pin::EN;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    bool enable = false;

    while (true)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.setEN(enable));
        vTaskDelay(pdMS_TO_TICKS(1000));
        bool _enable = gpio_get_level(MainBoard::Pin::EN);
        TEST_ASSERT_EQUAL(enable, _enable);
        printf("enable: %d\n", _enable);
        enable = !enable;
    }
}

TEST_CASE("test_deep_sleep_current_3V3_and_VSQT_disabled", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(Result::Ok, board.enable3V3(false));
    TEST_ASSERT_EQUAL(Result::Ok, board.enableVSQT(false));
    esp_deep_sleep_start();
}

TEST_CASE("test_deep_sleep_current_3V3_and_VSQT_enabled", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(Result::Ok, board.enable3V3(true));
    TEST_ASSERT_EQUAL(Result::Ok, board.enableVSQT(true));
    esp_deep_sleep_start();
}

TEST_CASE("test_3V3_VSQT_EN_on_glitch_deep_sleep", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableVSQT(true);
    board.setEN(true);
    esp_deep_sleep(MS_TO_US(100));
}


TEST_CASE("test_3V3_VSQT_EN_off_glitch_deep_sleep", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableVSQT(false);
    board.setEN(false);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("test_3V3_VSQT_EN_on_glitch_deep_sleep", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableVSQT(true);
    board.setEN(true);
    esp_restart_noos_dig();
}

TEST_CASE("test_3V3_VSQT_EN_off_glitch_deep_sleep", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableVSQT(false);
    board.setEN(false);
    esp_restart_noos_dig();
}

TEST_CASE("test_TS", MODULE_NAME)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (uint64_t)0b1 << MainBoard::Pin::INT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    TEST_ASSERT_EQUAL(ESP_OK, esp_event_handler_instance_register(TEST_EVENTS, CHARGER_INTERRUPT, display_charger_status, NULL, NULL));
    gpio_isr_handler_add(MainBoard::Pin::INT, [](void* arg) { esp_event_post(TEST_EVENTS, CHARGER_INTERRUPT, NULL, 0, portMAX_DELAY); }, NULL);

    float temp = 0.0f;

    //TODO: check that temperature reading is invalid before enabling temperature
    TEST_ASSERT_EQUAL(Result::InvalidState, board.getBatteryTemperature(temp));

    TEST_ASSERT_EQUAL(Result::Ok, board.enableTempSense(true));

    uint8_t adcSetup = 0;
    TEST_ASSERT_TRUE(board.getCharger().readReg(BQ2562x::Registers::ADC_Control, adcSetup));
    TEST_ASSERT_EQUAL(0xb0, adcSetup);

    board.getCharger().displayInfo();

    while (true)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryTemperature(temp));
        printf("temperature: %f\n", temp);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

TEST_CASE("test_BTN_and_LED", MODULE_NAME)
{
    gpio_set_intr_type(MainBoard::Pin::BTN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(MainBoard::Pin::BTN, [](void* arg) {gpio_set_level(MainBoard::Pin::LED, gpio_get_level(MainBoard::Pin::BTN));}, NULL);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("test_power_inputs", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(ESP_OK, gpio_reset_pin(MainBoard::Pin::LED));

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 4000,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    TEST_ASSERT_EQUAL(ESP_OK, ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    memset(&ledc_channel, 0, sizeof(ledc_channel));
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = LEDC_CHANNEL_0;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = MainBoard::Pin::LED;
    ledc_channel.duty           = 0; // Set duty to 0;
    ledc_channel.hpoint         = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ledc_channel_config(&ledc_channel));

    while (true)
    {
        bool suppg = false;
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyStatus(suppg));
        uint32_t duty = suppg? 8192 : 820;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

        display_voltages_and_currents();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

TEST_CASE("test_ship_mode", MODULE_NAME)
{
    wait_for_battery();
    board.enterShipMode();
}

TEST_CASE("test_shutdown_mode", MODULE_NAME)
{
    wait_for_battery();
    board.enterShutdownMode();
}

TEST_CASE("test_power_cycle", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(Result::Ok, board.doPowerCycle());
}

TEST_CASE("test_free_io", MODULE_NAME)
{
    auto setup_pin = [](gpio_num_t pin)
    {
        // Output frequency same as pin number, i.e. GPIO1 output 1Hz,
        // GPIO2 output 2Hz and so on.
        gpio_config_t io_conf = {};
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pin_bit_mask = (static_cast<uint64_t>(0b1) << pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        esp_timer_create_args_t periodic_timer_args;
        memset(&periodic_timer_args, 0, sizeof(periodic_timer_args));

        periodic_timer_args.callback = [](void* arg)
        {
            gpio_num_t pin = static_cast<gpio_num_t>((int)arg);
            gpio_set_level(pin, !gpio_get_level(pin));
        };

        periodic_timer_args.arg = reinterpret_cast<void*>(pin);

        esp_timer_handle_t periodic_timer;
        esp_timer_create(&periodic_timer_args, &periodic_timer);
        esp_timer_start_periodic(periodic_timer, ((100000 / (pin))) / 2);
    };

    gpio_num_t free_io[] = {
        MainBoard::Pin::A0,
        MainBoard::Pin::A1,
        MainBoard::Pin::A2,
        MainBoard::Pin::A3,
        MainBoard::Pin::A4,
        MainBoard::Pin::A5,
        MainBoard::Pin::D5,
        MainBoard::Pin::D6,
        MainBoard::Pin::D7,
        MainBoard::Pin::D8,
        MainBoard::Pin::D9,
        MainBoard::Pin::D10,
        MainBoard::Pin::D11,
        MainBoard::Pin::D12,
        MainBoard::Pin::D13,
        MainBoard::Pin::RX,
        MainBoard::Pin::TX,
        MainBoard::Pin::TX0,
        MainBoard::Pin::SCK,
        MainBoard::Pin::MOSI,
        MainBoard::Pin::MISO,
        MainBoard::Pin::SDA,
        MainBoard::Pin::SCL
    };

    for (gpio_num_t io: free_io)
    {
        setup_pin(io);
    }

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("test_battery_alarms", MODULE_NAME)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (uint64_t)0b1 << MainBoard::Pin::ALARM;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    TEST_ASSERT_EQUAL(ESP_OK, esp_event_handler_instance_register(TEST_EVENTS, FUEL_GAUGE_ALARM, display_fuel_gauge_status, NULL, NULL));
    gpio_isr_handler_add(MainBoard::Pin::ALARM, [](void* arg) { esp_event_post(TEST_EVENTS, FUEL_GAUGE_ALARM, NULL, 0, portMAX_DELAY); }, NULL);

    TEST_ASSERT_EQUAL(Result::Ok, board.setBatteryLowVoltageAlarm(3600));
    TEST_ASSERT_EQUAL(Result::Ok, board.setBatteryHighVoltageAlarm(4000));
    TEST_ASSERT_EQUAL(Result::Ok, board.setBatteryLowChargeAlarm(30));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("test_battery_information", MODULE_NAME)
{

    TEST_ASSERT_TRUE(true);
}

TEST_CASE("set VBAT min", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(Result::Ok, board.setVBATMinVoltage(3.7));
}

TEST_CASE("test_current_loading", MODULE_NAME)
{
    board.setSupplyMaxCurrent(2000);

    while (true)
    {
        display_voltages_and_currents();
    }
}

TEST_CASE("test_battery_charging", MODULE_NAME)
{
    // Measure VBAT, IBAT
    // Disable charging initially, until certain SOC
    // Enable charging, then disable again once another SOC is reached
    TEST_ASSERT_EQUAL(Result::Ok, board.enableSupply(false));
    TEST_ASSERT_EQUAL(Result::Ok, board.setChargingMaxCurrent(1000));
    TEST_ASSERT_EQUAL(Result::Ok, board.setSupplyMaxCurrent(2000));

    static constexpr uint8_t minSoc = 60;
    static constexpr uint8_t maxSoc = 70;

    while (true)
    {
        uint8_t soc = 0;
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCharge(soc));

        if (soc > maxSoc)
        {
            TEST_ASSERT_EQUAL(Result::Ok, board.enableSupply(false));
            TEST_ASSERT_EQUAL(Result::Ok, board.enableCharging(false));
        }
        else if (soc < minSoc)
        {
            TEST_ASSERT_EQUAL(Result::Ok, board.enableSupply(true));
            TEST_ASSERT_EQUAL(Result::Ok, board.enableCharging(true));
        } else
        {
            // Do nothing
        }

        int16_t ibat = 0, isup = 0;
        uint16_t vbat = 0;
        int timeLeft = 0;
        BQ2562x::ChargeStat stat;

        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCurrent(ibat));
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryVoltage(vbat));
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyCurrent(isup));
        Result timeLeftRes = board.getBatteryTimeLeft(timeLeft);
        TEST_ASSERT_TRUE(timeLeftRes == Result::Ok || timeLeftRes == Result::NotReady);
        TEST_ASSERT_TRUE(board.getCharger().getChargeStat(stat));

        if (stat != BQ2562x::ChargeStat::Terminated)
        {
            TEST_ASSERT_EQUAL(ibat < 0, timeLeft < 0);
        }

        const char* statStr[] = {"terminated", "trickle", "taper", "topoff"};

        printf("time: %lld\tcharge: %d\tphase: %s\tbattery voltage: %d mV\tbattery current: %d mA\tsupply current: %d mA\ttime left: %s\n",
                time(NULL), soc, statStr[static_cast<int>(stat)], vbat, ibat, isup,
                timeLeftRes == Result::Ok ? std::to_string(timeLeft).c_str() : "<estimating>");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}