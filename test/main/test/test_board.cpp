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

static void disable_fuel_gauge()
{
    uint8_t charge = 0;
    if (board.getBatteryCharge(charge) == Result::Ok) // make sure fuel guage is disabled
    {
        board.enableFuelGauge(false);
        TEST_ASSERT_EQUAL(Result::InvalidState, board.getBatteryCharge(charge));
        printf("disabled fuel gauge\n");
    }
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

TEST_CASE("test_3V3_VSQT_EN_on_glitch_deep_sleep", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableVSQT(true);
    board.setEN(true);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("test_3V3_VSQT_EN_on_glitch_digital_reset", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableVSQT(true);
    board.setEN(true);
    esp_restart();
}

TEST_CASE("test_3V3_VSQT_EN_on_glitch_light_sleep", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableVSQT(true);
    board.setEN(true);
    esp_sleep_enable_timer_wakeup(MS_TO_US(1000));
    esp_light_sleep_start();
}

TEST_CASE("test_3V3_VSQT_EN_off_glitch_deep_sleep", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableVSQT(false);
    board.setEN(false);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("test_3V3_VSQT_EN_off_glitch_digital_reset", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableVSQT(false);
    board.setEN(false);
    esp_restart();
}

TEST_CASE("test_3V3_VSQT_EN_off_glitch_light_sleep", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableVSQT(false);
    board.setEN(false);
    esp_sleep_enable_timer_wakeup(MS_TO_US(1000));
    esp_light_sleep_start();
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

    for (int i = 0; i < 10; i++)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.setEN(enable));
        vTaskDelay(pdMS_TO_TICKS(1000));

        bool _enable = gpio_get_level(MainBoard::Pin::EN);

        printf("en0: %d\ten: %d\n", enable, _enable);
        TEST_ASSERT_EQUAL(enable, _enable);
        enable = !enable;
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

TEST_CASE("test_power_cycle", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(Result::Ok, board.doPowerCycle());
}

TEST_CASE("test_ship_mode", MODULE_NAME)
{
    disable_fuel_gauge();
    wait_for_battery();
    board.enterShipMode();
}

TEST_CASE("test_shutdown_mode", MODULE_NAME)
{
    disable_fuel_gauge();
    wait_for_battery();
    board.enterShutdownMode();
}

TEST_CASE("test_deep_sleep_current_3V3_and_VSQT_enabled", MODULE_NAME)
{
    uint8_t charge = 0;
    TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCharge(charge));
    TEST_ASSERT_EQUAL(Result::Ok, board.enable3V3(true));
    TEST_ASSERT_EQUAL(Result::Ok, board.enableVSQT(true));
    esp_deep_sleep_start();
}

TEST_CASE("test_deep_sleep_current_3V3_and_VSQT_disabled", MODULE_NAME)
{
    uint8_t charge = 0;
    TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCharge(charge));
    TEST_ASSERT_EQUAL(Result::Ok, board.enable3V3(false));
    TEST_ASSERT_EQUAL(Result::Ok, board.enableVSQT(false));
    esp_deep_sleep_start();
}

TEST_CASE("test_deep_sleep_current_fuel_gauge_disabled", MODULE_NAME)
{
    disable_fuel_gauge();
    esp_deep_sleep_start();
}

TEST_CASE("test_TS", MODULE_NAME)
{
    static constexpr uint32_t eventId = __LINE__;
    TEST_ASSERT_EQUAL(ESP_OK, esp_event_handler_instance_register(TEST_EVENTS, eventId,
                      [](void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
                      {
                        printf("charger interrupt\n");
                      }
                      , NULL, NULL));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (uint64_t)0b1 << MainBoard::Pin::INT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_isr_handler_add(MainBoard::Pin::INT, [](void* arg) { esp_event_post(TEST_EVENTS, eventId, NULL, 0, portMAX_DELAY); }, NULL);

    // check that temperature reading is invalid before enabling temperature
    float temp = 0.0f;
    TEST_ASSERT_EQUAL(Result::InvalidState, board.getBatteryTemperature(temp));
    TEST_ASSERT_EQUAL(Result::Ok, board.enableTempSense(true));

    while (true)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryTemperature(temp));
        printf("temperature: %f\n", temp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

TEST_CASE("test_battery_voltage_alarms", MODULE_NAME)
{
    static constexpr uint32_t eventId = __LINE__;
    TEST_ASSERT_EQUAL(ESP_OK, esp_event_handler_instance_register(TEST_EVENTS, eventId,
            [](void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
                uint16_t vbat = 0;
                TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryVoltage(vbat));
                printf("battery voltage alarm: %d\n", vbat);
            }, NULL, NULL));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (uint64_t)0b1 << MainBoard::Pin::ALARM;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_isr_handler_add(MainBoard::Pin::ALARM, [](void* arg) { esp_event_post(TEST_EVENTS, eventId, NULL, 0, portMAX_DELAY); }, NULL);

    TEST_ASSERT_EQUAL(Result::Ok, board.setBatteryLowVoltageAlarm(3600));
    TEST_ASSERT_EQUAL(Result::Ok, board.setBatteryHighVoltageAlarm(4000));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("test_battery_charging", MODULE_NAME)
{
    static constexpr uint8_t minSoc = 40;
    static constexpr uint8_t maxSoc = 60;

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
        BQ2562x::ChargeStat stat = BQ2562x::ChargeStat::Terminated;

        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCurrent(ibat));
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryVoltage(vbat));
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyCurrent(isup));
        Result timeLeftRes = board.getBatteryTimeLeft(timeLeft);
        TEST_ASSERT_TRUE(timeLeftRes == Result::Ok || timeLeftRes == Result::NotReady);

        printf("time: %lld\tcharge: %d\t\tbattery voltage: %d mV\tbattery current: %d mA\tsupply current: %d mA\ttime left: %s\n",
                time(NULL), soc, vbat, ibat, isup,
                timeLeftRes == Result::Ok ? std::to_string(timeLeft).c_str() : "<estimating>");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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


TEST_CASE("test_power_inputs", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(ESP_OK, gpio_set_level(MainBoard::Pin::LED, 0));
    // TEST_ASSERT_EQUAL(Result::Ok, board.setSupplyMaxCurrent(1000));

    while (true)
    {
        bool good = false;
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyStatus(good));
        display_voltages_and_currents();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}