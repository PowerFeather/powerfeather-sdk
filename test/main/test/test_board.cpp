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
#include <iperf.h>

#include <PowerFeather.h>

using namespace PowerFeather;

MainBoard& board = Board;

static constexpr char MODULE_NAME[] = "[MainBoard]";
static inline size_t MS_TO_US(size_t ms) { return ms * 1000; }


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
    board.enable3V3(true);
    board.enableVSQT(true);
    esp_deep_sleep_start();
}

TEST_CASE("test_deep_sleep_current_3V3_and_VSQT_enabled", MODULE_NAME)
{
    board.enable3V3(false);
    TEST_ASSERT_EQUAL(Result::Ok, board.enableVSQT(false));
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
    TEST_ASSERT_EQUAL(Result::Ok, board.enableTempSense(true));
    TEST_ASSERT_TRUE(board.getCharger().setupADC(true));

    uint8_t adcSetup = 0;
    TEST_ASSERT_TRUE(board.getCharger().readReg(BQ2562x::Registers::ADC_Control, adcSetup));
    TEST_ASSERT_EQUAL(0xb0, adcSetup);

    board.getCharger().displayInfo();

    while (true)
    {
        float temp = 0.0f;
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryTemperature(temp));
        printf("temperature: %f\n", temp);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

TEST_CASE("test_BTN_and_LED", MODULE_NAME)
{
    gpio_set_intr_type(MainBoard::Pin::BTN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
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

    bool connected, prev_connected;
    prev_connected = false;

    while (true)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyStatus(connected));
        if (connected != prev_connected)
        {
            printf("supply good: %d\n", connected);
            uint32_t duty = connected? 8192 : 820;
            // Set duty to 50%
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
            prev_connected = connected;
        }
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











TEST_CASE("battery health", MODULE_NAME)
{
    uint8_t percent;

    while (true)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryHealth(percent));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("discharging and charging", MODULE_NAME)
{
    // Measure VBAT, IBAT
    // Disable charging initially, until certain SOC
    // Enable charging, then disable again once another SOC is reached
    TEST_ASSERT_TRUE(board.getCharger().setupADC(true));
    board.getCharger().setChargeCurrent(50);
    TEST_ASSERT_EQUAL(Result::Ok, board.enableSupply(false));

    while (true)
    {
        uint8_t soc = 0;
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCharge(soc));

        if (soc > 70)
        {
            TEST_ASSERT_EQUAL(Result::Ok, board.enableSupply(false));
            TEST_ASSERT_EQUAL(Result::Ok, board.enableCharging(false));
        }
        else if (soc < 60)
        {
            TEST_ASSERT_EQUAL(Result::Ok, board.enableSupply(true));
            TEST_ASSERT_EQUAL(Result::Ok, board.enableCharging(true));
        } else { }

        int16_t ibat = 0.0f;
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryCurrent(ibat));

        uint16_t vbat = 0.0f;
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryVoltage(vbat));

        int timeLeft = 0;
        Result timeLeftRes = board.getBatteryTimeLeft(timeLeft);

        TEST_ASSERT_TRUE(timeLeftRes == Result::Ok || timeLeftRes == Result::NotReady);

        BQ2562x::ChargeStat stat;
        TEST_ASSERT_TRUE(board.getCharger().getChargeStat(stat));

        if (stat != BQ2562x::ChargeStat::Terminated)
        {
            TEST_ASSERT_EQUAL(ibat < 0, timeLeft < 0);
        }

        const char* statStr[] = {"terminated", "trickle", "taper", "topoff"};

        printf("time: %lld\tsoc: %d\tstat: %s\tvbat: %d mV\tibat: %d mA\ttimeLeft: %s\n",
                time(NULL), soc, statStr[static_cast<int>(stat)], vbat, ibat, timeLeftRes == Result::Ok ? std::to_string(timeLeft).c_str() : "<estimating>");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


TEST_CASE("fuel guage interrupt", MODULE_NAME)
{
    // Write a high temperature to register
}

TEST_CASE("set VBAT min", MODULE_NAME)
{
    TEST_ASSERT_EQUAL(Result::Ok, board.setVBATMinVoltage(3.7));
}

static esp_netif_t *wifi_netif = NULL;

static void start_iperf_server()
{
    // Start iperf
    iperf_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(wifi_netif, &ip);

    cfg.flag |= IPERF_FLAG_SERVER | IPERF_FLAG_TCP;
    cfg.source_ip4 = ip.ip.addr;
    cfg.type = IPERF_IP_TYPE_IPV4;
    cfg.dport = IPERF_DEFAULT_PORT;
    cfg.sport = IPERF_DEFAULT_PORT;
    cfg.interval = IPERF_DEFAULT_INTERVAL;
    cfg.time = UINT32_MAX;
    cfg.len_send_buf = 0;
    cfg.bw_lim = IPERF_DEFAULT_NO_BW_LIMIT;

    printf("mode=%s-%s sip=%lu.%lu.%lu.%lu:%d, dip=%lu.%lu.%lu.%lu:%d, interval=%lu, time=%lu\n",
             cfg.flag & IPERF_FLAG_TCP ? "tcp" : "udp",
             cfg.flag & IPERF_FLAG_SERVER ? "server" : "client",
             cfg.source_ip4 & 0xFF, (cfg.source_ip4 >> 8) & 0xFF, (cfg.source_ip4 >> 16) & 0xFF,
             (cfg.source_ip4 >> 24) & 0xFF, cfg.sport,
             cfg.destination_ip4 & 0xFF, (cfg.destination_ip4 >> 8) & 0xFF,
             (cfg.destination_ip4 >> 16) & 0xFF, (cfg.destination_ip4 >> 24) & 0xFF, cfg.dport,
             cfg.interval, cfg.time);

    iperf_start(&cfg);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        printf("station " MACSTR " join, AID=%d\n", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        printf("station " MACSTR " leave, AID=%d\n", MAC2STR(event->mac), event->aid);
    }
}

static void start_ap()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    wifi_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL);

    wifi_config_t wifi_config;

    static constexpr char SSID[] = "powerfeather-ap";
    static constexpr char PASS[] = "powerfeather-pswd";

    memset(&wifi_config, 0, sizeof(wifi_config));

    strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), SSID, sizeof(wifi_config.ap.ssid));
    strncpy(reinterpret_cast<char*>(wifi_config.ap.password), PASS, sizeof(wifi_config.ap.password));

    wifi_config.ap.ssid_len = strlen(SSID);
    wifi_config.ap.channel = 6;
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen(PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
}

bool is_iperf_running()
{
    TaskHandle_t iperf_task = xTaskGetHandle("iperf_traffic");
    return iperf_task;
}

TEST_CASE("current loading", MODULE_NAME)
{
    TEST_ASSERT_TRUE(board.getCharger().setupADC(true));

    start_ap();

    while (true)
    {
        if (!is_iperf_running())
        {
            start_iperf_server();
        }

        uint16_t vbus = 0.0f;
        int16_t ibus = 0.0f;
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyVoltage(vbus));
        TEST_ASSERT_EQUAL(Result::Ok, board.getSupplyCurrent(ibus));

        printf("vbus: %d mV ibus: %d mA\n", vbus, ibus);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}