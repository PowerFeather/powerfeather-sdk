#include <time.h>

#include <esp_private/system_internal.h>
#include <esp_sleep.h>
#include <driver/ledc.h>
#include <esp_wifi.h>
#include <nvs_flash.h>

#include <unity.h>
#include <iperf.h>

#include <Board.h>
#include <BQ2562x.h>

using namespace PowerFeather;

Board board;

static constexpr char MODULE_NAME[] = "[Board]";
static inline size_t MS_TO_US(size_t ms) { return ms * 1000; }

TEST_CASE("rtc outputs off, no glitch on deep sleep and wake", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableSQT(false);
    board.setEN(false);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("rtc outputs on, no glitch on deep sleep and wake", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableSQT(true);
    board.setEN(true);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("3.3v power outputs on, deep sleep current draw", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableSQT(true);
    esp_deep_sleep(MS_TO_US(10000));
}

TEST_CASE("3.3V power outputs off, deep sleep current draw", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableSQT(false);
    esp_deep_sleep(MS_TO_US(10000));
}

TEST_CASE("rtc outputs off, no glitch on digital reset", MODULE_NAME)
{
    board.enable3V3(false);
    board.enableSQT(false);
    board.setEN(false);
    esp_restart_noos_dig();
}

TEST_CASE("rtc outputs on, no glitch on digital reset", MODULE_NAME)
{
    board.enable3V3(true);
    board.enableSQT(true);
    board.setEN(true);
    esp_restart_noos_dig();
}

extern "C" void determine_power_source()
{
    // No reset when removing external supply (usb/dc) with battery connected.
    board.enable3V3(true);

    #define LEDC_TIMER              LEDC_TIMER_0
    #define LEDC_MODE               LEDC_LOW_SPEED_MODE
    #define LEDC_OUTPUT_IO          (Board::Pin::FF::LED) // Define the output GPIO
    #define LEDC_CHANNEL            LEDC_CHANNEL_0
    #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
    #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
    #define LEDC_DUTY(x)            (((1 << 13 ) - 1) * x) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_FREQUENCY;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = LEDC_CHANNEL;
    ledc_channel.timer_sel      = LEDC_TIMER;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = LEDC_OUTPUT_IO;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    Board::PowerInput last = board.getPowerInput();
    bool first = true;

    while (true)
    {
        Board::PowerInput current = board.getPowerInput();
        if (current != last || first)
        {
            switch (current)
            {
            case Board::PowerInput::Battery:
                // Barely lit
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.001f));
                printf("battery\n");
                break;

            case Board::PowerInput::USB:
                // Faint
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01));
                printf("usb\n");
                break;

            case Board::PowerInput::DC:
                // Bright
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(1.0f));
                printf("dc\n");
                break;

            default:
                break;
            }

            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            last = current;
        }

        first = false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

TEST_CASE("determine power source", MODULE_NAME)
{
    determine_power_source();
}


static void periodic_timer_callback(void* arg)
{
    gpio_num_t pin = static_cast<gpio_num_t>((int)arg);
    gpio_set_level(pin, !gpio_get_level(pin));
}

void setup_pin(int pin)
{
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
    periodic_timer_args.callback = &periodic_timer_callback;
    periodic_timer_args.arg = reinterpret_cast<void*>(pin);

    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, (1000000 / (pin)) / 2);
}

TEST_CASE("digital pin connections", MODULE_NAME)
{
    // Output frequency same as pin number, i.e. GPIO1 output 1Hz,
    // GPIO2 output 2Hz and so on.
    gpio_num_t exclude[] = { GPIO_NUM_0, GPIO_NUM_3, GPIO_NUM_45, GPIO_NUM_46};
    int min = 36;
    int max = 40;

    for (int i = min; i < max + 1; i++)
    {
        bool excluded = false;
        for (int e : exclude)
        {
            if (i == e)
            {
                excluded = true;
                break;
            }
        }

        if (!excluded)
        {
            setup_pin(i);
        }
    }

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("3.3V regulator response to VSYS voltage", MODULE_NAME)
{
    // Check the response of the 3.3V switching regulator as VSYS is adjusted.
}

static bool charger_int = false;

static void charger_int_handler(void *arg)
{
    charger_int = true;
}

TEST_CASE("temperature sense", MODULE_NAME)
{
    // Tie potentiometer to temperature sense
    // Check interrupt, may be combined with another test
    board.getCharger().enableADC(true, BQ2562x::ADCRate::Continuous);

    gpio_set_intr_type(Board::Pin::FF::INT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(Board::Pin::FF::INT, charger_int_handler, &board);

    board.getCharger().enableTS(true);

    while (true)
    {
        if (charger_int)
        {
            printf("charger interrupt\n");
            charger_int = false;
        }
        printf("temperature: %f\n", board.getCharger().getBatteryTemperature());
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

TEST_CASE("charger status and flags", MODULE_NAME)
{
    printf("\nflag0: 0x%02x ", board.getCharger().getFlags(0));
    printf("flag1: 0x%02x ", board.getCharger().getFlags(1));
    printf("stat0: 0x%02x ", board.getCharger().getStat(0));
    printf("stat1: 0x%02x ", board.getCharger().getStat(1));
    printf("fault: 0x%02x\n\n", board.getCharger().getFault());
}

static void button_anyedge_handler(void *arg)
{
    gpio_set_level(Board::Pin::FF::LED, gpio_get_level(Board::Pin::FF::BTN));
}

TEST_CASE("button and led", MODULE_NAME)
{
    // Tie potentiometer to temperature sense
    // Check interrupt, may be combined with another test
    gpio_set_level(Board::Pin::FF::LED, true);

    gpio_set_intr_type(Board::Pin::FF::BTN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(Board::Pin::FF::BTN, button_anyedge_handler, NULL);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("charger i2c communicaton", MODULE_NAME)
{
    uint8_t partNum = 0, rev = 0;
    board.getCharger().readReg(BYTE(0x38), 0, 2, rev);
    board.getCharger().readReg(BYTE(0x38), 3, 5, partNum);
    TEST_ASSERT_EQUAL(0x02, partNum);
    TEST_ASSERT_EQUAL(0x02, rev);
}

TEST_CASE("fuel guage interrupt", MODULE_NAME)
{
    // Write a high temperature to register
}

TEST_CASE("discharging and charging", MODULE_NAME)
{
    // Measure VBAT, IBAT
    // Disable charging initially, until certain SOC
    // Enable charging, then disable again once another SOC is reached
    board.getCharger().enableADC(true, BQ2562x::ADCRate::Continuous);
    board.getCharger().enableCharging(true);
    board.getCharger().setChargeCurrent(50);

    while (true)
    {
        float vbat = board.getCharger().getVBAT();
        float ibat = board.getCharger().getIBAT();

        BQ2562x::ChargeStat stat = board.getCharger().getChargeStat();

        char *stat_str = NULL;

        switch (stat)
        {
        case BQ2562x::ChargeStat::Trickle:
            stat_str = "trickle";
            break;

        case BQ2562x::ChargeStat::Taper:
            stat_str = "taper";
            break;

        case BQ2562x::ChargeStat::TopOff:
            stat_str = "topoff";
            break;

        case BQ2562x::ChargeStat::Terminated:
        default:
            stat_str = "terminated";
            break;
        }

        printf("time: %ld\tstat: %s\tvbat: %.2f\tibat: %.2f\n", time(NULL), stat_str, vbat, ibat);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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

    printf("mode=%s-%s sip=%d.%d.%d.%d:%d, dip=%d.%d.%d.%d:%d, interval=%d, time=%d\n",
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
        printf("station "MACSTR" join, AID=%d\n", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        printf("station "MACSTR" leave, AID=%d\n", MAC2STR(event->mac), event->aid);
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
    // Run iperf server
    // 5V is loaded up to 2.5A
    // 3.3V is loaded up to 500mA
    // Measure ibus current
    board.getCharger().enableADC(true, BQ2562x::ADCRate::Continuous);

    start_ap();

    while (true)
    {
        if (!is_iperf_running())
        {
            start_iperf_server();
        }

        float ibus = board.getCharger().getIBUS();
        printf("ibus: %.2f\n", ibus);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void wait_for_battery(uint32_t delay_ms)
{
    Board::PowerInput input = board.getPowerInput();
    while (input != Board::PowerInput::Battery) {
        input = board.getPowerInput();
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

void test_mode(BQ2562x::BATFETControl control)
{
    wait_for_battery(1000);
    board.getCharger().setBATFETControl(control);
}

TEST_CASE("ship mode", MODULE_NAME)
{
    // Test ship mode can be entered
    // Measure ship mode current
    // Tie QON to reset, check that ship mode can be exited
    test_mode(BQ2562x::BATFETControl::ShipMode);
}

TEST_CASE("shutdown mode", MODULE_NAME)
{
    // Test shutdown mode can be entered
    // Measure shutdown mode current
    // Tie QON to reset, check that ship mode can be exited
    test_mode(BQ2562x::BATFETControl::ShutdownMode);
}

TEST_CASE("power cycle", MODULE_NAME)
{
    // Test shutdown mode can be entered
    // Measure shutdown mode current
    // Tie QON to reset, check that ship mode can be exited
    test_mode(BQ2562x::BATFETControl::SystemPowerReset);
}
