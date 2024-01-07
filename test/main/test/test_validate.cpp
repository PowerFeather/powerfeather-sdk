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
TEST_CASE("battery health", MODULE_NAME)
{
    uint8_t percent;

    while (true)
    {
        TEST_ASSERT_EQUAL(Result::Ok, board.getBatteryHealth(percent));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

