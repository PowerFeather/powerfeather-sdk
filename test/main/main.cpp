#include "test_utils.h"

#include <unity.h>

#include <MainBoard.h>
#include <BQ2562x.h>

extern "C" void app_main(void)
{
    TEST_ASSERT_EQUAL(PowerFeather::Board.init(650), PowerFeather::Result::Ok);

    // printf("cell voltage: %u\n", board.getFuelGauge().getCellVoltage());

    // uint8_t chargerInfo = 0;
    // board.getCharger().getPartInformation(chargerInfo);
    // printf("charger: %u\n", chargerInfo);

    // printf("done!\n");

    test_main();
}