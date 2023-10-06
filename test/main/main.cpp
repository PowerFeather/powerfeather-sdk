#include "test_utils.h"

#include <MainBoard.h>
#include <BQ2562x.h>

extern "C" void app_main(void)
{
    PowerFeather::Board.init(650);

    // printf("cell voltage: %u\n", board.getFuelGauge().getCellVoltage());

    // uint8_t chargerInfo = 0;
    // board.getCharger().getPartInformation(chargerInfo);
    // printf("charger: %u\n", chargerInfo);

    // printf("done!\n");

    test_main();
}