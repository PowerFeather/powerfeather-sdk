#include "test_utils.h"

#include <Board.h>
#include <BQ2562x.h>

using namespace PowerFeather;

extern Board board;

extern "C" void app_main(void)
{
    board.init();

    uint8_t chargerInfo = 0;
    board.getCharger().getPartInformation(chargerInfo);
    printf("charger: %u\n", chargerInfo);
}