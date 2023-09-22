#include "test_utils.h"

#include <Board.h>

extern PowerFeather::Board board;

extern "C" void app_main(void)
{
    board.init();
    board.getCharger().enableADC(false);
    test_main();
}