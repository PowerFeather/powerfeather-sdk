#include "test_utils.h"

#include <Board.h>

extern PowerFeather::Board board;

extern "C" void app_main(void)
{
    board.init();
    test_main();
}