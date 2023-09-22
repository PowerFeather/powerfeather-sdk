#include "test_utils.h"

#include <Board.h>
#include <BQ2562x.h>

using namespace PowerFeather;

extern Board board;

extern "C" void app_main(void)
{
    board.init();
    board.getCharger().enableADC(false);

    // Test ship mode can be entered
    // Measure ship mode current
    // Tie QON to reset, check that ship mode can be exited
    board.getCharger().setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms);
    test_main();
}