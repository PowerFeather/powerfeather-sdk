#include "test_utils.h"

#include <unity.h>

#include <MainBoard.h>
#include <BQ2562x.h>

extern "C" void app_main(void)
{
    TEST_ASSERT_EQUAL(PowerFeather::Board.init(500), PowerFeather::Result::Ok);
    // esp_deep_sleep(5000000000);
    test_main();
}