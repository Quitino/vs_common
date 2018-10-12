#include <gtest/gtest.h>
#include "vs_common.h"

using namespace vs;

TEST(vs_perf, FpsCalculator)
{
    FpsCalculator fc;
    for(int i = 0; i < 30; i++)
    {
        fc.start();
        usleep(5e3);
        fc.stop();
    }
    EXPECT_NEAR(fc.fps(), 200, 10);
}