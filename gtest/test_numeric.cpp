#include <gtest/gtest.h>
#include "vs_common.h"

using namespace vs;

TEST(vs_numeric, fequal)
{
    EXPECT_TRUE(fequal(0.01, 0, 0.1));
    EXPECT_FALSE(fequal(0.01, 0));
    EXPECT_TRUE(fequal(0, 0));
}

TEST(vs_numeric, sign)
{
    EXPECT_EQ(sign(10), 1);
    EXPECT_EQ(sign(0), 1);
    EXPECT_EQ(sign(1), 1);
    EXPECT_EQ(sign(INT_MAX), 1);
    EXPECT_EQ(sign(-1), -1);
    EXPECT_EQ(sign(-10), -1);
}

TEST(vs_numeric, inRange)
{
    EXPECT_TRUE(inRange(0,0,0));
    EXPECT_TRUE(inRange(1,1,1));
    EXPECT_TRUE(inRange(1,0,1));
    EXPECT_TRUE(inRange(0,0,1));
    EXPECT_TRUE(inRange(0.99,0,1));
    EXPECT_FALSE(inRange(1.01,0,1));
    EXPECT_FALSE(inRange(0.99,1,0));
    EXPECT_FALSE(inRange(0,1,0));
}
