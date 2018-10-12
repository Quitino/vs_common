#include <gtest/gtest.h>
#include "vs_common.h"

using namespace vs;

TEST(vs_vecutils, findKth)
{
    std::vector<int> a = { 1, 6, 8, 4, 9, 5, 2, 3, 7, 0 };
    EXPECT_EQ(findKth(a, 3), 3);
    EXPECT_EQ(findKth(a, 8), 8);
    EXPECT_EQ(findKth(a, 0), 0);
    EXPECT_EQ(findKth(a, 9), 9);
    EXPECT_EQ(findKth(a, -1), 0);
    EXPECT_EQ(findKth(a, 11), 9);
}