#include <gtest/gtest.h>
#include "vs_common.h"

using namespace vs;

TEST(vs_geometry3d, checkRot)
{
    // check rot and so3 exp/log
    for(int i = 0; i < 1000; i++)
    {
        Eigen::Vector3d v(randf(-10, 10), randf(-10, 10), randf(-10, 10));
        Eigen::Matrix3d R = expSO3(v);
        EXPECT_TRUE(checkRot(R));

        Eigen::Vector3d v2 = logSO3(R);
        Eigen::Matrix3d R2 = expSO3(v2);
        EXPECT_LT(rotDiff(R, R2), 1e-5);
    }
}
