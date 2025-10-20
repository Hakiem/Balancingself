#include <gtest/gtest.h>
#include "motion_math.hpp"

TEST(MotionMath, Encode9BitClamps)
{
    using motion::math::encode9bit;
    EXPECT_EQ(0u, encode9bit(0));
    EXPECT_EQ(255u, encode9bit(300));
    EXPECT_EQ(512u - 128u, encode9bit(-128));
}

TEST(MotionMath, PackCurrents)
{
    using motion::math::packCurrents;
    uint32_t packed = packCurrents(100, -50);
    EXPECT_EQ(100u, packed & 0x1FFu);
    EXPECT_EQ(((512u - 50u) & 0x1FFu), (packed >> 16) & 0x1FFu);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

