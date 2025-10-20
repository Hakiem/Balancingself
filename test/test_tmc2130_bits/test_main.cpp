#include <gtest/gtest.h>
#include "tmc2130_bits.hpp"

using namespace tmc2130::detail;

TEST(TMC2130Bits, EncodeIHOLDIRUN)
{
    uint32_t value = encodeIHOLDIRUN(10, 20, 5);
    EXPECT_EQ((10u) | (20u << 8) | (5u << 16), value);
    EXPECT_EQ((31u) | (31u << 8) | (15u << 16), encodeIHOLDIRUN(99, 99, 99));
}

TEST(TMC2130Bits, MakeGconf)
{
    uint32_t bits = makeGconf(true, true, false, true, false, true, true);
    EXPECT_TRUE(bits & (1u << 2));
    EXPECT_TRUE(bits & (1u << 1));
    EXPECT_TRUE(bits & (1u << 5));
    EXPECT_TRUE(bits & (1u << 8));
    EXPECT_TRUE(bits & (1u << 16));
    EXPECT_FALSE(bits & (1u << 4));
}

TEST(TMC2130Bits, MakeChopconf)
{
    uint32_t bits = makeChopconf(20, 5, 9, 4, true, false, true, true, false, 256);
    EXPECT_EQ(15u, bits & 0xF); // toff saturated
    EXPECT_EQ(5u + 3u, (bits >> 7) & 0xF);
    EXPECT_TRUE(bits & (1u << 28));
    EXPECT_TRUE(bits & (1u << 29));
    EXPECT_TRUE(bits & (1u << 17));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

