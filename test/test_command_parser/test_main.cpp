#include <gtest/gtest.h>
#include "command_processor.hpp"

TEST(CommandParser, ParsesHelp)
{
    char buf[] = "help";
    command::Command cmd = command::parse(buf);
    EXPECT_EQ(command::Type::Help, cmd.type);
}

TEST(CommandParser, ParsesAmplitude)
{
    char buf[] = "amplitude 180.5";
    command::Command cmd = command::parse(buf);
    EXPECT_EQ(command::Type::SetAmplitude, cmd.type);
    EXPECT_NEAR(180.5f, cmd.amplitude, 0.001f);
}

TEST(CommandParser, ParsesScurveWithOptions)
{
    char buf[] = "scurve 2.5 1200 rev 15";
    command::Command cmd = command::parse(buf);
    EXPECT_EQ(command::Type::Scurve, cmd.type);
    EXPECT_NEAR(2.5f, cmd.duration, 0.001f);
    EXPECT_NEAR(1200.f, cmd.peak, 0.001f);
    EXPECT_EQ(motion::Direction::Reverse, cmd.direction);
    EXPECT_EQ(15u, cmd.period_ms);
}

TEST(CommandParser, UnknownFallback)
{
    char buf[] = "bogus 123";
    command::Command cmd = command::parse(buf);
    EXPECT_EQ(command::Type::None, cmd.type);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
