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

TEST(CommandParser, ParsesConstVelocity)
{
    char buf[] = "const 3.0 1500 rev";
    command::Command cmd = command::parse(buf);
    EXPECT_EQ(command::Type::ConstantVelocity, cmd.type);
    EXPECT_NEAR(3.0f, cmd.duration, 0.001f);
    EXPECT_NEAR(1500.f, cmd.peak, 0.001f);
    EXPECT_EQ(motion::Direction::Reverse, cmd.direction);
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
