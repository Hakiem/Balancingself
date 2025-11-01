#include <gtest/gtest.h>
#include "motion_profile.hpp"
#include "logger.hpp"

namespace logsys {
void printf(const char*, ...) {}
void set_transmitter(TransmitFn) {}
void init(UART_HandleTypeDef*) {}
void dump(const char*, const uint8_t*, size_t) {}
} // namespace logsys

using namespace motion;

class MotionProfileTest : public ::testing::Test
{
protected:
    GPIO_TypeDef port_a_ {};
    MotionProfile::StepPins pins_{ &port_a_, 1, &port_a_, 2, TIM_CHANNEL_1 };
    MotionProfile profile_{ pins_, nullptr };

    void SetUp() override
    {
        profile_.setAmplitude(200.f);
        profile_.setMicrostepResolution(256);
    }
};

TEST_F(MotionProfileTest, ConstantVelocityGeneratesSteps)
{
    EXPECT_TRUE(profile_.runConstantVelocity(0.05f, 800.f, Direction::Forward));
    profile_.handleTimerEvent();
    profile_.handleTimerEvent();
    EXPECT_GT(profile_.lastStepCount(), 0u);
    EXPECT_EQ(profile_.lastDirection(), Direction::Forward);
}

TEST_F(MotionProfileTest, ReverseSCurveRecordsDirection)
{
    EXPECT_TRUE(profile_.runSCurve(0.04f, 500.f, Direction::Reverse, 10));
    profile_.handleTimerEvent();
    profile_.handleTimerEvent();
    EXPECT_GT(profile_.lastStepCount(), 0u);
    EXPECT_EQ(profile_.lastDirection(), Direction::Reverse);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
