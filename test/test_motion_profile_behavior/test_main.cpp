#include <gtest/gtest.h>
#include "motion_profile.hpp"
#include "tmc2130_motor.hpp"
#include "spi_bridge.hpp"
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace logsys {
void printf(const char*, ...) {}
void set_transmitter(TransmitFn) {}
void init(UART_HandleTypeDef*) {}
void dump(const char*, const uint8_t*, size_t) {}
} // namespace logsys

using namespace motion;

namespace {
std::vector<std::array<uint8_t, 5>> recorded_frames;

bool datagram_hook(const uint8_t* tx, uint8_t* rx)
{
    std::array<uint8_t, 5> frame {};
    std::copy(tx, tx + 5, frame.begin());
    recorded_frames.push_back(frame);
    std::fill(rx, rx + 5, 0);
    return true;
}

int16_t decode_current(uint16_t raw)
{
    if (raw & 0x100u)
        return static_cast<int16_t>(raw) - 512;
    return static_cast<int16_t>(raw);
}

uint32_t frame_value(const std::array<uint8_t, 5>& frame)
{
    return (uint32_t(frame[1]) << 24) | (uint32_t(frame[2]) << 16)
        | (uint32_t(frame[3]) << 8) | uint32_t(frame[4]);
}

} // namespace

class MotionProfileTest : public ::testing::Test
{
protected:
    SPI_HandleTypeDef spi_ {};
    GPIO_TypeDef port_ {};
    SPI_Bridge bridge_{ &spi_, &port_, 1 };
    tmc2130::Motor motor_{ bridge_ };
    MotionProfile profile_{ motor_, nullptr };

    void SetUp() override
    {
        recorded_frames.clear();
        tmc2130::Motor::setDatagramHook(&datagram_hook);
        profile_.setAmplitude(200.f);
        profile_.setMicrostepResolution(256);
    }

    void TearDown() override
    {
        tmc2130::Motor::setDatagramHook(nullptr);
        recorded_frames.clear();
    }
};

TEST_F(MotionProfileTest, ConstantVelocityProducesWrites)
{
    ASSERT_TRUE(profile_.runConstantVelocity(0.05f, 800.f, Direction::Forward));
    ASSERT_FALSE(recorded_frames.empty());

    // Last write should be zeroed currents due to stop()
    uint32_t last_value = frame_value(recorded_frames.back());
    EXPECT_EQ(0u, last_value);

    // Currents should never exceed amplitude setting
    for (size_t i = 0; i + 1 < recorded_frames.size(); ++i) {
        uint32_t value = frame_value(recorded_frames[i]);
        int16_t cur_a = decode_current(value & 0x1FFu);
        int16_t cur_b = decode_current((value >> 16) & 0x1FFu);
        EXPECT_LE(std::abs(cur_a), 200);
        EXPECT_LE(std::abs(cur_b), 200);
    }
}

TEST_F(MotionProfileTest, ReverseSCurveChangesPhase)
{
    recorded_frames.clear();
    ASSERT_TRUE(profile_.runSCurve(0.04f, 500.f, Direction::Reverse, 10));
    ASSERT_FALSE(recorded_frames.empty());

    // Check that some currents are negative (reverse direction)
    bool has_negative = false;
    for (size_t i = 0; i + 1 < recorded_frames.size(); ++i) {
        uint32_t value = frame_value(recorded_frames[i]);
        int16_t cur_a = decode_current(value & 0x1FFu);
        int16_t cur_b = decode_current((value >> 16) & 0x1FFu);
        if (cur_a < 0 || cur_b < 0) {
            has_negative = true;
            break;
        }
    }
    EXPECT_TRUE(has_negative);
    EXPECT_EQ(0u, frame_value(recorded_frames.back()));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
