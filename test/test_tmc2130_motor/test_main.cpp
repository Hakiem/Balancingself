#include <gtest/gtest.h>
#include "tmc2130_motor.hpp"
#include "spi_bridge.hpp"
#include <array>
#include <deque>
#include <vector>
#include <algorithm>

namespace logsys {
void printf(const char*, ...) {}
void set_transmitter(TransmitFn) {}
void init(UART_HandleTypeDef*) {}
void dump(const char*, const uint8_t*, size_t) {}
} // namespace logsys

using namespace tmc2130;

namespace {
std::vector<std::array<uint8_t, 5>> recorded_frames;
std::deque<std::array<uint8_t, 5>> response_queue;
int datagram_calls = 0;

bool datagram_hook(const uint8_t* tx, uint8_t* rx)
{
    std::array<uint8_t, 5> frame {};
    std::copy(tx, tx + 5, frame.begin());
    recorded_frames.push_back(frame);
    ++datagram_calls;

    std::array<uint8_t, 5> resp {};
    if (!response_queue.empty()) {
        resp = response_queue.front();
        response_queue.pop_front();
    }
    std::copy(resp.begin(), resp.end(), rx);
    return true;
}

uint32_t frame_value(const std::array<uint8_t, 5>& frame)
{
    return (uint32_t(frame[1]) << 24) | (uint32_t(frame[2]) << 16)
        | (uint32_t(frame[3]) << 8) | uint32_t(frame[4]);
}
} // namespace

class MotorTest : public ::testing::Test
{
protected:
    SPI_HandleTypeDef spi_ {};
    GPIO_TypeDef port_ {};
    SPI_Bridge bridge_{ &spi_, &port_, 1 };
    Motor motor_{ bridge_ };

    void SetUp() override
    {
        recorded_frames.clear();
        response_queue.clear();
        datagram_calls = 0;
        Motor::setDatagramHook(&datagram_hook);
    }

    void TearDown() override
    {
        Motor::setDatagramHook(nullptr);
        recorded_frames.clear();
        response_queue.clear();
        datagram_calls = 0;
    }
};

TEST_F(MotorTest, InitializeEmitsExpectedWrites)
{
    Motor::Config cfg;
    cfg.write_tpwmthrs = true;
    cfg.tpwmthrs = 0x00ABCDEF;
    cfg.write_tcoolthrs = true;
    cfg.tcoolthrs = 0x00012345;
    cfg.write_thigh = true;
    cfg.thigh = 0x00000010;
    cfg.write_coolconf = true;
    cfg.coolconf = 0xDEADBEEF;
    cfg.write_dcctrl = true;
    cfg.dcctrl = 0x0000F00D;

    ASSERT_TRUE(motor_.initialize(cfg));

    // Expect writes for: GCONF, IHOLD_IRUN, TPOWERDOWN, TPWMTHRS, TCOOLTHRS,
    // THIGH, CHOPCONF, PWMCONF, COOLCONF, DCCTRL
    ASSERT_EQ(10u, recorded_frames.size());

    auto reg = [](const std::array<uint8_t, 5>& frame) { return frame[0] & 0x7F; };

    EXPECT_EQ(static_cast<uint8_t>(Reg::GCONF), reg(recorded_frames[0]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::IHOLD_IRUN), reg(recorded_frames[1]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::TPOWERDOWN), reg(recorded_frames[2]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::TPWMTHRS), reg(recorded_frames[3]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::TCOOLTHRS), reg(recorded_frames[4]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::THIGH), reg(recorded_frames[5]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::CHOPCONF), reg(recorded_frames[6]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::PWMCONF), reg(recorded_frames[7]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::COOLCONF), reg(recorded_frames[8]));
    EXPECT_EQ(static_cast<uint8_t>(Reg::DCCTRL), reg(recorded_frames[9]));

    uint32_t gconf_expected = detail::makeGconf(cfg.enable_stealthchop,
        cfg.use_internal_rsense, cfg.invert_direction, cfg.diag0_on_error,
        cfg.diag0_on_otpw, cfg.diag1_on_stall, cfg.direct_mode);
    EXPECT_EQ(gconf_expected, frame_value(recorded_frames[0]));

    uint32_t ihold_expected = detail::encodeIHOLDIRUN(cfg.ihold, cfg.irun, cfg.ihold_delay);
    EXPECT_EQ(ihold_expected, frame_value(recorded_frames[1]));

    EXPECT_EQ(cfg.tpowerdown, frame_value(recorded_frames[2]) & 0xFFFFu);
    EXPECT_EQ(cfg.tpwmthrs, frame_value(recorded_frames[3]) & 0x00FFFFFFu);
    EXPECT_EQ(cfg.tcoolthrs, frame_value(recorded_frames[4]) & 0x00FFFFFFu);
    EXPECT_EQ(cfg.thigh, frame_value(recorded_frames[5]) & 0x00FFFFFFu);

    uint32_t chop_expected = detail::makeChopconf(cfg.toff, cfg.hend, cfg.hstrt,
        cfg.blank_time, cfg.high_vsense, cfg.use_constant_off_time,
        cfg.enable_interpolation, cfg.double_edge_step,
        cfg.disable_s2g_protection, cfg.microsteps);
    EXPECT_EQ(chop_expected, frame_value(recorded_frames[6]));

    uint32_t pwm_expected = detail::makePwmconf(cfg.pwm_ampl, cfg.pwm_grad,
        cfg.pwm_freq, cfg.pwm_autoscale, cfg.pwm_symmetric, cfg.pwm_freewheel);
    EXPECT_EQ(pwm_expected, frame_value(recorded_frames[7]));

    EXPECT_EQ(cfg.coolconf, frame_value(recorded_frames[8]));
    EXPECT_EQ(cfg.dcctrl, frame_value(recorded_frames[9]) & 0x00FFFFFFu);
}

TEST_F(MotorTest, ReadReturnsQueuedValue)
{
    // Prime response queue: first dummy read, then actual data
    response_queue.push_back({ 0, 0, 0, 0, 0 });
    response_queue.push_back({ 0x80, 0x12, 0x34, 0x56, 0x78 });

    Motor::setDatagramHook(&datagram_hook);
    ASSERT_TRUE(Motor::hasDatagramHook());

    uint32_t value = 0;
    ASSERT_TRUE(motor_.read(Reg::DRV_STATUS, value));
    EXPECT_EQ(0x12345678u, value);
    EXPECT_EQ(2, datagram_calls);
    EXPECT_EQ(2u, recorded_frames.size());
    EXPECT_EQ(0u, response_queue.size());
    if (recorded_frames.size() == 2u) {
        EXPECT_EQ(static_cast<uint8_t>(Reg::DRV_STATUS), recorded_frames[0][0] & 0x7F);
        EXPECT_EQ(static_cast<uint8_t>(Reg::DRV_STATUS), recorded_frames[1][0] & 0x7F);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
