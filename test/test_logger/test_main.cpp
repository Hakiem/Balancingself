#include <gtest/gtest.h>
#include "logger.hpp"
#include "logger.cpp"
#include <string>
#include <vector>

namespace {
std::vector<uint8_t> transmitted;

HAL_StatusTypeDef capture_transmit(
    UART_HandleTypeDef*, uint8_t* data, uint16_t len, uint32_t)
{
    transmitted.insert(transmitted.end(), data, data + len);
    return HAL_OK;
}

void reset_capture()
{
    transmitted.clear();
}
} // namespace

TEST(Logger, PrintsFormattedString)
{
    UART_HandleTypeDef uart;
    logsys::init(&uart);
    logsys::set_transmitter(&capture_transmit);
    reset_capture();
    logsys::printf("Value=%d", 42);
    std::string s(transmitted.begin(), transmitted.end());
    EXPECT_EQ("Value=42", s);
}

TEST(Logger, DumpsHex)
{
    UART_HandleTypeDef uart;
    logsys::init(&uart);
    logsys::set_transmitter(&capture_transmit);
    reset_capture();
    uint8_t data[3] = { 0xAA, 0x55, 0x10 };
    logsys::dump("TAG", data, 3);
    std::string s(transmitted.begin(), transmitted.end());
    EXPECT_NE(std::string::npos, s.find("TAG"));
    EXPECT_NE(std::string::npos, s.find("AA"));
    EXPECT_NE(std::string::npos, s.find("55"));
}

TEST(Logger, IgnoresWhenNotInitialised)
{
    logsys::set_transmitter(&capture_transmit);
    reset_capture();
    logsys::init(nullptr);
    logsys::printf("ShouldNotAppear");
    EXPECT_TRUE(transmitted.empty());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
