#pragma once

#include <cstdint>

namespace motion::math {

inline constexpr uint16_t encode9bit(int16_t value)
{
    if (value > 255)
        value = 255;
    if (value < -256)
        value = -256;
    return (value < 0) ? static_cast<uint16_t>(512 + value)
                       : static_cast<uint16_t>(value);
}

inline constexpr uint32_t packCurrents(int16_t cur_a, int16_t cur_b)
{
    const uint16_t raw_a = encode9bit(cur_a) & 0x1FFu;
    const uint16_t raw_b = encode9bit(cur_b) & 0x1FFu;
    return (static_cast<uint32_t>(raw_b) << 16)
        | static_cast<uint32_t>(raw_a);
}

} // namespace motion::math

