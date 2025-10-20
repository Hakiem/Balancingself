#pragma once

#include <cstdint>

namespace tmc2130::detail {

inline constexpr uint32_t encodeIHOLDIRUN(uint8_t ihold, uint8_t irun, uint8_t ihold_delay)
{
    if (ihold > 31)
        ihold = 31;
    if (irun > 31)
        irun = 31;
    if (ihold_delay > 15)
        ihold_delay = 15;

    return (uint32_t(ihold) << 0) | (uint32_t(irun) << 8)
        | (uint32_t(ihold_delay) << 16);
}

inline constexpr uint32_t encodeHEND(int8_t hend)
{
    if (hend < -3)
        hend = -3;
    if (hend > 12)
        hend = 12;
    return uint32_t(static_cast<uint8_t>(hend + 3)) << 7;
}

inline constexpr uint32_t encodeMRES(uint16_t microsteps)
{
    switch (microsteps) {
    case 1:
        return 0x8u << 24;
    case 2:
        return 0x7u << 24;
    case 4:
        return 0x6u << 24;
    case 8:
        return 0x5u << 24;
    case 16:
        return 0x4u << 24;
    case 32:
        return 0x3u << 24;
    case 64:
        return 0x2u << 24;
    case 128:
        return 0x1u << 24;
    case 256:
    default:
        return 0x0u << 24;
    }
}

inline constexpr uint32_t makeChopconf(uint8_t toff, int8_t hend, uint8_t hstrt,
    uint8_t blank_time, bool high_vsense, bool use_constant_off_time,
    bool enable_interpolation, bool double_edge_step, bool disable_s2g_protection,
    uint16_t microsteps)
{
    if (toff > 15)
        toff = 15;
    if (hstrt > 7)
        hstrt = 7;
    if (blank_time > 3)
        blank_time = 3;

    uint32_t v = uint32_t(toff);
    v |= encodeHEND(hend);
    v |= uint32_t(hstrt) << 4;
    v |= uint32_t(blank_time) << 15;
    if (high_vsense)
        v |= 1u << 17;
    if (use_constant_off_time)
        v |= 1u << 14;
    if (enable_interpolation)
        v |= 1u << 28;
    if (double_edge_step)
        v |= 1u << 29;
    if (disable_s2g_protection)
        v |= 1u << 30;
    v |= encodeMRES(microsteps);
    return v;
}

inline constexpr uint32_t makeGconf(bool enable_stealthchop, bool use_internal_rsense,
    bool invert_direction, bool diag0_on_error, bool diag0_on_otpw,
    bool diag1_on_stall, bool direct_mode)
{
    uint32_t v = 0;
    if (enable_stealthchop)
        v |= 1u << 2;
    if (use_internal_rsense)
        v |= 1u << 1;
    if (invert_direction)
        v |= 1u << 4;
    if (diag0_on_error)
        v |= 1u << 5;
    if (diag0_on_otpw)
        v |= 1u << 6;
    if (diag1_on_stall)
        v |= 1u << 8;
    if (direct_mode)
        v |= 1u << 16;
    return v;
}

inline constexpr uint32_t makePwmconf(uint8_t pwm_ampl, uint8_t pwm_grad,
    uint8_t pwm_freq, bool pwm_autoscale, bool pwm_symmetric, uint8_t pwm_freewheel)
{
    if (pwm_freq > 3)
        pwm_freq = 3;
    if (pwm_freewheel > 3)
        pwm_freewheel = 3;

    uint32_t v = uint32_t(pwm_ampl);
    v |= uint32_t(pwm_grad) << 8;
    v |= uint32_t(pwm_freq) << 16;
    if (pwm_autoscale)
        v |= 1u << 18;
    if (pwm_symmetric)
        v |= 1u << 19;
    v |= uint32_t(pwm_freewheel) << 20;
    return v;
}

} // namespace tmc2130::detail
