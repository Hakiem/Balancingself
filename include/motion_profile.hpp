#pragma once
#include "stm32f3xx_hal.h"
#include "tmc2208_hal_uart.hpp"
#include <cmath>
#include <cstdint>

// Header-only motion helper for TMC2208 using UART-only control (VACTUAL).
// Provides:
//   - Direction enum
//   - VACTUAL <-> microsteps/s conversion
//   - Smooth S-curve (cosine ease-in/ease-out) profile runner
//   - Simple triangular profile (optional)
// All functions return 'bool' to indicate whether UART writes succeeded.
namespace motion
{
enum class Direction : int { Reverse = -1, Forward = +1 };

class MotionProfile final
{
public:
    explicit MotionProfile(TMC2208::Device& drv, float fclk_hz = 12'000'000.0f)
        : drv_(drv)
        , fclk_hz_(fclk_hz)
    {
    }

    // Configure the internal/external clock value (Hz) used for VACTUAL
    // scaling.
    void setClockHz(float fclk_hz) noexcept { fclk_hz_ = fclk_hz; }
    float clockHz() const noexcept { return fclk_hz_; }

    // --- Conversions ---------------------------------------------------------

    // Convert desired microsteps per second -> VACTUAL (signed 24-bit).
    // VACTUAL LSB corresponds to fclk/2^24 µsteps/s.
    int32_t vactualFromUSteps(float usteps_per_s) const noexcept
    {
        const float lsb_hz = fclk_hz_ / float(1u << 24);
        float v = usteps_per_s / lsb_hz;

        // Clamp to signed 24-bit range ([-2^23, 2^23-1])
        constexpr float VMAX = 8'388'607.0f; // 2^23 - 1
        constexpr float VMIN = -8'388'608.0f; // -2^23
        if (v > VMAX)
            v = VMAX;
        if (v < VMIN)
            v = VMIN;

        // Round to nearest integer (symmetric)
        return static_cast<int32_t>(v >= 0.f ? (v + 0.5f) : (v - 0.5f));
    }

    // Convert VACTUAL back to microsteps per second.
    float ustepsFromVactual(int32_t vactual) const noexcept
    {
        const float lsb_hz = fclk_hz_ / float(1u << 24);
        return static_cast<float>(vactual) * lsb_hz;
    }

    // --- Profiles ------------------------------------------------------------

    // Smooth S-curve (cosine ease-in/out) velocity profile.
    // duration_s       : total duration of the profile (s)
    // peak_usteps_s    : peak speed magnitude (µsteps/s)
    // dir              : motion direction (Forward/Reverse)
    // update_period_ms : how often to update VACTUAL; lower is smoother (5–10
    // ms typical) Returns false if any UART write fails; otherwise true.
    bool runSCurve(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        if (duration_s <= 0.f || update_period_ms == 0u)
            return false;

        const uint32_t steps = static_cast<uint32_t>(
            (duration_s * 1000.0f) / static_cast<float>(update_period_ms));
        if (steps == 0u)
            return false;

        const float dir_sign = (dir == Direction::Forward) ? +1.f : -1.f;

        // Cosine-eased triangle on velocity magnitude: rise → fall
        for (uint32_t i = 0; i < steps; ++i) {
            const float x
                = static_cast<float>(i) / static_cast<float>(steps); // 0..1
            const float x2
                = (x <= 0.5f) ? (x * 2.f) : ((1.f - x) * 2.f); // 0..1..0
            const float v_mag
                = peak_usteps_s * 0.5f * (1.f - std::cos(3.14159265f * x2));
            const float v_signed = dir_sign * v_mag;

            const int32_t vact = vactualFromUSteps(v_signed);
            if (!drv_.vactual(vact))
                return false;

            HAL_Delay(update_period_ms);
        }

        // Clean stop
        (void)drv_.vactual(0);
        return true;
    }

    // Simple linear triangular profile (optional alternative).
    // Accelerates linearly to peak at t=duration/2, then linearly decelerates
    // to 0.
    bool runTriangular(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        if (duration_s <= 0.f || update_period_ms == 0u)
            return false;

        const uint32_t steps = static_cast<uint32_t>(
            (duration_s * 1000.0f) / static_cast<float>(update_period_ms));
        if (steps == 0u)
            return false;

        const float dir_sign = (dir == Direction::Forward) ? +1.f : -1.f;

        for (uint32_t i = 0; i < steps; ++i) {
            const float x
                = static_cast<float>(i) / static_cast<float>(steps); // 0..1
            const float tri
                = (x <= 0.5f) ? (x * 2.f) : (2.f - 2.f * x); // 0..1..0
            const float v_signed = dir_sign * (peak_usteps_s * tri);

            const int32_t vact = vactualFromUSteps(v_signed);
            if (!drv_.vactual(vact))
                return false;

            HAL_Delay(update_period_ms);
        }

        (void)drv_.vactual(0);
        return true;
    }

    // Immediate stop helper.
    bool stop() { return drv_.vactual(0); }

private:
    TMC2208::Device& drv_;
    float fclk_hz_; // Clock frequency used for VACTUAL scaling
};
} // namespace motion