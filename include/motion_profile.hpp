#pragma once
#include "stm32f3xx_hal.h"
#include "tmc2208_hal_uart.hpp"
#include <cmath>
#include <cstdint>

// Header-only motion helper for TMC2209 using UART-only control (VACTUAL).
// Provides:
//   - Direction enum
//   - VACTUAL <-> microsteps/s conversion
//   - Smooth S-curve (cosine ease-in/ease-out) profile runner
//   - Triangular profile (linear acceleration/deceleration)
//   - Trapezoidal profile (accel → constant → decel)
//   - Exponential profile (natural acceleration curve)
//   - Sinusoidal profile (pure sine wave - very smooth)
//   - Constant velocity profile (rectangular)
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

    // Trapezoidal profile: accel → constant velocity → decel
    // accel_time_s     : time to reach peak speed (s)
    // const_time_s     : time at constant peak speed (s)
    // decel_time_s     : time to decelerate from peak (s)
    // peak_usteps_s    : peak speed magnitude (µsteps/s)
    // dir              : motion direction
    // update_period_ms : update rate
    bool runTrapezoidal(float accel_time_s, float const_time_s,
        float decel_time_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        if (accel_time_s <= 0.f || const_time_s < 0.f || decel_time_s <= 0.f
            || update_period_ms == 0u)
            return false;

        const float dir_sign = (dir == Direction::Forward) ? +1.f : -1.f;

        // Phase 1: Linear acceleration
        const uint32_t accel_steps = static_cast<uint32_t>(
            (accel_time_s * 1000.0f) / static_cast<float>(update_period_ms));

        for (uint32_t i = 0; i < accel_steps; ++i) {
            const float progress
                = static_cast<float>(i) / static_cast<float>(accel_steps);
            const float v_signed = dir_sign * (peak_usteps_s * progress);

            const int32_t vact = vactualFromUSteps(v_signed);
            if (!drv_.vactual(vact))
                return false;
            HAL_Delay(update_period_ms);
        }

        // Phase 2: Constant velocity
        if (const_time_s > 0.f) {
            const uint32_t const_steps
                = static_cast<uint32_t>((const_time_s * 1000.0f)
                    / static_cast<float>(update_period_ms));
            const int32_t const_vact
                = vactualFromUSteps(dir_sign * peak_usteps_s);

            for (uint32_t i = 0; i < const_steps; ++i) {
                if (!drv_.vactual(const_vact))
                    return false;
                HAL_Delay(update_period_ms);
            }
        }

        // Phase 3: Linear deceleration
        const uint32_t decel_steps = static_cast<uint32_t>(
            (decel_time_s * 1000.0f) / static_cast<float>(update_period_ms));

        for (uint32_t i = 0; i < decel_steps; ++i) {
            const float progress = 1.0f
                - (static_cast<float>(i) / static_cast<float>(decel_steps));
            const float v_signed = dir_sign * (peak_usteps_s * progress);

            const int32_t vact = vactualFromUSteps(v_signed);
            if (!drv_.vactual(vact))
                return false;
            HAL_Delay(update_period_ms);
        }

        // Clean stop
        (void)drv_.vactual(0);
        return true;
    }

    // Exponential profile: natural acceleration curve
    // duration_s       : total duration of the profile (s)
    // peak_usteps_s    : peak speed magnitude (µsteps/s)
    // steepness        : curve steepness (2.0 = gentle, 6.0 = sharp)
    // dir              : motion direction
    // update_period_ms : update rate
    bool runExponential(float duration_s, float peak_usteps_s,
        float steepness = 4.0f, Direction dir = Direction::Forward,
        uint32_t update_period_ms = 10)
    {
        if (duration_s <= 0.f || steepness <= 0.f || update_period_ms == 0u)
            return false;

        const uint32_t steps = static_cast<uint32_t>(
            (duration_s * 1000.0f) / static_cast<float>(update_period_ms));
        if (steps == 0u)
            return false;

        const float dir_sign = (dir == Direction::Forward) ? +1.f : -1.f;

        for (uint32_t i = 0; i < steps; ++i) {
            const float x
                = static_cast<float>(i) / static_cast<float>(steps); // 0..1

            // Exponential ease-in-out curve
            float exp_curve;
            if (x <= 0.5f) {
                // Ease in: exponential acceleration
                const float t = x * 2.0f; // 0..1
                exp_curve = (std::exp(steepness * t) - 1.0f)
                    / (std::exp(steepness) - 1.0f);
                exp_curve *= 0.5f; // Scale to 0..0.5
            } else {
                // Ease out: exponential deceleration
                const float t = (1.0f - x) * 2.0f; // 1..0
                exp_curve = (std::exp(steepness * t) - 1.0f)
                    / (std::exp(steepness) - 1.0f);
                exp_curve
                    = 1.0f - (exp_curve * 0.5f); // Scale to 0.5..1, then invert
            }

            const float v_signed = dir_sign * (peak_usteps_s * exp_curve);
            const int32_t vact = vactualFromUSteps(v_signed);
            if (!drv_.vactual(vact))
                return false;

            HAL_Delay(update_period_ms);
        }

        // Clean stop
        (void)drv_.vactual(0);
        return true;
    }

    // Constant velocity profile: immediate jump to speed, hold, then stop
    // duration_s       : time at constant speed (s)
    // usteps_s         : constant speed (µsteps/s)
    // dir              : motion direction
    bool runConstantVelocity(
        float duration_s, float usteps_s, Direction dir = Direction::Forward)
    {
        if (duration_s <= 0.f)
            return false;

        const float dir_sign = (dir == Direction::Forward) ? +1.f : -1.f;
        const int32_t vact = vactualFromUSteps(dir_sign * usteps_s);

        // Set velocity immediately
        if (!drv_.vactual(vact))
            return false;

        // Hold for duration
        HAL_Delay(static_cast<uint32_t>(duration_s * 1000.0f));

        // Stop
        return drv_.vactual(0);
    }

    // Sinusoidal profile: pure sine wave acceleration (very smooth)
    // duration_s       : total duration of the profile (s)
    // peak_usteps_s    : peak speed magnitude (µsteps/s)
    // dir              : motion direction
    // update_period_ms : update rate
    bool runSinusoidal(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        if (duration_s <= 0.f || update_period_ms == 0u)
            return false;

        const uint32_t steps = static_cast<uint32_t>(
            (duration_s * 1000.0f) / static_cast<float>(update_period_ms));
        if (steps == 0u)
            return false;

        const float dir_sign = (dir == Direction::Forward) ? +1.f : -1.f;
        const float pi = 3.14159265f;

        // Pure sine wave profile: 0 → peak → 0
        for (uint32_t i = 0; i < steps; ++i) {
            const float x
                = static_cast<float>(i) / static_cast<float>(steps); // 0..1
            const float sine_value
                = std::sin(pi * x); // 0..1..0 (sine wave from 0 to π)
            const float v_signed = dir_sign * (peak_usteps_s * sine_value);

            const int32_t vact = vactualFromUSteps(v_signed);
            if (!drv_.vactual(vact))
                return false;

            HAL_Delay(update_period_ms);
        }

        // Clean stop
        (void)drv_.vactual(0);
        return true;
    }

private:
    TMC2208::Device& drv_;
    float fclk_hz_; // Clock frequency used for VACTUAL scaling
};
} // namespace motion