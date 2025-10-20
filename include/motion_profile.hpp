#pragma once

#include "motion_math.hpp"
#include "motion_types.hpp"
#ifdef UNIT_TEST
#include "hal_stubs.hpp"
#else
#include "stm32f3xx_hal.h"
#endif
#include "tmc2130_motor.hpp"
#include <cmath>
#include <cstdint>
#include <math.h>

namespace motion {

/**
 * @brief SPI-only motion helper that drives TMC2130 in direct mode (XDIRECT).
 *
 * The class synthesises sine/cosine coil currents for the requested velocity
 * profile and streams them to the driver via @ref tmc2130::Motor. No STEP/DIR
 * pins are used; all motion is produced through SPI writes to @ref Reg::XDIRECT.
 */
class MotionProfile final
{
public:
    /**
     * @brief Construct the profile helper.
     *
     * @param motor Reference to an initialised TMC2130 motor wrapper. Make sure
     *              GCONF.direct_mode is enabled before issuing movement.
     * @param timer Optional hardware timer providing microsecond timing for
     *              microstep updates. When nullptr the helper falls back to
     *              coarse millisecond delays.
     */
    explicit MotionProfile(
        tmc2130::Motor& motor, TIM_HandleTypeDef* timer = nullptr)
        : motor_(motor)
        , timer_(timer)
    {
        if (timer_) {
            HAL_TIM_Base_Start(timer_);
            timer_started_ = true;
        }
        recalcPhase();
    }

    /**
     * @brief Set the number of microsteps used to complete one electrical revolution.
     */
    void setMicrostepResolution(uint16_t microsteps)
    {
        if (microsteps == 0)
            microsteps = 256;
        microsteps_per_cycle_ = microsteps;
        if (microstep_index_ >= microsteps_per_cycle_)
            microstep_index_ %= microsteps_per_cycle_;
        recalcPhase();
    }

    /**
     * @brief Adjust the sine wave amplitude (0…255).
     */
    void setAmplitude(float amplitude)
    {
        if (amplitude < 0.f)
            amplitude = 0.f;
        if (amplitude > 255.f)
            amplitude = 255.f;
        amplitude_ = amplitude;
    }

    /**
     * @brief Access the underlying motor helper.
     */
    tmc2130::Motor& driver() { return motor_; }
    const tmc2130::Motor& driver() const { return motor_; }

    bool runSCurve(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [](float t) {
                const float t2 = (t <= 0.5f) ? (t * 2.f) : ((1.f - t) * 2.f);
                return 0.5f * (1.f - cosf(kPi * t2));
            });
    }

    bool runTriangular(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [](float t) {
                return (t <= 0.5f) ? (t * 2.f) : (2.f - 2.f * t);
            });
    }

    bool runTrapezoidal(float accel_time_s, float const_time_s,
        float decel_time_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        if (accel_time_s < 0.f || const_time_s < 0.f || decel_time_s < 0.f
            || update_period_ms == 0u)
            return false;

        if (!runRamp(accel_time_s, peak_usteps_s, dir, update_period_ms, true))
            return false;

        if (const_time_s > 0.f
            && !runProfile(const_time_s, peak_usteps_s, dir, update_period_ms,
                [](float) { return 1.f; }))
            return false;

        if (!runRamp(decel_time_s, peak_usteps_s, dir, update_period_ms, false))
            return false;

        return stop();
    }

    bool runExponential(float duration_s, float peak_usteps_s,
        float steepness = 4.0f, Direction dir = Direction::Forward,
        uint32_t update_period_ms = 10)
    {
        if (steepness <= 0.f)
            return false;
        const float denom = expf(steepness) - 1.0f;
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [steepness, denom](float t) {
                if (t <= 0.5f) {
                    const float x = t * 2.0f;
                    return (expf(steepness * x) - 1.0f) / denom * 0.5f;
                }
                const float x = (1.0f - t) * 2.0f;
                return 1.0f - (expf(steepness * x) - 1.0f) / denom * 0.5f;
            });
    }

    bool runConstantVelocity(
        float duration_s, float usteps_s, Direction dir = Direction::Forward)
    {
        if (duration_s <= 0.f)
            return false;

        constexpr uint32_t period_ms = 10;
        return runProfile(duration_s, usteps_s, dir, period_ms,
            [](float) { return 1.f; });
    }

    bool runSinusoidal(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [](float t) { return sinf(kPi * t); });
    }

    /**
     * @brief Immediately halt motion (no additional SPI writes).
     */
    bool stop()
    {
        fractional_steps_ = 0.f;
        return motor_.write(tmc2130::Reg::XDIRECT, 0);
    }

private:
    template <typename Curve>
    bool runProfile(float duration_s, float peak_usteps_s, Direction dir,
        uint32_t update_period_ms, Curve curve)
    {
        if (duration_s <= 0.f || update_period_ms == 0u)
            return false;

        const uint32_t segments
            = static_cast<uint32_t>((duration_s * 1000.0f) / update_period_ms);
        if (segments == 0u)
            return false;

        for (uint32_t i = 0; i < segments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(segments);
            float magnitude = peak_usteps_s * curve(t);
            if (magnitude < 0.f)
                magnitude = 0.f;
            if (!applyVelocity(dir, magnitude, update_period_ms))
                return false;
        }
        return stop();
    }

    bool runRamp(float duration_s, float peak_usteps_s, Direction dir,
        uint32_t update_period_ms, bool accelerating)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [accelerating](float t) {
                return accelerating ? t : (1.0f - t);
            });
    }

    bool applyVelocity(
        Direction dir, float usteps_per_s, uint32_t update_period_ms)
    {
        if (update_period_ms == 0u)
            return false;
        if (usteps_per_s <= 0.f) {
            fractional_steps_ = 0.f;
            return true;
        }

        const float interval_s
            = static_cast<float>(update_period_ms) * 0.001f;
        float desired_steps = usteps_per_s * interval_s + fractional_steps_;
        const uint32_t whole_steps = static_cast<uint32_t>(desired_steps);
        fractional_steps_ = desired_steps - static_cast<float>(whole_steps);

        return emitMicrosteps(dir, whole_steps, usteps_per_s);
    }

    bool emitMicrosteps(Direction dir, uint32_t steps, float usteps_per_s)
    {
        if (steps == 0)
            return true;

        const float period_us_f = 1'000'000.0f / usteps_per_s;
        uint32_t period_us = static_cast<uint32_t>(period_us_f);
        if (period_us == 0u)
            period_us = 1u;

        for (uint32_t i = 0; i < steps; ++i) {
            advanceIndex(dir);
            if (!writeDirectCurrents())
                return false;
            waitMicroseconds(period_us);
        }
        return true;
    }

    void advanceIndex(Direction dir)
    {
        const float sin_delta = sin_delta_;
        const float cos_delta = cos_delta_;

        if (dir == Direction::Forward) {
            microstep_index_ = (microstep_index_ + 1) % microsteps_per_cycle_;
            const float new_sin = sine_phase_ * cos_delta + cosine_phase_ * sin_delta;
            const float new_cos = cosine_phase_ * cos_delta - sine_phase_ * sin_delta;
            sine_phase_ = new_sin;
            cosine_phase_ = new_cos;
        } else {
            microstep_index_
                = (microstep_index_ == 0) ? (microsteps_per_cycle_ - 1)
                                          : (microstep_index_ - 1);
            const float new_sin = sine_phase_ * cos_delta - cosine_phase_ * sin_delta;
            const float new_cos = cosine_phase_ * cos_delta + sine_phase_ * sin_delta;
            sine_phase_ = new_sin;
            cosine_phase_ = new_cos;
        }

        if (++renorm_counter_ >= kRenormInterval) {
            const float mag = sine_phase_ * sine_phase_ + cosine_phase_ * cosine_phase_;
            if (mag > 0.0f) {
                const float inv = 1.0f / sqrtf(mag);
                sine_phase_ *= inv;
                cosine_phase_ *= inv;
            }
            renorm_counter_ = 0;
        }
    }

    bool writeDirectCurrents()
    {
        const int16_t cur_a = static_cast<int16_t>(
            lroundf(amplitude_ * sine_phase_));
        const int16_t cur_b = static_cast<int16_t>(
            lroundf(amplitude_ * cosine_phase_));

        const uint32_t packed = math::packCurrents(cur_a, cur_b);
        return motor_.write(tmc2130::Reg::XDIRECT, packed);
    }

    void waitMicroseconds(uint32_t us)
    {
        if (!timer_) {
            const uint32_t ms = (us + 999u) / 1000u;
            if (ms != 0u)
                HAL_Delay(ms);
            return;
        }

        if (!timer_started_) {
            if (HAL_TIM_Base_Start(timer_) == HAL_OK)
                timer_started_ = true;
        }

        __HAL_TIM_SET_COUNTER(timer_, 0);
        while (__HAL_TIM_GET_COUNTER(timer_) < us) { }
    }

    tmc2130::Motor& motor_;
    TIM_HandleTypeDef* timer_ = nullptr;
    bool timer_started_ = false;
    uint16_t microsteps_per_cycle_ = 256;
    uint16_t microstep_index_ = 0;
    float amplitude_ = 247.0f;
    float fractional_steps_ = 0.0f;
    float sine_phase_ = 0.0f;
    float cosine_phase_ = 1.0f;
    float sin_delta_ = 0.0f;
    float cos_delta_ = 1.0f;
    uint16_t renorm_counter_ = 0;

    static constexpr float kPi = 3.14159265f;
    static constexpr float kTwoPi = 6.28318531f;
    static constexpr uint16_t kRenormInterval = 64;

    void recalcPhase()
    {
        const float angle = (kTwoPi * static_cast<float>(microstep_index_))
            / static_cast<float>(microsteps_per_cycle_);
        sine_phase_ = sinf(angle);
        cosine_phase_ = cosf(angle);
        const float delta = kTwoPi / static_cast<float>(microsteps_per_cycle_);
        sin_delta_ = sinf(delta);
        cos_delta_ = cosf(delta);
        renorm_counter_ = 0;
    }
};

} // namespace motion
