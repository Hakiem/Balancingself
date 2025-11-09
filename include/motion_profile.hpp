#pragma once

#include "motion_types.hpp"
#ifdef UNIT_TEST
#include "hal_stubs.hpp"
#else
#include "stm32f3xx_hal.h"
#endif
#include <cmath>
#include <cstdint>
#include <deque>

namespace motion {

class MotionProfile final
{
public:
    struct StepPins {
        GPIO_TypeDef* step_port = nullptr;
        uint16_t step_pin = 0;
        GPIO_TypeDef* dir_port = nullptr;
        uint16_t dir_pin = 0;
        uint32_t tim_channel = 0;
    };

    MotionProfile(const StepPins& pins, TIM_HandleTypeDef* timer = nullptr)
        : pins_(pins)
        , timer_(timer)
    {
    }

    void setMicrostepResolution(uint16_t microsteps)
    {
        if (microsteps == 0)
            microsteps = 256;
        microsteps_per_cycle_ = microsteps;
    }

    void setAmplitude(float amplitude)
    {
        if (amplitude < 0.f)
            amplitude = 0.f;
        if (amplitude > 255.f)
            amplitude = 255.f;
        amplitude_ = amplitude;
    }

    bool runConstantVelocity(
        float duration_s, float usteps_s, Direction dir = Direction::Forward)
    {
        if (duration_s <= 0.f && usteps_s <= 0.f)
            return false;

        uint32_t duration_ms = static_cast<uint32_t>(duration_s * 1000.f);
        if (duration_ms == 0u && usteps_s > 0.f)
            duration_ms = kDefaultIntervalMs;

        enqueueSegment(Segment { dir, usteps_s, duration_ms, false });
        return true;
    }

    bool runSCurve(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [](float t) {
                const float t2 = (t <= 0.5f) ? (t * 2.f) : ((1.f - t) * 2.f);
                return 0.5f * (1.f - std::cos(static_cast<float>(kPi) * t2));
            });
    }

    bool runTriangular(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [](float t) { return (t <= 0.5f) ? (t * 2.f) : (2.f - 2.f * t); });
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

        return true;
    }

    bool runExponential(float duration_s, float peak_usteps_s,
        float steepness = 4.0f, Direction dir = Direction::Forward,
        uint32_t update_period_ms = 10)
    {
        if (steepness <= 0.f)
            return false;
        const float denom = std::exp(steepness) - 1.0f;
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [steepness, denom](float t) {
                if (t <= 0.5f) {
                    const float x = t * 2.0f;
                    return (std::exp(steepness * x) - 1.0f) / denom * 0.5f;
                }
                const float x = (1.0f - t) * 2.0f;
                return 1.0f - (std::exp(steepness * x) - 1.0f) / denom * 0.5f;
            });
    }

    bool runSinusoidal(float duration_s, float peak_usteps_s,
        Direction dir = Direction::Forward, uint32_t update_period_ms = 10)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [](float t) { return std::sin(static_cast<float>(kPi) * t); });
    }

    bool stop()
    {
        queue_.clear();
        active_ = false;
        endless_running_ = false;
        fractional_steps_ = 0.f;
        steps_remaining_ = 0;
        current_velocity_ = 0.f;
        total_steps_ = 0;
        if (timer_)
            HAL_TIM_OC_Stop_IT(timer_, pins_.tim_channel);
        HAL_GPIO_WritePin(pins_.step_port, pins_.step_pin, GPIO_PIN_RESET);
        return true;
    }

    bool runForever(float usteps_s, Direction dir)
    {
        if (usteps_s <= 0.f)
            return false;
        enqueueSegment(Segment { dir, usteps_s, 0u, true });
        return true;
    }

    bool updateVelocity(float usteps_s, Direction dir)
    {
        if (usteps_s <= 0.f)
            return stop();

        queue_.clear();
        current_velocity_ = usteps_s;
        last_velocity_ = usteps_s;
        current_direction_ = dir;
        last_direction_ = dir;
        HAL_GPIO_WritePin(
            pins_.dir_port, pins_.dir_pin,
            (dir == Direction::Forward) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        interval_ticks_ = computeIntervalTicks(current_velocity_);
        if (interval_ticks_ == 0)
            interval_ticks_ = 1;

        if (!active_) {
            steps_remaining_ = 0;
            fractional_steps_ = 0.f;
            endless_running_ = true;
            if (!timer_started_ && timer_) {
                HAL_TIM_Base_Start(timer_);
                timer_started_ = true;
            }
            if (timer_) {
                uint32_t now = __HAL_TIM_GET_COUNTER(timer_);
                __HAL_TIM_SET_COMPARE(timer_, pins_.tim_channel, now + interval_ticks_);
                HAL_TIM_OC_Start_IT(timer_, pins_.tim_channel);
            }
            active_ = true;
            toggle_state_ = false;
            HAL_GPIO_WritePin(pins_.step_port, pins_.step_pin, GPIO_PIN_RESET);
            return true;
        }

        if (timer_) {
            uint32_t now = __HAL_TIM_GET_COUNTER(timer_);
            __HAL_TIM_SET_COMPARE(timer_, pins_.tim_channel, now + interval_ticks_);
        }
        return true;
    }

    void handleTimerEvent()
    {
        if (!active_ || current_velocity_ <= 0.f)
            return;

        toggle_state_ = !toggle_state_;
        HAL_GPIO_WritePin(pins_.step_port, pins_.step_pin,
            toggle_state_ ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (timer_) {
            const uint32_t current_compare
                = __HAL_TIM_GET_COMPARE(timer_, pins_.tim_channel);
            __HAL_TIM_SET_COMPARE(timer_, pins_.tim_channel, current_compare + interval_ticks_);
        }

        if (toggle_state_) {
            ++total_steps_;
            if (!endless_running_) {
                if (steps_remaining_ > 0) {
                    --steps_remaining_;
                    if (steps_remaining_ == 0)
                        advanceSegment();
                }
            }
        }
    }

    float lastVelocity() const { return last_velocity_; }
    Direction lastDirection() const { return last_direction_; }
    uint32_t lastStepCount() const { return total_steps_; }

private:
    struct Segment {
        Direction dir;
        float velocity_usteps_s;
        uint32_t duration_ms;
        bool endless;
    };

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
            enqueueSegment(Segment { dir, magnitude, update_period_ms, false });
        }
        const float consumed_time = static_cast<float>(segments * update_period_ms) / 1000.f;
        const float remaining_time = duration_s - consumed_time;
        if (remaining_time > 0.f) {
            const uint32_t remainder_ms
                = static_cast<uint32_t>(remaining_time * 1000.f);
            if (remainder_ms > 0u)
                enqueueSegment(Segment { dir, peak_usteps_s, remainder_ms, false });
        }

        return true;
    }
    0
    bool runRamp(float duration_s, float peak_usteps_s, Direction dir,
        uint32_t update_period_ms, bool accelerating)
    {
        return runProfile(duration_s, peak_usteps_s, dir, update_period_ms,
            [accelerating](float t) {
                return accelerating ? t : (1.0f - t);
            });
    }

    void enqueueSegment(const Segment& seg)
    {
        const bool reset_required = (!active_ && queue_.empty());
        queue_.push_back(seg);
        if (reset_required) {
            total_steps_ = 0;
            fractional_steps_ = 0.f;
            steps_remaining_ = 0;
        }
        if (!active_)
            advanceSegment();
    }

    void advanceSegment()
    {
        while (!queue_.empty()) {
            Segment seg = queue_.front();
            queue_.pop_front();
            if (seg.velocity_usteps_s <= 0.f) {
                continue;
            }

            current_direction_ = seg.dir;
            last_direction_ = seg.dir;
            HAL_GPIO_WritePin(
                pins_.dir_port, pins_.dir_pin,
                (seg.dir == Direction::Forward) ? GPIO_PIN_RESET : GPIO_PIN_SET);

            current_velocity_ = seg.velocity_usteps_s;
            last_velocity_ = current_velocity_;

            endless_running_ = seg.endless;
            if (!seg.endless) {
                double total_steps = (static_cast<double>(seg.duration_ms) / 1000.0)
                    * static_cast<double>(seg.velocity_usteps_s)
                    + fractional_steps_;
                steps_remaining_ = static_cast<uint32_t>(total_steps);
                fractional_steps_ = static_cast<float>(total_steps - steps_remaining_);
                if (steps_remaining_ == 0 && seg.velocity_usteps_s > 0.f) {
                    steps_remaining_ = 1;
                    fractional_steps_ = 0.f;
                }
            } else {
                steps_remaining_ = 0;
            }

            interval_ticks_ = computeIntervalTicks(current_velocity_);
            if (interval_ticks_ == 0)
                interval_ticks_ = 1;

            toggle_state_ = false;
            HAL_GPIO_WritePin(pins_.step_port, pins_.step_pin, GPIO_PIN_RESET);

            if (!timer_started_ && timer_) {
                HAL_TIM_Base_Start(timer_);
                timer_started_ = true;
            }

            if (timer_) {
                uint32_t now = __HAL_TIM_GET_COUNTER(timer_);
                __HAL_TIM_SET_COMPARE(timer_, pins_.tim_channel, now + interval_ticks_);
                HAL_TIM_OC_Start_IT(timer_, pins_.tim_channel);
            }
            active_ = true;
            return;
        }

        active_ = false;
        endless_running_ = false;
        current_velocity_ = 0.f;
        if (timer_)
            HAL_TIM_OC_Stop_IT(timer_, pins_.tim_channel);
        HAL_GPIO_WritePin(pins_.step_port, pins_.step_pin, GPIO_PIN_RESET);
    }

    static uint32_t computeIntervalTicks(float velocity)
    {
        if (velocity <= 0.f)
            return 0u;
        double toggle_interval = 500000.0 / static_cast<double>(velocity);
        if (toggle_interval < 1.0)
            toggle_interval = 1.0;
        if (toggle_interval > static_cast<double>(0xFFFFFFFFu))
            toggle_interval = static_cast<double>(0xFFFFFFFFu);
        return static_cast<uint32_t>(toggle_interval);
    }

    StepPins pins_;
    TIM_HandleTypeDef* timer_ = nullptr;
    bool timer_started_ = false;
    std::deque<Segment> queue_;
    bool active_ = false;
    bool endless_running_ = false;
    bool toggle_state_ = false;
    uint16_t microsteps_per_cycle_ = 256;
    float amplitude_ = 200.f;
    float last_velocity_ = 0.f;
    Direction last_direction_ = Direction::Forward;
    Direction current_direction_ = Direction::Forward;
    uint32_t total_steps_ = 0;
    uint32_t steps_remaining_ = 0;
    float fractional_steps_ = 0.f;
    float current_velocity_ = 0.f;
    uint32_t interval_ticks_ = 0u;

    static constexpr uint32_t kDefaultIntervalMs = 10;
    static constexpr double kPi = 3.14159265358979323846;
};

} // namespace motion
