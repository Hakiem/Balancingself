#pragma once

#include "motion_types.hpp"
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace command {

enum class Type {
    None,
    Help,
    Status,
    Stop,
    SetAmplitude,
    SetMicrosteps,
    Scurve,
    Triangle,
    Trapezoid,
    Expo,
    Sine,
    ConstantVelocity
};

struct Command {
    Type type = Type::None;
    float duration = 0.f;
    float peak = 0.f;
    float accel = 0.f;
    float constant = 0.f;
    float decel = 0.f;
    float steep = 0.f;
    float amplitude = 0.f;
    uint32_t period_ms = 0;
    uint32_t microsteps = 0;
    motion::Direction direction = motion::Direction::Forward;
};

/**
 * @brief Parse command buffer into an actionable structure.
 * @param buffer Null-terminated buffer (can be modified in-place).
 * @return Parsed command; Command::type==None when unrecognised or invalid.
 */
inline Command parse(char* buffer)
{
    auto equals_ignore_case = [](const char* a, const char* b) -> bool {
        if (!a || !b)
            return false;
        while (*a && *b) {
            if (std::tolower(static_cast<unsigned char>(*a))
                != std::tolower(static_cast<unsigned char>(*b)))
                return false;
            ++a;
            ++b;
        }
        return *a == '\0' && *b == '\0';
    };

    auto parse_float_token = [](const char* token, float& out) -> bool {
        if (!token)
            return false;
        char* end = nullptr;
        float value = std::strtof(token, &end);
        if (end == token)
            return false;
        out = value;
        return true;
    };

    auto parse_u32_token = [](const char* token, uint32_t& out) -> bool {
        if (!token)
            return false;
        char* end = nullptr;
        unsigned long value = std::strtoul(token, &end, 10);
        if (end == token)
            return false;
        out = static_cast<uint32_t>(value);
        return true;
    };

    auto parse_direction_token = [&](const char* token, motion::Direction& out) -> bool {
        if (!token)
            return false;
        if (equals_ignore_case(token, "rev") || equals_ignore_case(token, "reverse")
            || equals_ignore_case(token, "r")) {
            out = motion::Direction::Reverse;
            return true;
        }
        if (equals_ignore_case(token, "fwd") || equals_ignore_case(token, "forward")
            || equals_ignore_case(token, "f")) {
            out = motion::Direction::Forward;
            return true;
        }
        return false;
    };

    Command cmd;
    if (!buffer)
        return cmd;

    char* token = std::strtok(buffer, " ");
    if (!token)
        return cmd;

    if (equals_ignore_case(token, "help")) {
        cmd.type = Type::Help;
        return cmd;
    }
    if (equals_ignore_case(token, "status")) {
        cmd.type = Type::Status;
        return cmd;
    }
    if (equals_ignore_case(token, "stop")) {
        cmd.type = Type::Stop;
        return cmd;
    }
    if (equals_ignore_case(token, "amplitude")) {
        float amp = 0.f;
        if (!parse_float_token(std::strtok(nullptr, " "), amp))
            return cmd;
        cmd.type = Type::SetAmplitude;
        cmd.amplitude = amp;
        return cmd;
    }
    if (equals_ignore_case(token, "microsteps")) {
        uint32_t steps = 0;
        if (!parse_u32_token(std::strtok(nullptr, " "), steps))
            return cmd;
        cmd.type = Type::SetMicrosteps;
        cmd.microsteps = steps;
        return cmd;
    }
    if (equals_ignore_case(token, "scurve")) {
        float duration = 0.f;
        float peak = 0.f;
        if (!parse_float_token(std::strtok(nullptr, " "), duration)
            || !parse_float_token(std::strtok(nullptr, " "), peak))
            return cmd;
        motion::Direction dir = motion::Direction::Forward;
        uint32_t period = 10;
        char* next = std::strtok(nullptr, " ");
        if (next) {
            motion::Direction tmp;
            if (parse_direction_token(next, tmp)) {
                dir = tmp;
                next = std::strtok(nullptr, " ");
            }
            if (next) {
                uint32_t tmp_period;
                if (parse_u32_token(next, tmp_period))
                    period = tmp_period;
            }
        }
        cmd.type = Type::Scurve;
        cmd.duration = duration;
        cmd.peak = peak;
        cmd.direction = dir;
        cmd.period_ms = period;
        return cmd;
    }
    if (equals_ignore_case(token, "triangle")) {
        float duration = 0.f;
        float peak = 0.f;
        if (!parse_float_token(std::strtok(nullptr, " "), duration)
            || !parse_float_token(std::strtok(nullptr, " "), peak))
            return cmd;
        motion::Direction dir = motion::Direction::Forward;
        uint32_t period = 10;
        char* next = std::strtok(nullptr, " ");
        if (next) {
            motion::Direction tmp;
            if (parse_direction_token(next, tmp)) {
                dir = tmp;
                next = std::strtok(nullptr, " ");
            }
            if (next) {
                uint32_t tmp_period;
                if (parse_u32_token(next, tmp_period))
                    period = tmp_period;
            }
        }
        cmd.type = Type::Triangle;
        cmd.duration = duration;
        cmd.peak = peak;
        cmd.direction = dir;
        cmd.period_ms = period;
        return cmd;
    }
    if (equals_ignore_case(token, "trapezoid")) {
        float accel = 0.f, constant = 0.f, decel = 0.f, target = 0.f;
        if (!parse_float_token(std::strtok(nullptr, " "), accel)
            || !parse_float_token(std::strtok(nullptr, " "), constant)
            || !parse_float_token(std::strtok(nullptr, " "), decel)
            || !parse_float_token(std::strtok(nullptr, " "), target))
            return cmd;
        motion::Direction dir = motion::Direction::Forward;
        uint32_t period = 10;
        char* next = std::strtok(nullptr, " ");
        if (next) {
            motion::Direction tmp;
            if (parse_direction_token(next, tmp)) {
                dir = tmp;
                next = std::strtok(nullptr, " ");
            }
            if (next) {
                uint32_t tmp_period;
                if (parse_u32_token(next, tmp_period))
                    period = tmp_period;
            }
        }
        cmd.type = Type::Trapezoid;
        cmd.accel = accel;
        cmd.constant = constant;
        cmd.decel = decel;
        cmd.peak = target;
        cmd.direction = dir;
        cmd.period_ms = period;
        return cmd;
    }
    if (equals_ignore_case(token, "expo")) {
        float duration = 0.f;
        float peak = 0.f;
        float steep = 4.0f;
        if (!parse_float_token(std::strtok(nullptr, " "), duration)
            || !parse_float_token(std::strtok(nullptr, " "), peak))
            return cmd;
        char* next = std::strtok(nullptr, " ");
        if (next) {
            parse_float_token(next, steep);
            next = std::strtok(nullptr, " ");
        }
        motion::Direction dir = motion::Direction::Forward;
        if (next) {
            motion::Direction tmp;
            if (parse_direction_token(next, tmp)) {
                dir = tmp;
                next = std::strtok(nullptr, " ");
            }
        }
        uint32_t period = 10;
        if (next) {
            uint32_t tmp_period;
            if (parse_u32_token(next, tmp_period))
                period = tmp_period;
        }
        cmd.type = Type::Expo;
        cmd.duration = duration;
        cmd.peak = peak;
        cmd.steep = steep;
        cmd.direction = dir;
        cmd.period_ms = period;
        return cmd;
    }
    if (equals_ignore_case(token, "sine")) {
        float duration = 0.f;
        float peak = 0.f;
        if (!parse_float_token(std::strtok(nullptr, " "), duration)
            || !parse_float_token(std::strtok(nullptr, " "), peak))
            return cmd;
        motion::Direction dir = motion::Direction::Forward;
        uint32_t period = 10;
        char* next = std::strtok(nullptr, " ");
        if (next) {
            motion::Direction tmp;
            if (parse_direction_token(next, tmp)) {
                dir = tmp;
                next = std::strtok(nullptr, " ");
            }
            if (next) {
                uint32_t tmp_period;
                if (parse_u32_token(next, tmp_period))
                    period = tmp_period;
            }
        }
        cmd.type = Type::Sine;
        cmd.duration = duration;
        cmd.peak = peak;
        cmd.direction = dir;
        cmd.period_ms = period;
        return cmd;
    }
    if (equals_ignore_case(token, "const")) {
        float duration = 0.f;
        float speed = 0.f;
        if (!parse_float_token(std::strtok(nullptr, " "), duration)
            || !parse_float_token(std::strtok(nullptr, " "), speed))
            return cmd;
        motion::Direction dir = motion::Direction::Forward;
        char* next = std::strtok(nullptr, " ");
        if (next) {
            motion::Direction tmp;
            if (parse_direction_token(next, tmp))
                dir = tmp;
        }
        cmd.type = Type::ConstantVelocity;
        cmd.duration = duration;
        cmd.peak = speed;
        cmd.direction = dir;
        return cmd;
    }

    return cmd;
}

} // namespace command
