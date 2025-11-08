#pragma once

#include "spi_bridge.hpp"
#include "tmc5160_bits.hpp"
#include "tmc5160_registers.hpp"
#include <cstdint>

namespace tmc5160 {

class Motor
{
public:
    enum class Direction { Forward, Reverse };

    struct Config {
        bool write_gconf = true;
        bool write_ihold_irun = true;
        bool write_chopconf = true;
        bool write_pwmconf = true;
        bool write_tpowerdown = true;
        bool write_tpwmthrs = false;
        bool write_tcoolthrs = false;
        bool write_thigh = false;
        bool write_global_scaler = false;
        bool write_vstart = true;
        bool write_a1 = false;
        bool write_v1 = false;
        bool write_amax = true;
        bool write_vmax = true;
        bool write_dmax = true;
        bool write_d1 = false;
        bool write_vstop = true;
        bool write_tzerowait = true;
        bool write_dcctrl = false;

        bool enable_stealthchop = true;
        bool use_internal_rsense = false;
        bool invert_direction = false;
        bool diag0_on_error = true;
        bool diag0_on_otpw = true;
        bool diag1_on_stall = false;
        bool stop_enable = false;

        uint8_t ihold = 16;
        uint8_t irun = 31;
        uint8_t ihold_delay = 6;
        uint8_t tpowerdown = 20;

        uint16_t microsteps = 256;
        uint8_t toff = 4;
        int8_t hend = 1;
        uint8_t hstrt = 4;
        uint8_t blank_time = 2;
        bool high_vsense = false;
        bool enable_spreadcycle = false;
        bool enable_interpolation = true;
        bool double_edge_step = false;
        bool disable_s2g_protection = false;

        uint8_t pwm_ampl = 128;
        uint8_t pwm_grad = 4;
        uint8_t pwm_freq = 1;
        bool pwm_autoscale = true;
        bool pwm_symmetric = false;
        uint8_t pwm_freewheel = 0;

        uint8_t global_scaler = 0; ///< 0 disables write.
        uint32_t tpwm_thrs = 0;
        uint32_t tcool_thrs = 0;
        uint32_t thigh = 0;
        uint32_t dcctrl = 0;

        uint32_t vstart = 0;
        uint32_t a1 = 0;
        uint32_t v1 = 0;
        uint32_t amax = 2000;
        uint32_t vmax = 0;
        uint32_t dmax = 2000;
        uint32_t d1 = 0;
        uint32_t vstop = 10;
        uint32_t tzerowait = 0;

        uint32_t clock_frequency_hz = 12000000;
    };

    explicit Motor(SPI_Bridge& spi, void (*status_cb)(uint8_t) = nullptr)
        : spi_(spi)
        , status_cb_(status_cb)
    {
    }

    bool initialize(const Config& cfg);

    bool write(Reg reg, uint32_t value);
    bool read(Reg reg, uint32_t& value);
    [[nodiscard]] uint32_t read(Reg reg)
    {
        uint32_t v = 0;
        (void)read(reg, v);
        return v;
    }

    bool setMicrosteps(uint16_t microsteps, const Config& cfg_template);
    bool setCurrent(uint8_t irun, uint8_t ihold, uint8_t ihold_delay);
    bool setVelocity(float microsteps_per_second, Direction dir, uint32_t clock_hz);
    bool stop(uint32_t clock_hz);

    uint8_t lastStatus() const { return last_status_; }

private:
    bool datagram(const uint8_t tx[5], uint8_t* rx = nullptr);
    static uint32_t velocityToReg(float microsteps_per_second, uint32_t clock_hz);
    void handleStatus(uint8_t status);

    SPI_Bridge& spi_;
    void (*status_cb_)(uint8_t) = nullptr;
    uint8_t last_status_ = 0;
};

inline bool Motor::write(Reg reg, uint32_t value)
{
    uint8_t tx[5] = {
        static_cast<uint8_t>(0x80u | static_cast<uint8_t>(reg)),
        static_cast<uint8_t>(value >> 24),
        static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value) };
    return datagram(tx);
}

inline bool Motor::read(Reg reg, uint32_t& value)
{
    uint8_t tx[5] = { static_cast<uint8_t>(reg), 0, 0, 0, 0 };
    uint8_t first[5] = {};
    if (!datagram(tx, first))
        return false;
    uint8_t second[5] = {};
    if (!datagram(tx, second))
        return false;
    value = (uint32_t(second[1]) << 24) | (uint32_t(second[2]) << 16)
        | (uint32_t(second[3]) << 8) | uint32_t(second[4]);
    return true;
}

inline bool Motor::datagram(const uint8_t tx[5], uint8_t* rx)
{
    uint8_t local_rx[5] = {};
    uint8_t* target = rx ? rx : local_rx;

    auto guard = spi_.scopedSelect();
    if (!spi_.transfer(tx, target, 5))
        return false;

    handleStatus(target[0]);
    return true;
}

inline void Motor::handleStatus(uint8_t status)
{
    last_status_ = status;
    if (status_cb_)
        status_cb_(status);
}

inline uint32_t Motor::velocityToReg(float microsteps_per_second, uint32_t clock_hz)
{
    if (microsteps_per_second <= 0.f || clock_hz == 0u)
        return 0u;

    double value = static_cast<double>(microsteps_per_second) * (1ull << 24)
        / static_cast<double>(clock_hz);
    if (value < 0.0)
        value = 0.0;
    if (value > 0x7FFFFFu)
        value = 0x7FFFFFu;
    return static_cast<uint32_t>(value);
}

inline bool Motor::initialize(const Config& cfg)
{
    if (cfg.write_gconf) {
        uint32_t gconf = detail::makeGconf(cfg.enable_stealthchop,
            cfg.use_internal_rsense, cfg.invert_direction, cfg.diag0_on_error,
            cfg.diag0_on_otpw, cfg.diag1_on_stall, cfg.stop_enable);
        if (!write(Reg::GCONF, gconf))
            return false;
    }

    if (cfg.write_ihold_irun) {
        uint32_t ihold_irun = detail::encodeIHOLDIRUN(
            cfg.ihold, cfg.irun, cfg.ihold_delay);
        if (!write(Reg::IHOLD_IRUN, ihold_irun))
            return false;
    }

    if (cfg.write_tpowerdown) {
        if (!write(Reg::TPOWERDOWN, cfg.tpowerdown))
            return false;
    }

    if (cfg.write_chopconf) {
        uint32_t chopconf = detail::makeChopconf(cfg.toff, cfg.hend, cfg.hstrt,
            cfg.blank_time, cfg.high_vsense, cfg.enable_spreadcycle,
            cfg.enable_interpolation, cfg.double_edge_step,
            cfg.disable_s2g_protection, cfg.microsteps);
        if (!write(Reg::CHOPCONF, chopconf))
            return false;
    }

    if (cfg.write_pwmconf) {
        uint32_t pwmconf = detail::makePwmconf(cfg.pwm_ampl, cfg.pwm_grad,
            cfg.pwm_freq, cfg.pwm_autoscale, cfg.pwm_symmetric, cfg.pwm_freewheel);
        if (!write(Reg::PWMCONF, pwmconf))
            return false;
    }

    if (cfg.write_tpwmthrs && cfg.tpwm_thrs) {
        if (!write(Reg::TPWMTHRS, cfg.tpwm_thrs))
            return false;
    }

    if (cfg.write_tcoolthrs && cfg.tcool_thrs) {
        if (!write(Reg::TCOOLTHRS, cfg.tcool_thrs))
            return false;
    }

    if (cfg.write_thigh && cfg.thigh) {
        if (!write(Reg::THIGH, cfg.thigh))
            return false;
    }

    if (cfg.write_global_scaler && cfg.global_scaler) {
        if (!write(Reg::GLOBAL_SCALER, cfg.global_scaler))
            return false;
    }

    if (cfg.write_vstart) {
        if (!write(Reg::VSTART, cfg.vstart))
            return false;
    }
    if (cfg.write_a1) {
        if (!write(Reg::A1, cfg.a1))
            return false;
    }
    if (cfg.write_v1) {
        if (!write(Reg::V1, cfg.v1))
            return false;
    }
    if (cfg.write_amax) {
        if (!write(Reg::AMAX, cfg.amax))
            return false;
    }
    if (cfg.write_vmax) {
        if (!write(Reg::VMAX, cfg.vmax))
            return false;
    }
    if (cfg.write_dmax) {
        if (!write(Reg::DMAX, cfg.dmax))
            return false;
    }
    if (cfg.write_d1) {
        if (!write(Reg::D1, cfg.d1))
            return false;
    }
    if (cfg.write_vstop) {
        if (!write(Reg::VSTOP, cfg.vstop))
            return false;
    }
    if (cfg.write_tzerowait) {
        if (!write(Reg::TZEROWAIT, cfg.tzerowait))
            return false;
    }
    if (cfg.write_dcctrl && cfg.dcctrl) {
        if (!write(Reg::DCCTRL, cfg.dcctrl))
            return false;
    }

    if (!write(Reg::RAMPMODE, RAMPMODE_HOLD))
        return false;
    if (!write(Reg::VMAX, 0))
        return false;

    return true;
}

inline bool Motor::setMicrosteps(uint16_t microsteps, const Config& cfg_template)
{
    Config cfg = cfg_template;
    cfg.microsteps = microsteps;
    cfg.write_chopconf = true;
    cfg.write_gconf = false;
    cfg.write_pwmconf = false;
    cfg.write_ihold_irun = false;
    cfg.write_tpwmthrs = false;
    cfg.write_tcoolthrs = false;
    cfg.write_thigh = false;
    cfg.write_global_scaler = false;
    cfg.write_vstart = false;
    cfg.write_a1 = false;
    cfg.write_v1 = false;
    cfg.write_amax = false;
    cfg.write_vmax = false;
    cfg.write_dmax = false;
    cfg.write_d1 = false;
    cfg.write_vstop = false;
    cfg.write_tzerowait = false;
    cfg.write_dcctrl = false;
    cfg.write_tpowerdown = false;
    return initialize(cfg);
}

inline bool Motor::setCurrent(uint8_t irun, uint8_t ihold, uint8_t ihold_delay)
{
    uint32_t value = detail::encodeIHOLDIRUN(ihold, irun, ihold_delay);
    return write(Reg::IHOLD_IRUN, value);
}

inline bool Motor::setVelocity(float microsteps_per_second, Direction dir, uint32_t clock_hz)
{
    uint32_t vmax = velocityToReg(microsteps_per_second, clock_hz);
    if (!write(Reg::VMAX, vmax))
        return false;
    uint32_t mode = (dir == Direction::Forward) ? RAMPMODE_VELOCITY_POS
                                                : RAMPMODE_VELOCITY_NEG;
    return write(Reg::RAMPMODE, mode);
}

inline bool Motor::stop(uint32_t clock_hz)
{
    (void)clock_hz;
    if (!write(Reg::VMAX, 0))
        return false;
    return write(Reg::RAMPMODE, RAMPMODE_HOLD);
}

} // namespace tmc5160
