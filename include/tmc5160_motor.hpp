#pragma once

#include "TMC5160_regs.hpp"
#include "logger.hpp"
#include "spi_bridge.hpp"
#include <cmath>
#include <cstdint>

namespace tmc5160
{

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
        bool fast_standstill = false;

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
        bool enable_pwm_autoscale = true;
        bool enable_pwm_autograd = false;
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
    bool setVelocity(
        float microsteps_per_second, Direction dir, uint32_t clock_hz);
    bool stop(uint32_t clock_hz);

    uint8_t lastStatus() const { return last_status_; }

private:
    bool datagram(const uint8_t tx[5], uint8_t* rx = nullptr);
    static uint32_t velocityToReg(
        float microsteps_per_second, uint32_t clock_hz);
    void handleStatus(uint8_t status);

    SPI_Bridge& spi_;
    void (*status_cb_)(uint8_t) = nullptr;
    uint8_t last_status_ = 0;
};

inline bool Motor::write(Reg reg, uint32_t value)
{
    uint8_t tx[5] = { static_cast<uint8_t>(0x80u | static_cast<uint8_t>(reg)),
        static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value) };
    return datagram(tx);
}

inline bool Motor::read(Reg reg, uint32_t& value)
{
    // Send read command (register address without MSB set)
    uint8_t addr = static_cast<uint8_t>(reg);
    uint8_t tx[5] = { addr, 0, 0, 0, 0 };

    // First datagram gets status + old data
    uint8_t first[5] = {};
    if (!datagram(tx, first))
        return false;

    // Second datagram gets status + requested data
    uint8_t second[5] = {};
    if (!datagram(tx, second))
        return false;

    // Extract the data from the response
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

    // Debug: Log full SPI exchange
    logsys::printf(
        "[SPI] TX:[%02X %02X %02X %02X %02X] RX:[%02X %02X %02X %02X %02X]\r\n",
        tx[0], tx[1], tx[2], tx[3], tx[4], target[0], target[1], target[2],
        target[3], target[4]);

    handleStatus(target[0]);
    return true;
}

inline void Motor::handleStatus(uint8_t status)
{
    last_status_ = status;
    if (status_cb_)
        status_cb_(status);
}

inline uint32_t Motor::velocityToReg(
    float microsteps_per_second, uint32_t clock_hz)
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
    // Clear any existing GSTAT flags first
    logsys::printf("[INIT] Clearing GSTAT flags...\r\n");
    write(Reg::GSTAT, 0x07); // Clear all GSTAT flags (reset, drv_err, uv_cp)

    // Verify GSTAT was cleared
    uint32_t gstat_verify = 0;
    read(Reg::GSTAT, gstat_verify);
    logsys::printf(
        "[INIT] GSTAT after clear: 0x%08lX\r\n", (unsigned long)gstat_verify);

    if (cfg.write_gconf) {
        uint32_t gconf = 0;
        if (cfg.enable_stealthchop)
            gconf = GCONF::EN_PWM_MODE.set(gconf, 1);
        if (cfg.fast_standstill)
            gconf = GCONF::FASTSTANDSTILL.set(gconf, 1);
        if (cfg.invert_direction)
            gconf = GCONF::SHAFT.set(gconf, 1);
        if (cfg.diag0_on_error)
            gconf = GCONF::DIAG0_ERROR.set(gconf, 1);
        if (cfg.diag0_on_otpw)
            gconf = GCONF::DIAG0_OTPW.set(gconf, 1);
        if (cfg.diag1_on_stall)
            gconf = GCONF::DIAG1_STALL_or_DIR.set(gconf, 1);
        if (cfg.stop_enable)
            gconf = GCONF::STOP_ENABLE.set(gconf, 1);
        if (!write(Reg::GCONF, gconf))
            return false;
    }

    // CRITICAL: Write CHOPCONF in stages - first without TOFF, then enable
    if (cfg.write_chopconf) {
        // Build CHOPCONF with TOFF=0 (disabled) first
        uint32_t chopconf_disabled = 0;
        chopconf_disabled
            = CHOPCONF::TOFF.set(chopconf_disabled, 0); // Keep disabled
        chopconf_disabled = CHOPCONF::HEND.set(chopconf_disabled, cfg.hend);
        chopconf_disabled = CHOPCONF::HSTRT.set(chopconf_disabled, cfg.hstrt);
        chopconf_disabled
            = CHOPCONF::TBL.set(chopconf_disabled, cfg.blank_time);
        chopconf_disabled = SHORT_CONF::VSENSE.set(
            chopconf_disabled, cfg.high_vsense ? 1 : 0);
        chopconf_disabled = CHOPCONF::CHM.set(
            chopconf_disabled, cfg.enable_spreadcycle ? 1 : 0);
        chopconf_disabled
            = CHOPCONF::INTPOL.set(chopconf_disabled, cfg.enable_interpolation);
        chopconf_disabled
            = CHOPCONF::DEDGE.set(chopconf_disabled, cfg.double_edge_step);
        chopconf_disabled = CHOPCONF::DISS2G.set(
            chopconf_disabled, cfg.disable_s2g_protection);
        chopconf_disabled = set_mres(
            chopconf_disabled, static_cast<MicrostepRes>(cfg.microsteps));

        logsys::printf("[INIT] Writing CHOPCONF (TOFF=0): 0x%08lX\r\n",
            (unsigned long)chopconf_disabled);

        if (!write(Reg::CHOPCONF, chopconf_disabled))
            return false;

        // Read back to verify
        uint32_t chopconf_check = 0;
        read(Reg::CHOPCONF, chopconf_check);
        logsys::printf("[INIT] CHOPCONF readback (TOFF=0): 0x%08lX\r\n",
            (unsigned long)chopconf_check);
    }

    if (cfg.write_ihold_irun) {
        uint32_t ihold_irun = 0;
        ihold_irun = IHOLD_IRUN::IHOLD.set(ihold_irun, cfg.ihold);
        ihold_irun = IHOLD_IRUN::IRUN.set(ihold_irun, cfg.irun);
        ihold_irun = IHOLD_IRUN::IHOLDDELAY.set(ihold_irun, cfg.ihold_delay);

        logsys::printf(
            "[INIT] Writing IHOLD_IRUN: 0x%08lX (IHOLD=%lu IRUN=%lu)\r\n",
            (unsigned long)ihold_irun, (unsigned long)cfg.ihold,
            (unsigned long)cfg.irun);

        if (!write(Reg::IHOLD_IRUN, ihold_irun))
            return false;

        // Now enable the driver by setting TOFF > 0
        if (cfg.write_chopconf && cfg.toff > 0) {
            uint32_t chopconf_enabled = 0;
            chopconf_enabled = CHOPCONF::TOFF.set(chopconf_enabled, cfg.toff);
            chopconf_enabled = CHOPCONF::HEND.set(chopconf_enabled, cfg.hend);
            chopconf_enabled = CHOPCONF::HSTRT.set(chopconf_enabled, cfg.hstrt);
            chopconf_enabled
                = CHOPCONF::TBL.set(chopconf_enabled, cfg.blank_time);
            chopconf_enabled = SHORT_CONF::VSENSE.set(
                chopconf_enabled, cfg.high_vsense ? 1 : 0);
            chopconf_enabled = CHOPCONF::CHM.set(
                chopconf_enabled, cfg.enable_spreadcycle ? 1 : 0);
            chopconf_enabled = CHOPCONF::INTPOL.set(
                chopconf_enabled, cfg.enable_interpolation);
            chopconf_enabled
                = CHOPCONF::DEDGE.set(chopconf_enabled, cfg.double_edge_step);
            chopconf_enabled = CHOPCONF::DISS2G.set(
                chopconf_enabled, cfg.disable_s2g_protection);
            chopconf_enabled = set_mres(
                chopconf_enabled, static_cast<MicrostepRes>(cfg.microsteps));

            logsys::printf("[INIT] Enabling driver (TOFF=%lu): 0x%08lX\r\n",
                (unsigned long)cfg.toff, (unsigned long)chopconf_enabled);

            if (!write(Reg::CHOPCONF, chopconf_enabled))
                return false;
        }

        // Debug: Read GSTAT (global status) to check for UV/charge pump/reset
        uint32_t gstat = 0;
        if (read(Reg::GSTAT, gstat)) {
            uint32_t reset = gstat & 0x1;
            uint32_t drv_err = (gstat >> 1) & 0x1;
            uint32_t uv_cp = (gstat >> 2) & 0x1;
            logsys::printf(
                "[GSTAT] 0x%08lX reset=%lu drv_err=%lu uv_cp=%lu\r\n",
                (unsigned long)gstat, (unsigned long)reset,
                (unsigned long)drv_err, (unsigned long)uv_cp);
            if (uv_cp) {
                logsys::printf(
                    "[FAULT] Charge pump undervoltage detected!\r\n");
            }
        }

        // Debug: Read DRV_STATUS immediately after setting current
        uint32_t drv_status = 0;
        if (read(Reg::DRV_STATUS, drv_status)) {
            uint32_t otpw
                = (drv_status >> 26) & 0x1; // Overtemperature pre-warning
            uint32_t ot = (drv_status >> 25) & 0x1; // Overtemperature
            uint32_t s2ga = (drv_status >> 27) & 0x1; // Short to GND phase A
            uint32_t s2gb = (drv_status >> 28) & 0x1; // Short to GND phase B
            uint32_t s2vsa
                = (drv_status >> 29) & 0x1; // Short to supply phase A
            uint32_t s2vsb
                = (drv_status >> 30) & 0x1; // Short to supply phase B
            uint32_t ola = (drv_status >> 18) & 0x1; // Open load phase A
            uint32_t olb = (drv_status >> 19) & 0x1; // Open load phase B
            uint32_t stst = (drv_status >> 31) & 0x1; // Standstill detected

            logsys::printf("[DRV_STATUS] After IHOLD_IRUN: 0x%08lX\r\n",
                (unsigned long)drv_status);
            logsys::printf("[DRV_FAULTS] ot=%lu otpw=%lu s2ga=%lu s2gb=%lu "
                           "s2vsa=%lu s2vsb=%lu ola=%lu olb=%lu stst=%lu\r\n",
                (unsigned long)ot, (unsigned long)otpw, (unsigned long)s2ga,
                (unsigned long)s2gb, (unsigned long)s2vsa, (unsigned long)s2vsb,
                (unsigned long)ola, (unsigned long)olb, (unsigned long)stst);
        }

        // Read CHOPCONF again to see if TOFF was cleared
        uint32_t chopconf_final = 0;
        if (read(Reg::CHOPCONF, chopconf_final)) {
            uint32_t toff_final = chopconf_final & 0x0F;
            logsys::printf("[INIT] Final CHOPCONF: 0x%08lX TOFF=%lu %s\r\n",
                (unsigned long)chopconf_final, (unsigned long)toff_final,
                (toff_final == 0) ? "[DISABLED BY PROTECTION!]" : "[OK]");
        }
    }

    if (cfg.write_tpowerdown) {
        if (!write(Reg::TPOWERDOWN, cfg.tpowerdown))
            return false;
    }

    if (cfg.write_pwmconf) {
        uint32_t pwmconf = 0;
        pwmconf = PWMCONF::PWM_OFS.set(pwmconf, cfg.pwm_ampl);
        pwmconf = PWMCONF::PWM_GRAD.set(pwmconf, cfg.pwm_grad);
        pwmconf = PWMCONF::PWM_FREQ.set(pwmconf, cfg.pwm_freq);
        pwmconf = PWMCONF::PWM_AUTOSCALE.set(
            pwmconf, cfg.enable_pwm_autoscale ? 1 : 0);
        pwmconf = PWMCONF::PWM_AUTOGRAD.set(
            pwmconf, cfg.enable_pwm_autograd ? 1 : 0);
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

    if (!write(Reg::RAMPMODE, static_cast<uint32_t>(RAMPMODE::Mode::HOLD)))
        return false;

    return true;
}

inline bool Motor::setMicrosteps(
    uint16_t microsteps, const Config& cfg_template)
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
    uint32_t value = 0;
    value = IHOLD_IRUN::IHOLD.set(value, ihold);
    value = IHOLD_IRUN::IRUN.set(value, irun);
    value = IHOLD_IRUN::IHOLDDELAY.set(value, ihold_delay);
    return write(Reg::IHOLD_IRUN, value);
}

inline bool Motor::setVelocity(
    float microsteps_per_second, Direction dir, uint32_t clock_hz)
{
    // Calculate VMAX register value
    uint32_t vmax = velocityToReg(microsteps_per_second, clock_hz);

    // Set RAMPMODE first (order matters for TMC5160)
    uint32_t mode = (dir == Direction::Forward)
        ? static_cast<uint32_t>(RAMPMODE::Mode::VEL_POS)
        : static_cast<uint32_t>(RAMPMODE::Mode::VEL_NEG);
    if (!write(Reg::RAMPMODE, mode))
        return false;

    // CRITICAL: Write AMAX unconditionally (must be set for velocity mode)
    if (!write(Reg::AMAX, 5000))
        return false;

    // Write VMAX
    if (!write(Reg::VMAX, vmax))
        return false;

    return true;
}

inline bool Motor::stop(uint32_t clock_hz)
{
    (void)clock_hz;
    if (!write(Reg::VMAX, 0))
        return false;
    return write(Reg::RAMPMODE, static_cast<uint32_t>(RAMPMODE::Mode::HOLD));
}

} // namespace tmc5160
