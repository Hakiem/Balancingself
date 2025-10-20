#pragma once

#include "spi_bridge.hpp"
#include "tmc2130_bits.hpp"
#include "tmc2130_registers.hpp"
#include <cstddef>
#include <cstdint>

namespace tmc2130 {

/**
 * @brief High-level helper encapsulating SPI access to a single TMC2130.
 */
class Motor
{
public:
    /**
     * @brief Configuration bundle consumed by @ref initialize.
     *
     * The fields mirror the most common bring-up knobs from the datasheet and
     * default to a conservative StealthChop-friendly setup. Individual flags
     * determine which registers are written; untouched registers keep their
     * power-on values.
     */
    struct Config {
        /// When true, @ref Reg::GCONF is written with @ref makeGconf output.
        bool write_gconf = true;
        /// When true, @ref Reg::IHOLD_IRUN is written.
        bool write_ihold_irun = true;
        /// When true, @ref Reg::CHOPCONF is programmed.
        bool write_chopconf = true;
        /// When true, @ref Reg::PWMCONF is programmed.
        bool write_pwmconf = true;
        /// When true, @ref Reg::TPOWERDOWN is programmed.
        bool write_tpowerdown = true;
        bool write_tpwmthrs = false;
        bool write_tcoolthrs = false;
        bool write_thigh = false;
        bool write_coolconf = false;
        bool write_dcctrl = false;

        // GCONF options
        /// Enable StealthChop PWM mode.
        bool enable_stealthchop = true;
        /// Force internal sense resistors (usually false for external Rsense).
        bool use_internal_rsense = false;
        /// Flip the motor direction (equivalent to GCONF.shaft).
        bool invert_direction = false;
        /// Route driver errors to DIAG0.
        bool diag0_on_error = true;
        /// Route overtemperature prewarning to DIAG0.
        bool diag0_on_otpw = true;
        /// Route stall indicator to DIAG1.
        bool diag1_on_stall = false;
        /// Drive coils directly via XDIRECT (bypasses STEP/DIR logic).
        bool direct_mode = false;

        // IHOLD_IRUN options
        uint8_t ihold = 8; ///< Standstill current (0..31).
        uint8_t irun = 31; ///< Running current (0..31).
        uint8_t ihold_delay = 6; ///< Current ramp delay (0..15).

        // CHOPCONF options
        uint8_t toff = 4; ///< Slow-decay off time (>=3 enables driver).
        int8_t hend = 2; ///< Hysteresis end offset (-3..12).
        uint8_t hstrt = 4; ///< Hysteresis start (or TFD[2:0] in constant mode).
        uint8_t blank_time = 1; ///< Comparator blank time selector (0..3).
        bool use_constant_off_time = true; ///< Set CHM to StealthChop mode.
        bool enable_interpolation = true; ///< Enable 256 µstep interpolation.
        bool double_edge_step = false; ///< Double edge STEP pulses (DEdge).
        bool disable_s2g_protection = false; ///< Disable short-to-ground detect.
        bool high_vsense = true; ///< Use 0.18V sense reference (VSENSE=1).
        uint16_t microsteps = 256; ///< Desired microstep resolution (1..256).

        // PWMCONF options
        uint8_t pwm_ampl = 128; ///< User amplitude offset (0..255).
        uint8_t pwm_grad = 4; ///< Velocity gradient.
        uint8_t pwm_freq = 1; ///< PWM frequency selector (0..3).
        bool pwm_autoscale = true; ///< Let StealthChop autoscale amplitude.
        bool pwm_symmetric = false; ///< Force symmetric PWM when true.
        uint8_t pwm_freewheel = 0; ///< Freewheel mode (0..3).

        // Misc timing thresholds
        uint8_t tpowerdown = 20; ///< Standstill powerdown delay (units of 2^18 clocks).
        uint32_t tpwmthrs = 0; ///< StealthChop threshold (0 disables write).
        uint32_t tcoolthrs = 0; ///< CoolStep/StallGuard threshold.
        uint32_t thigh = 0; ///< High-velocity threshold.
        uint32_t coolconf = 0; ///< Raw value to write into COOLCONF when enabled.
        uint32_t dcctrl = 0; ///< Raw value to write into DCCTRL when enabled.
    };

    /**
     * @param spi       Reference to an initialized SPI bridge.
     * @param status_cb Optional callback invoked after each transfer with the
     *                  8-bit status byte returned by the driver.
     */
    explicit Motor(SPI_Bridge& spi, void (*status_cb)(uint8_t) = nullptr)
        : spi_(spi)
        , status_cb_(status_cb)
    {
    }

    /** @brief Write a 32-bit value to a TMC2130 register. */
    bool write(Reg reg, uint32_t value);

    /**
     * @brief Read a 32-bit value from a TMC2130 register.
     *
     * The driver returns the requested data one frame after the read command.
     * This helper performs the extra 40-bit dummy transfer transparently and
     * returns the freshly latched register value.
     */
    bool read(Reg reg, uint32_t& value);

    /** @brief Convenience overload that returns the value. */
    [[nodiscard]] uint32_t read(Reg reg)
    {
        uint32_t v = 0;
        (void)read(reg, v);
        return v;
    }

    /**
     * @brief Exchange a raw 40-bit datagram (address byte + 32-bit payload).
     *
     * @param tx Pointer to the 5-byte frame to send (MSB first, address byte
     *           followed by data).
     * @param rx Optional pointer to capture the 5-byte reply (status bits plus
     *           the data resulting from the previous command, as defined in the
     *           datasheet). When nullptr, the reply is ignored except for the
     *           status byte bookkeeping.
     * @return true when the HAL transfer succeeds.
     */
    bool datagram(const uint8_t tx[5], uint8_t* rx = nullptr);

    /**
     * @brief Apply a convenient default configuration.
     *
     * Writes the registers indicated by the @ref Config flags. Returns false if
     * any SPI transfer fails.
     */
    bool initialize(const Config& cfg);

    /// @brief Convenience overload using default-constructed configuration.
    bool initialize() { return initialize(Config{}); }

#ifdef UNIT_TEST
    using DatagramHook = bool (*)(const uint8_t* tx, uint8_t* rx);
    static void setDatagramHook(DatagramHook hook) { datagram_hook_ = hook; }
    static bool hasDatagramHook() { return datagram_hook_ != nullptr; }
#endif

    /**
     * @brief Latest SPI status byte latched during the most recent transfer.
     */
    /** @brief Most recent SPI status byte (bits per datasheet table 4.1). */
    uint8_t lastStatus() const { return last_status_; }

private:
    /// Record and optionally forward the SPI status byte.
    void handleStatus(uint8_t status);

    SPI_Bridge& spi_;
    void (*status_cb_)(uint8_t);
    uint8_t last_status_ = 0;
#ifdef UNIT_TEST
    static inline DatagramHook datagram_hook_ = nullptr;
#endif
};

/**
 * @brief Issue a TMC2130 write datagram to the selected register.
 *
 * The address byte is sent with WRITE_notREAD set (bit 7 = 1) and the
 * 32-bit payload follows MSB first, matching the timing diagram in section
 * 4.1 of the datasheet.
 */
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

/**
 * @brief Perform the two-step read sequence required by the TMC2130.
 *
 * The first frame primes the internal pipeline; the second returns the
 * contents of the requested register. Both status bytes are captured for
 * later inspection via @ref lastStatus.
 */
inline bool Motor::read(Reg reg, uint32_t& value)
{
    uint8_t tx[5] = { static_cast<uint8_t>(reg), 0, 0, 0, 0 };

    uint8_t first[5] = {};
    uint8_t second[5] = {};

    // First transfer primes the read pipeline (discard returned data).
    if (!datagram(tx, first))
        return false;

    // Second transfer clocks out the requested register contents.
    if (!datagram(tx, second))
        return false;

    value = (uint32_t(second[1]) << 24) | (uint32_t(second[2]) << 16)
        | (uint32_t(second[3]) << 8) | second[4];
    return true;
}

/**
 * @brief Send an arbitrary SPI frame and optionally capture the reply.
 *
 * Useful for advanced diagnostics or when stacking operations (e.g.,
 * daisy-chained drivers) where the caller wants raw access to the datagram.
 */
inline bool Motor::datagram(const uint8_t tx[5], uint8_t* rx)
{
    uint8_t local_rx[5] = {};
    uint8_t* target = rx ? rx : local_rx;

#ifdef UNIT_TEST
    if (datagram_hook_) {
        if (!datagram_hook_(tx, target))
            return false;
        handleStatus(target[0]);
        return true;
    }
#endif

    auto guard = spi_.scopedSelect();
    if (!spi_.transfer(tx, target, 5))
        return false;

    handleStatus(target[0]);
    return true;
}

/**
 * @brief Program the device using the supplied configuration profile.
 *
 * Only the registers with corresponding `write_*` flags set are touched.
 * Returns false if any SPI transaction reports an error.
 */
inline bool Motor::initialize(const Config& cfg)
{
    bool ok = true;

    if (cfg.write_gconf) {
        ok &= write(Reg::GCONF,
            detail::makeGconf(cfg.enable_stealthchop, cfg.use_internal_rsense,
                cfg.invert_direction, cfg.diag0_on_error, cfg.diag0_on_otpw,
                cfg.diag1_on_stall, cfg.direct_mode));
    }
    if (cfg.write_ihold_irun) {
        ok &= write(Reg::IHOLD_IRUN,
            detail::encodeIHOLDIRUN(cfg.ihold, cfg.irun, cfg.ihold_delay));
    }
    if (cfg.write_tpowerdown) {
        ok &= write(Reg::TPOWERDOWN, cfg.tpowerdown);
    }
    if (cfg.write_tpwmthrs) {
        ok &= write(Reg::TPWMTHRS, cfg.tpwmthrs);
    }
    if (cfg.write_tcoolthrs) {
        ok &= write(Reg::TCOOLTHRS, cfg.tcoolthrs);
    }
    if (cfg.write_thigh) {
        ok &= write(Reg::THIGH, cfg.thigh);
    }
    if (cfg.write_chopconf) {
        ok &= write(Reg::CHOPCONF,
            detail::makeChopconf(cfg.toff, cfg.hend, cfg.hstrt, cfg.blank_time,
                cfg.high_vsense, cfg.use_constant_off_time, cfg.enable_interpolation,
                cfg.double_edge_step, cfg.disable_s2g_protection, cfg.microsteps));
    }
    if (cfg.write_pwmconf) {
        ok &= write(Reg::PWMCONF,
            detail::makePwmconf(cfg.pwm_ampl, cfg.pwm_grad, cfg.pwm_freq,
                cfg.pwm_autoscale, cfg.pwm_symmetric, cfg.pwm_freewheel));
    }
    if (cfg.write_coolconf) {
        ok &= write(Reg::COOLCONF, cfg.coolconf);
    }
    if (cfg.write_dcctrl) {
        ok &= write(Reg::DCCTRL, cfg.dcctrl);
    }
    return ok;
}

/**
 * @brief Track the latest SPI status byte and forward it to the callback.
 */
inline void Motor::handleStatus(uint8_t status)
{
    last_status_ = status;
    if (status_cb_)
        status_cb_(status);
}

} // namespace tmc2130
