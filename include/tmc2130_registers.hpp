#pragma once

#include <stdint.h>

/**
 * @brief Helpers for addressing and configuring the Trinamic TMC2130 over SPI.
 *
 * All register addresses and bit masks are taken from the TMC2130 datasheet
 * (Rev. 1.15). SPI writes must set the MSB of the address (addr | 0x80). Reads
 * use the plain 7-bit address.
 */
namespace tmc2130 {

/**
 * @brief Register addresses for the TMC2130 motion controller/driver.
 */
enum class Reg : uint8_t {
    /// 0x00 (RW): Global configuration flags.
    GCONF = 0x00,
    /// 0x01 (R+C): Global status flags (clear on read).
    GSTAT = 0x01,
    /// 0x04 (R): Input pin states and silicon version.
    IOIN = 0x04,

    /// 0x10 (W): Standstill/run current control and delay.
    IHOLD_IRUN = 0x10,
    /// 0x11 (W): Auto power-down delay after standstill.
    TPOWERDOWN = 0x11,
    /// 0x12 (R): Measured microstep period (1/256 step).
    TSTEP = 0x12,
    /// 0x13 (W): Upper velocity threshold for StealthChop PWM.
    TPWMTHRS = 0x13,
    /// 0x14 (W): Lower threshold for CoolStep/StallGuard2.
    TCOOLTHRS = 0x14,
    /// 0x15 (W): Threshold for chopper/full-step switching.
    THIGH = 0x15,

    /**
     * @brief 0x2D (RW): Direct SPI-controlled coil currents.
     *
     * @details When @ref GCONF_direct_mode is set, the signed 9-bit values in
     * XDIRECT override the normal current regulation path (including any
     * external VREF potentiometer setting). Coil A current is stored in bits
     * 8:0, coil B current in bits 24:16. Velocity-based PWM regulation, DcStep,
     * and automatic current scaling are disabled in this mode, and CoolStep /
     * StallGuard2 require an external STEP signal.
     */
    XDIRECT = 0x2D,

    /**
     * @brief 0x33 (W): Minimum DcStep velocity.
     *
     * @details Defines the step-rate threshold at which DcStep is permitted to
     * engage once the external @c DCEN input is asserted. The raw value is a
     * 23-bit time reference (@f$t = \mathrm{value} \times 2^{24} / f_{\mathrm{clk}}@f$).
     * Configure @ref Reg::DCCTRL in conjunction with this register to tune
     * DcStep behaviour for heavily loaded moves.
     */
    VDCMIN = 0x33,

    /// 0x60 (W): Microstep LUT entries 0–31.
    MSLUT0 = 0x60,
    /// 0x61 (W): Microstep LUT entries 32–63.
    MSLUT1 = 0x61,
    /// 0x62 (W): Microstep LUT entries 64–95.
    MSLUT2 = 0x62,
    /// 0x63 (W): Microstep LUT entries 96–127.
    MSLUT3 = 0x63,
    /// 0x64 (W): Microstep LUT entries 128–159.
    MSLUT4 = 0x64,
    /// 0x65 (W): Microstep LUT entries 160–191.
    MSLUT5 = 0x65,
    /// 0x66 (W): Microstep LUT entries 192–223.
    MSLUT6 = 0x66,
    /// 0x67 (W): Microstep LUT entries 224–255.
    MSLUT7 = 0x67,
    /// 0x68 (W): LUT segment selector (four segments per quarter wave).
    MSLUTSEL = 0x68,
    /// 0x69 (W): START_SIN and START_SIN90 initial values.
    MSLUTSTART = 0x69,
    /// 0x6A (R): Microstep counter (sine table index).
    MSCNT = 0x6A,
    /// 0x6B (R): Actual microstep currents (CUR_A/CUR_B).
    MSCURACT = 0x6B,

    /// 0x6C (RW): SpreadCycle/Classic chopper configuration.
    CHOPCONF = 0x6C,
    /// 0x6D (RW): CoolStep and StallGuard2 configuration.
    COOLCONF = 0x6D,
    /// 0x6E (W): DcStep configuration (DC_TIME/DC_SG).
    DCCTRL = 0x6E,
    /// 0x6F (R): Driver error flags and StallGuard2 result.
    DRV_STATUS = 0x6F,
    /// 0x70 (W): StealthChop PWM configuration.
    PWMCONF = 0x70,
    /// 0x71 (R): Actual PWM amplitude scaler (255 = max).
    PWM_SCALE = 0x71,
    /// 0x72 (W): Encoder commutation mode control.
    ENCM_CTRL = 0x72,
    /// 0x73 (R): DcStep lost steps counter.
    LOST_STEPS = 0x73
};

/**
 * @brief Convenience list for iterating over the eight microstep LUT registers.
 *
 * The TMC2130 splits one electrical revolution into four quarters, each
 * defined by 256 positions. The eight LUT registers hold the 32 differential
 * entries required per quarter wave (see datasheet section 5.5).
 */
static constexpr Reg MSLUT[8] = {
    Reg::MSLUT0, Reg::MSLUT1, Reg::MSLUT2, Reg::MSLUT3,
    Reg::MSLUT4, Reg::MSLUT5, Reg::MSLUT6, Reg::MSLUT7
};

/// @name GCONF (Global configuration register) bit masks
///@{
/// Use external AIN as the current reference.
constexpr uint32_t GCONF_I_scale_analog = 1u << 0;
/// Use internal sense resistors instead of external Rsense.
constexpr uint32_t GCONF_internal_Rsense = 1u << 1;
/// Enable StealthChop PWM mode (velocity dependent).
constexpr uint32_t GCONF_en_pwm_mode = 1u << 2;
/// Enable full-step encoder commutation.
constexpr uint32_t GCONF_enc_commutation = 1u << 3;
/// Invert motor direction (shaft swap).
constexpr uint32_t GCONF_shaft = 1u << 4;
/// Route driver error events to DIAG0.
constexpr uint32_t GCONF_diag0_error = 1u << 5;
/// Route overtemperature prewarning to DIAG0.
constexpr uint32_t GCONF_diag0_otpw = 1u << 6;
/// Route motor stall indication to DIAG0.
constexpr uint32_t GCONF_diag0_stall = 1u << 7;
/// Route motor stall indication to DIAG1.
constexpr uint32_t GCONF_diag1_stall = 1u << 8;
/// Route microstep index position to DIAG1.
constexpr uint32_t GCONF_diag1_index = 1u << 9;
/// Route chopper-on indication to DIAG1.
constexpr uint32_t GCONF_diag1_onstate = 1u << 10;
/// Toggle DIAG1 when DcStep skips a step.
constexpr uint32_t GCONF_diag1_steps_skipped = 1u << 11;
/// Configure DIAG0 as push-pull (default open-drain).
constexpr uint32_t GCONF_diag0_pushpull = 1u << 12;
/// Configure DIAG1 as push-pull (default open-drain).
constexpr uint32_t GCONF_diag1_pushpull = 1u << 13;
/// Reduce StallGuard comparator hysteresis to 1/32.
constexpr uint32_t GCONF_small_hysteresis = 1u << 14;
/// Enable emergency stop via the DCIN input.
constexpr uint32_t GCONF_stop_enable = 1u << 15;
/**
 * @brief Activate direct coil current control through XDIRECT.
 *
 * @details Setting this bit immediately switches the driver to use the values
 * stored in @ref Reg::XDIRECT. This bypasses StealthChop voltage regulation and
 * any analog VREF/potentiometer hardware. Use with caution—other smart current
 * features (DcStep, automatic scaling, StallGuard-based adaptation) become
 * unavailable.
 */
constexpr uint32_t GCONF_direct_mode = 1u << 16;
/// Test mode (should remain cleared during normal operation).
constexpr uint32_t GCONF_test_mode = 1u << 17;
///@}

/// @name GSTAT (Global status register) bit masks
///@{
/// Indicates that the device has reset since the last read.
constexpr uint32_t GSTAT_reset = 1u << 0;
/// Driver shut down due to overtemperature or short circuit.
constexpr uint32_t GSTAT_drv_err = 1u << 1;
/// Undervoltage detected on the charge pump (driver disabled).
constexpr uint32_t GSTAT_uv_cp = 1u << 2;
///@}

/// @name IOIN (Input pin state register) bit masks
///@{
/// STEP input level.
constexpr uint32_t IOIN_STEP = 1u << 0;
/// DIR input level.
constexpr uint32_t IOIN_DIR = 1u << 1;
/// DCEN/CFG4 input level.
constexpr uint32_t IOIN_DCEN_CFG4 = 1u << 2;
/// DCIN/CFG5 input level.
constexpr uint32_t IOIN_DCIN_CFG5 = 1u << 3;
/// DRV_ENN/CFG6 input level.
constexpr uint32_t IOIN_DRV_ENN_CFG6 = 1u << 4;
/// Digital current comparator output.
constexpr uint32_t IOIN_DCO = 1u << 5;
/// VERSION field (0x11 for Rev. 1.15 silicon).
constexpr uint32_t IOIN_VERSION_MASK = 0xFFu << 24;
/// Bit position of the VERSION field.
constexpr uint32_t IOIN_VERSION_SHIFT = 24;
///@}

/// @name IHOLD_IRUN register field helpers
///@{
/// Standstill current (0–31) mask.
constexpr uint32_t IHOLD_IRUN_IHOLD_MASK = 0x1Fu << 0;
/// Bit position of the standstill current field.
constexpr uint32_t IHOLD_IRUN_IHOLD_SHIFT = 0;
/// Run current (0–31) mask.
constexpr uint32_t IHOLD_IRUN_IRUN_MASK = 0x1Fu << 8;
/// Bit position of the run current field.
constexpr uint32_t IHOLD_IRUN_IRUN_SHIFT = 8;
/// Current ramp delay (0–15) mask.
constexpr uint32_t IHOLD_IRUN_IHOLDDELAY_MASK = 0x0Fu << 16;
/// Bit position of the IHOLDDELAY field.
constexpr uint32_t IHOLD_IRUN_IHOLDDELAY_SHIFT = 16;
///@}

/// Auto power-down delay (value * 2^18 clock cycles).
constexpr uint32_t TPOWERDOWN_MASK = 0xFFu;

/// Measured microstep period (1/256 steps).
constexpr uint32_t TSTEP_MASK = 0xFFFFFu;
/// Upper velocity threshold for StealthChop PWM.
constexpr uint32_t TPWMTHRS_MASK = 0xFFFFFu;
/// Lower threshold for CoolStep/StallGuard2 activation.
constexpr uint32_t TCOOLTHRS_MASK = 0xFFFFFu;
/// Velocity threshold for switching to full-step mode.
constexpr uint32_t THIGH_MASK = 0xFFFFFu;
/// Minimum DcStep velocity time reference (value * 2^24 / f_clk).
constexpr uint32_t VDCMIN_MASK = 0x7FFFFFu;

/// @name MSLUTSEL segmentation definitions
///@{
/// Segment 0 LUT width code (bits 1:0).
constexpr uint32_t MSLUTSEL_W0_MASK = 0x3u << 0;
/// Shift helper for @ref MSLUTSEL_W0_MASK.
constexpr uint32_t MSLUTSEL_W0_SHIFT = 0;
/// Segment 1 LUT width code (bits 3:2).
constexpr uint32_t MSLUTSEL_W1_MASK = 0x3u << 2;
/// Shift helper for @ref MSLUTSEL_W1_MASK.
constexpr uint32_t MSLUTSEL_W1_SHIFT = 2;
/// Segment 2 LUT width code (bits 5:4).
constexpr uint32_t MSLUTSEL_W2_MASK = 0x3u << 4;
/// Shift helper for @ref MSLUTSEL_W2_MASK.
constexpr uint32_t MSLUTSEL_W2_SHIFT = 4;
/// Segment 3 LUT width code (bits 7:6).
constexpr uint32_t MSLUTSEL_W3_MASK = 0x3u << 6;
/// Shift helper for @ref MSLUTSEL_W3_MASK.
constexpr uint32_t MSLUTSEL_W3_SHIFT = 6;
/// LUT segment 1 start index X1 (bits 15:8).
constexpr uint32_t MSLUTSEL_X1_MASK = 0xFFu << 8;
/// Shift helper for @ref MSLUTSEL_X1_MASK.
constexpr uint32_t MSLUTSEL_X1_SHIFT = 8;
/// LUT segment 2 start index X2 (bits 23:16).
constexpr uint32_t MSLUTSEL_X2_MASK = 0xFFu << 16;
/// Shift helper for @ref MSLUTSEL_X2_MASK.
constexpr uint32_t MSLUTSEL_X2_SHIFT = 16;
/// LUT segment 3 start index X3 (bits 31:24).
constexpr uint32_t MSLUTSEL_X3_MASK = 0xFFu << 24;
/// Shift helper for @ref MSLUTSEL_X3_MASK.
constexpr uint32_t MSLUTSEL_X3_SHIFT = 24;
///@}

/// @name MSLUTSTART initial conditions
///@{
/// START_SIN (bits 15:0) default LUT start value.
constexpr uint32_t MSLUTSTART_START_SIN_MASK = 0xFFFFu << 0;
/// START_SIN bit shift helper.
constexpr uint32_t MSLUTSTART_START_SIN_SHIFT = 0;
/// START_SIN90 (bits 31:16) default 90° LUT start value.
constexpr uint32_t MSLUTSTART_START_SIN90_MASK = 0xFFFFu << 16;
/// START_SIN90 bit shift helper.
constexpr uint32_t MSLUTSTART_START_SIN90_SHIFT = 16;
///@}

/// Microstep counter range (0–1023).
constexpr uint32_t MSCNT_MASK = 0x3FFu;

/// @name MSCURACT field helpers
///@{
/// Signed coil B current sample (bits 8:0).
constexpr uint32_t MSCURACT_CUR_B_MASK = 0x1FFu << 0;
/// CUR_B shift helper.
constexpr uint32_t MSCURACT_CUR_B_SHIFT = 0;
/// Signed coil A current sample (bits 24:16).
constexpr uint32_t MSCURACT_CUR_A_MASK = 0x1FFu << 16;
/// CUR_A shift helper.
constexpr uint32_t MSCURACT_CUR_A_SHIFT = 16;
///@}

/// @name CHOPCONF (chopper configuration) bit masks
///@{
/// Slow-decay off-time (bits 3:0); set >0 to enable driver.
constexpr uint32_t CHOPCONF_TOFF_MASK = 0xFu << 0;
/// Shift helper for @ref CHOPCONF_TOFF_MASK.
constexpr uint32_t CHOPCONF_TOFF_SHIFT = 0;
/// Hysteresis start value HSTRT (bits 6:4, SpreadCycle).
constexpr uint32_t CHOPCONF_HSTRT_MASK = 0x7u << 4;
/// Shift helper for @ref CHOPCONF_HSTRT_MASK.
constexpr uint32_t CHOPCONF_HSTRT_SHIFT = 4;
/// Hysteresis end value HEND (bits 10:7, SpreadCycle).
constexpr uint32_t CHOPCONF_HEND_MASK = 0xFu << 7;
/// Shift helper for @ref CHOPCONF_HEND_MASK.
constexpr uint32_t CHOPCONF_HEND_SHIFT = 7;
/// Fast-decay time (TFD) MSB (bit 11, constant-off mode).
constexpr uint32_t CHOPCONF_FD3 = 1u << 11;
/// Disable current comparator for fast decay (bit 12).
constexpr uint32_t CHOPCONF_DISFDCC = 1u << 12;
/// Randomize TOFF within ±3 clocks (bit 13).
constexpr uint32_t CHOPCONF_RNDTF = 1u << 13;
/// Chopper mode select: 0=SpreadCycle, 1=constant off (bit 14).
constexpr uint32_t CHOPCONF_CHM = 1u << 14;
/// Comparator blank time select TBL (bits 16:15).
constexpr uint32_t CHOPCONF_TBL_MASK = 0x3u << 15;
/// Shift helper for @ref CHOPCONF_TBL_MASK.
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;
/// Sense resistor voltage scaling (bit 17).
constexpr uint32_t CHOPCONF_VSENSE = 1u << 17;
/// Enable full-step switching above VHIGH (bit 18).
constexpr uint32_t CHOPCONF_VHIGHFS = 1u << 18;
/// Enable high-velocity chopper mode above VHIGH (bit 19).
constexpr uint32_t CHOPCONF_VHIGHCHM = 1u << 19;
/// Chopper synchronization setting SYNC (bits 23:20).
constexpr uint32_t CHOPCONF_SYNC_MASK = 0xFu << 20;
/// Shift helper for @ref CHOPCONF_SYNC_MASK.
constexpr uint32_t CHOPCONF_SYNC_SHIFT = 20;
/// Microstep resolution MRES (bits 27:24).
constexpr uint32_t CHOPCONF_MRES_MASK = 0xFu << 24;
/// Shift helper for @ref CHOPCONF_MRES_MASK.
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;
/// Enable 256 µstep interpolation (bit 28).
constexpr uint32_t CHOPCONF_INTPOL = 1u << 28;
/// Double edge STEP pulses (bit 29).
constexpr uint32_t CHOPCONF_DEDGE = 1u << 29;
/// Disable short-to-ground protection (bit 30).
constexpr uint32_t CHOPCONF_DISS2G = 1u << 30;
///@}

/// @name COOLCONF (smart current control) bit masks
///@{
/// StallGuard filtering enable (bit 24).
constexpr uint32_t COOLCONF_SFILT = 1u << 24;
/// StallGuard threshold SGT (bits 22:16, signed -64…+63).
constexpr uint32_t COOLCONF_SGT_MASK = 0x7Fu << 16;
/// Shift helper for @ref COOLCONF_SGT_MASK.
constexpr uint32_t COOLCONF_SGT_SHIFT = 16;
/// Minimum current for smart energy control (bit 15).
constexpr uint32_t COOLCONF_SEIMIN = 1u << 15;
/// Current decrement speed SEDN (bits 13:12).
constexpr uint32_t COOLCONF_SEDN_MASK = 0x3u << 12;
/// Shift helper for @ref COOLCONF_SEDN_MASK.
constexpr uint32_t COOLCONF_SEDN_SHIFT = 12;
/// StallGuard hysteresis SEMAX (bits 10:8).
constexpr uint32_t COOLCONF_SEMAX_MASK = 0x7u << 8;
/// Shift helper for @ref COOLCONF_SEMAX_MASK.
constexpr uint32_t COOLCONF_SEMAX_SHIFT = 8;
/// Current increment step width SEUP (bits 6:5).
constexpr uint32_t COOLCONF_SEUP_MASK = 0x3u << 5;
/// Shift helper for @ref COOLCONF_SEUP_MASK.
constexpr uint32_t COOLCONF_SEUP_SHIFT = 5;
/// Minimum StallGuard threshold for smart current SEMIN (bits 3:0).
constexpr uint32_t COOLCONF_SEMIN_MASK = 0xFu << 0;
/// Shift helper for @ref COOLCONF_SEMIN_MASK.
constexpr uint32_t COOLCONF_SEMIN_SHIFT = 0;
///@}

/// @name DCCTRL register fields
///@{
/// Maximum PWM on-time before commutation (bits 9:0).
constexpr uint32_t DCCTRL_DC_TIME_MASK = 0x3FFu << 0;
/// DC_TIME shift helper.
constexpr uint32_t DCCTRL_DC_TIME_SHIFT = 0;
/// Maximum PWM on-time allowed for missed-step detection (bits 23:16).
constexpr uint32_t DCCTRL_DC_SG_MASK = 0xFFu << 16;
/// DC_SG shift helper.
constexpr uint32_t DCCTRL_DC_SG_SHIFT = 16;
///@}

/// @name PWMCONF (StealthChop voltage PWM) bit masks
///@{
/// User-defined PWM amplitude offset (bits 7:0).
constexpr uint32_t PWMCONF_PWM_AMPL_MASK = 0xFFu << 0;
/// Shift helper for @ref PWMCONF_PWM_AMPL_MASK.
constexpr uint32_t PWMCONF_PWM_AMPL_SHIFT = 0;
/// Velocity-dependent PWM gradient (bits 15:8).
constexpr uint32_t PWMCONF_PWM_GRAD_MASK = 0xFFu << 8;
/// Shift helper for @ref PWMCONF_PWM_GRAD_MASK.
constexpr uint32_t PWMCONF_PWM_GRAD_SHIFT = 8;
/// PWM frequency selection (bits 17:16).
constexpr uint32_t PWMCONF_PWM_FREQ_MASK = 0x3u << 16;
/// Shift helper for @ref PWMCONF_PWM_FREQ_MASK.
constexpr uint32_t PWMCONF_PWM_FREQ_SHIFT = 16;
/// Enable automatic amplitude scaling (bit 18).
constexpr uint32_t PWMCONF_PWM_AUTOSCALE = 1u << 18;
/// Enforce symmetric PWM (bit 19).
constexpr uint32_t PWMCONF_PWM_SYMMETRIC = 1u << 19;
/// Freewheel mode selection (bits 21:20).
constexpr uint32_t PWMCONF_FREEWHEEL_MASK = 0x3u << 20;
/// Shift helper for @ref PWMCONF_FREEWHEEL_MASK.
constexpr uint32_t PWMCONF_FREEWHEEL_SHIFT = 20;
///@}

/// PWM amplitude scaler (0–255).
constexpr uint32_t PWM_SCALE_MASK = 0xFFu;

/// @name ENCM_CTRL feature bits
///@{
/// Invert encoder input direction.
constexpr uint32_t ENCM_CTRL_INV = 1u << 0;
/// Ignore STEP input and use IHOLD to set current (max speed mode).
constexpr uint32_t ENCM_CTRL_MAXSPEED = 1u << 1;
///@}

/// Lost-steps counter (20-bit up/down counter).
constexpr uint32_t LOST_STEPS_MASK = 0xFFFFFu;

/// @name DRV_STATUS (driver diagnostics) bit masks
///@{
/// StallGuard result (bits 9:0).
constexpr uint32_t DRV_STATUS_SG_RESULT_MASK = 0x3FFu << 0;
/// Shift helper for @ref DRV_STATUS_SG_RESULT_MASK.
constexpr uint32_t DRV_STATUS_SG_RESULT_SHIFT = 0;
/// Full-step active indicator (bit 15).
constexpr uint32_t DRV_STATUS_FSACTIVE = 1u << 15;
/// Actual motor current / smart energy current level (bits 20:16).
constexpr uint32_t DRV_STATUS_CS_ACTUAL_MASK = 0x1Fu << 16;
/// Shift helper for @ref DRV_STATUS_CS_ACTUAL_MASK.
constexpr uint32_t DRV_STATUS_CS_ACTUAL_SHIFT = 16;
/// StallGuard status flag (bit 24).
constexpr uint32_t DRV_STATUS_STALLGUARD = 1u << 24;
/// Overtemperature flag (bit 25).
constexpr uint32_t DRV_STATUS_OT = 1u << 25;
/// Overtemperature prewarning (bit 26).
constexpr uint32_t DRV_STATUS_OTPW = 1u << 26;
/// Short to ground phase A (bit 27).
constexpr uint32_t DRV_STATUS_S2GA = 1u << 27;
/// Short to ground phase B (bit 28).
constexpr uint32_t DRV_STATUS_S2GB = 1u << 28;
/// Open load indicator phase A (bit 29).
constexpr uint32_t DRV_STATUS_OLA = 1u << 29;
/// Open load indicator phase B (bit 30).
constexpr uint32_t DRV_STATUS_OLB = 1u << 30;
/// Standstill indicator (bit 31).
constexpr uint32_t DRV_STATUS_STST = 1u << 31;
///@}

} // namespace tmc2130
