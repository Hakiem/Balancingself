#pragma once
#include "stm32f3xx_hal.h"
#include "uart_bridge.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

// ---------- CRC8 (TMC22xx spec: CRC8-ATM, LSB-first, polynomial 0x07)
inline uint8_t crc8_tmc(const uint8_t* bytes, size_t n)
{
    uint8_t crc = 0;
    while (n--) {
        uint8_t current = *bytes++;
        for (int i = 0; i < 8; ++i) {
            if (((crc >> 7) & 1u) ^ (current & 1u)) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07u);
            } else {
                crc <<= 1;
            }
            current >>= 1;
        }
    }
    return crc;
}

// ===================== TMC2208 UART Transport =====================
// NOTE: This version uses UartBridge (class-based, half-duplex aware) instead
// of free functions.
class TMC2208_UART_Transport
{
public:
    struct Config {
        uint8_t retries; // how many extra tries on CRC/short RX
        uint32_t read_timeout_us; // per-try RX timeout
        bool verify_write; // read back after write (when possible)

        Config()
            : retries(1)
            , read_timeout_us(2000)
            , verify_write(false)
        {
        }
    };

    // slave_addr is 7-bit TMC node (almost always 0x00 unless you wired
    // multiple)
    explicit TMC2208_UART_Transport(
        UART_Bridge* bridge, uint8_t slave_addr = 0x00, Config cfg = Config())
        : bridge_(bridge)
        , slave_(slave_addr)
        , cfg_(cfg)
    {
    }

    void setSlave(uint8_t addr) { slave_ = addr; }
    uint8_t getSlave() const { return slave_; }
    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& getConfig() const { return cfg_; }

    // Read 32-bit register (7-bit addr, raw) -> fills 'out' on success
    bool regRead(uint8_t reg, uint32_t& out, uint32_t timeout_us_override = 0)
    {
        const uint32_t timeout
            = timeout_us_override ? timeout_us_override : cfg_.read_timeout_us;

        // Request: [0x05][slave][reg][crc]
        uint8_t req[4] = { 0x05, slave_, static_cast<uint8_t>(reg & 0x7F), 0 };
        req[3] = crc8_tmc(req, 3);

        for (uint8_t attempt = 0; attempt <= cfg_.retries; ++attempt) {
            if (!bridge_->transmit(req, sizeof(req)))
                continue;

            // Response: [0x05][slave][reg][D3][D2][D1][D0][crc]
            uint8_t resp[8] = { 0 };
            size_t got = bridge_->receive(resp, sizeof(resp), timeout);
            if (got != sizeof(resp))
                continue;
            if (resp[0] != 0x05 || resp[1] != slave_ || resp[2] != (reg & 0x7F))
                continue;
            if (crc8_tmc(resp, 7) != resp[7])
                continue;

            out = (uint32_t(resp[3]) << 24) | (uint32_t(resp[4]) << 16)
                | (uint32_t(resp[5]) << 8) | uint32_t(resp[6]);
            return true;
        }
        return false;
    }

    // Write 32-bit register (7-bit addr, raw)
    // If verify_write=true, it will read back and check equality (useful on RW
    // regs).
    bool regWrite(uint8_t reg, uint32_t val, uint32_t timeout_us_override = 0)
    {
        (void)timeout_us_override; // write has no ACK; timeout only used for
                                   // optional verify

        // Write: [0x05][slave][reg|0x80][D3][D2][D1][D0][crc]
        uint8_t frm[8];
        frm[0] = 0x05;
        frm[1] = slave_;
        frm[2] = static_cast<uint8_t>((reg & 0x7F) | 0x80);
        frm[3] = static_cast<uint8_t>(val >> 24);
        frm[4] = static_cast<uint8_t>(val >> 16);
        frm[5] = static_cast<uint8_t>(val >> 8);
        frm[6] = static_cast<uint8_t>(val >> 0);
        frm[7] = crc8_tmc(frm, 7);

        if (!bridge_->transmit(frm, sizeof(frm)))
            return false;

        if (!cfg_.verify_write)
            return true;

        // Optional verify: try to read back. (If it's a W-only reg, this will
        // just fail gracefully.)
        uint32_t rb = 0;
        if (regRead(static_cast<uint8_t>(reg & 0x7F), rb)) {
            return (rb == val);
        }
        return true; // can't verify (likely W-only); consider this success
    }

    // Helper: set NODECONF (0x03) with address and senddelay (bits [11:8]).
    // Call while only ONE device is enabled on the bus.
    bool programNodeConf(uint8_t new_node_addr, uint8_t senddelay_0_15 = 0)
    {
        uint32_t v = (uint32_t(new_node_addr) & 0x7F)
            | (uint32_t(senddelay_0_15 & 0x0F) << 8);
        return regWrite(0x03 /*NODECONF*/, v);
    }

private:
    UART_Bridge* bridge_;
    uint8_t slave_;
    Config cfg_;
};

// ========================= Register map (addresses) =========================
namespace TMC2208
{
constexpr uint8_t GCONF = 0x00; // RW
constexpr uint8_t GSTAT = 0x01; // R+C
constexpr uint8_t IFCNT = 0x02; // R
constexpr uint8_t NODECONF = 0x03; // W
constexpr uint8_t OTP_PROG = 0x04; // W
constexpr uint8_t OTP_READ = 0x05; // R
constexpr uint8_t IOIN = 0x06; // R
constexpr uint8_t FACTORY_CONF = 0x07; // RW

constexpr uint8_t IHOLD_IRUN = 0x10; // W
constexpr uint8_t TPOWERDOWN = 0x11; // W
constexpr uint8_t TSTEP = 0x12; // R
constexpr uint8_t TPWMTHRS = 0x13; // W
constexpr uint8_t TCOOLTHRS = 0x14; // W
constexpr uint8_t THIGH = 0x15; // W
constexpr uint8_t VACTUAL = 0x22; // W

constexpr uint8_t MSCNT = 0x6A; // R
constexpr uint8_t MSCURACT = 0x6B; // R

constexpr uint8_t CHOPCONF = 0x6C; // RW
constexpr uint8_t DRV_STATUS = 0x6F; // R
constexpr uint8_t PWMCONF = 0x70; // RW
constexpr uint8_t PWM_SCALE = 0x71; // R
constexpr uint8_t PWM_AUTO = 0x72; // R

// field helpers
constexpr uint32_t mask_range(unsigned msb, unsigned lsb)
{
    return (msb < lsb) ? 0u : ((0xFFFFFFFFu >> (31u - (msb - lsb))) << lsb);
}
inline uint32_t set_field(
    uint32_t reg, uint32_t msk, unsigned pos, uint32_t val)
{
    return (reg & ~msk) | ((val << pos) & msk);
}
inline uint32_t get_field(uint32_t reg, uint32_t msk, unsigned pos)
{
    return (reg & msk) >> pos;
}

// ---- GCONF bits (RW) ----
namespace gconf
{
    enum : uint32_t {
        I_SCALE_ANALOG = 1u << 0,
        INTERNAL_RSENSE = 1u << 1,
        EN_SPREADCYCLE = 1u << 2,
        SHAFT = 1u << 3,
        INDEX_OTPW = 1u << 4,
        INDEX_STEP = 1u << 5,
        PDN_DISABLE = 1u << 6,
        MSTEP_REG_SELECT = 1u << 7,
        MULTISTEP_FILT = 1u << 8,
        TEST_MODE = 1u << 9
    };
}

// ---- FACTORY_CONF (RW) ----
namespace factory_conf
{
    constexpr unsigned FCLKTRIM_Pos = 0;
    constexpr uint32_t FCLKTRIM_Msk = mask_range(4, 0);
    constexpr unsigned OTTRIM_Pos = 8;
    constexpr uint32_t OTTRIM_Msk = mask_range(9, 8);
    enum OTTRIM : uint32_t {
        OT_143C_120C = 0,
        OT_150C_120C = 1,
        OT_150C_143C = 2,
        OT_157C_143C = 3
    };
}

// ---- CHOPCONF (RW) ----
namespace chopconf
{
    enum : uint32_t {
        DISS2VS = 1u << 31,
        DISS2G = 1u << 30,
        DEDGE = 1u << 29,
        INTPOL = 1u << 28
    };
    constexpr unsigned MRES_Pos = 24;
    constexpr uint32_t MRES_Msk = mask_range(27, 24);
    enum MRES : uint32_t {
        _256 = 0,
        _128 = 1,
        _64 = 2,
        _32 = 3,
        _16 = 4,
        _8 = 5,
        _4 = 6,
        _2 = 7,
        _1 = 8
    };
    constexpr unsigned VSENSE_Pos = 17;
    constexpr uint32_t VSENSE_Msk = 1u << VSENSE_Pos;
    constexpr unsigned TBL_Pos = 15;
    constexpr uint32_t TBL_Msk = mask_range(16, 15);
    constexpr unsigned HEND_Pos = 8;
    constexpr uint32_t HEND_Msk = mask_range(11, 8);
    constexpr unsigned HSTRT_Pos = 4;
    constexpr uint32_t HSTRT_Msk = mask_range(6, 4);
    constexpr unsigned TOFF_Pos = 0;
    constexpr uint32_t TOFF_Msk = mask_range(3, 0);
}

// ---- PWMCONF (RW) ----
namespace pwmconf
{
    constexpr unsigned FREEWHEEL_Pos = 20;
    constexpr uint32_t FREEWHEEL_Msk = mask_range(21, 20);
    enum Freewheel : uint32_t {
        FW_NORMAL = 0b00,
        FW_PASSIVE = 0b01,
        FW_FREEWHEEL = 0b10,
        FW_RESERVED = 0b11
    };
    constexpr unsigned PWM_AUTOGRAD_Pos = 19;
    constexpr uint32_t PWM_AUTOGRAD_Msk = 1u << PWM_AUTOGRAD_Pos;
    constexpr unsigned PWM_AUTOSCALE_Pos = 18;
    constexpr uint32_t PWM_AUTOSCALE_Msk = 1u << PWM_AUTOSCALE_Pos;
    constexpr unsigned PWM_FREQ_Pos = 16;
    constexpr uint32_t PWM_FREQ_Msk = mask_range(17, 16);
    enum Freq : uint32_t { FREQ_2 = 0, FREQ_4 = 1, FREQ_8 = 2, FREQ_12 = 3 };
    constexpr unsigned PWM_GRAD_Pos = 8;
    constexpr uint32_t PWM_GRAD_Msk = mask_range(15, 8);
    constexpr unsigned PWM_OFS_Pos = 0;
    constexpr uint32_t PWM_OFS_Msk = mask_range(7, 0);
}

// ---- DRV_STATUS (R) ----
namespace drv_status
{
    constexpr unsigned STST_Pos = 31;
    constexpr uint32_t STST_Msk = 1u << STST_Pos;
    constexpr unsigned STEALTH_Pos = 30;
    constexpr uint32_t STEALTH_Msk = 1u << STEALTH_Pos;
    constexpr unsigned CS_ACTUAL_Pos = 16;
    constexpr uint32_t CS_ACTUAL_Msk = mask_range(20, 16);
    constexpr unsigned T157_Pos = 11, T150_Pos = 10, T143_Pos = 9, T120_Pos = 8;
    constexpr uint32_t T157_Msk = 1u << T157_Pos, T150_Msk = 1u << T150_Pos,
                       T143_Msk = 1u << T143_Pos, T120_Msk = 1u << T120_Pos;
    constexpr unsigned OLB_Pos = 7, OLA_Pos = 6, S2VSB_Pos = 5, S2VSA_Pos = 4,
                       S2GB_Pos = 3, S2GA_Pos = 2;
    constexpr uint32_t OLB_Msk = 1u << OLB_Pos, OLA_Msk = 1u << OLA_Pos,
                       S2VSB_Msk = 1u << S2VSB_Pos, S2VSA_Msk = 1u << S2VSA_Pos,
                       S2GB_Msk = 1u << S2GB_Pos, S2GA_Msk = 1u << S2GA_Pos;
}

// ========================= High-level Device (uses transport)
// =========================
class Device
{
public:
    explicit Device(TMC2208_UART_Transport& t)
        : t_(t)
    {
    }

    // --- RW helpers (read-modify-write) ---
    bool gconf_or(uint32_t mask)
    {
        uint32_t v;
        if (!rd(GCONF, v))
            return false;
        v |= mask;
        return wr(GCONF, v);
    }
    bool gconf_andc(uint32_t mask)
    {
        uint32_t v;
        if (!rd(GCONF, v))
            return false;
        v &= ~mask;
        return wr(GCONF, v);
    }

    bool factory_set_fclktrim(uint8_t v5)
    {
        uint32_t v;
        if (!rd(FACTORY_CONF, v))
            return false;
        v = set_field(v, factory_conf::FCLKTRIM_Msk, factory_conf::FCLKTRIM_Pos,
            v5 & 0x1F);
        return wr(FACTORY_CONF, v);
    }
    bool factory_set_ottrim(factory_conf::OTTRIM ot)
    {
        uint32_t v;
        if (!rd(FACTORY_CONF, v))
            return false;
        v = set_field(v, factory_conf::OTTRIM_Msk, factory_conf::OTTRIM_Pos,
            (uint32_t)ot);
        return wr(FACTORY_CONF, v);
    }

    bool chop_set(uint32_t mask)
    {
        uint32_t v;
        if (!rd(CHOPCONF, v))
            return false;
        v |= mask;
        return wr(CHOPCONF, v);
    }
    bool chop_clr(uint32_t mask)
    {
        uint32_t v;
        if (!rd(CHOPCONF, v))
            return false;
        v &= ~mask;
        return wr(CHOPCONF, v);
    }
    bool chop_set_mres(chopconf::MRES m)
    {
        return setf(
            CHOPCONF, chopconf::MRES_Msk, chopconf::MRES_Pos, (uint32_t)m);
    }
    bool chop_set_tbl(uint8_t code)
    {
        return setf(CHOPCONF, chopconf::TBL_Msk, chopconf::TBL_Pos, code & 0x3);
    }
    bool chop_set_vsense(bool hi)
    {
        return setf(
            CHOPCONF, chopconf::VSENSE_Msk, chopconf::VSENSE_Pos, hi ? 1u : 0u);
    }
    bool chop_set_hend(uint8_t v4)
    {
        return setf(CHOPCONF, chopconf::HEND_Msk, chopconf::HEND_Pos, v4 & 0xF);
    }
    bool chop_set_hstrt(uint8_t v3)
    {
        return setf(
            CHOPCONF, chopconf::HSTRT_Msk, chopconf::HSTRT_Pos, v3 & 0x7);
    }
    bool chop_set_toff(uint8_t v4)
    {
        return setf(CHOPCONF, chopconf::TOFF_Msk, chopconf::TOFF_Pos, v4 & 0xF);
    }

    bool pwm_set_autoscale(bool en)
    {
        return setf(PWMCONF, pwmconf::PWM_AUTOSCALE_Msk,
            pwmconf::PWM_AUTOSCALE_Pos, en ? 1u : 0u);
    }
    bool pwm_set_autograd(bool en)
    {
        return setf(PWMCONF, pwmconf::PWM_AUTOGRAD_Msk,
            pwmconf::PWM_AUTOGRAD_Pos, en ? 1u : 0u);
    }
    bool pwm_set_freewheel(pwmconf::Freewheel fw)
    {
        return setf(PWMCONF, pwmconf::FREEWHEEL_Msk, pwmconf::FREEWHEEL_Pos,
            (uint32_t)fw);
    }
    bool pwm_set_freq(pwmconf::Freq f)
    {
        return setf(
            PWMCONF, pwmconf::PWM_FREQ_Msk, pwmconf::PWM_FREQ_Pos, (uint32_t)f);
    }
    bool pwm_set_grad(uint8_t v)
    {
        return setf(PWMCONF, pwmconf::PWM_GRAD_Msk, pwmconf::PWM_GRAD_Pos, v);
    }
    bool pwm_set_ofs(uint8_t v)
    {
        return setf(PWMCONF, pwmconf::PWM_OFS_Msk, pwmconf::PWM_OFS_Pos, v);
    }

    // --- W-only setters (convenience) ---
    bool ihold_irun(uint8_t IHOLD, uint8_t IRUN, uint8_t IHOLDDELAY)
    {
        uint32_t v = 0;
        v = set_field(v, mask_range(4, 0), 0, IHOLD & 0x1F);
        v = set_field(v, mask_range(12, 8), 8, IRUN & 0x1F);
        v = set_field(v, mask_range(19, 16), 16, IHOLDDELAY & 0x0F);
        return wr(IHOLD_IRUN, v);
    }
    bool tpowerdown(uint8_t v8) { return wr(TPOWERDOWN, v8 & 0xFF); }
    bool tpwmthrs(uint32_t v20) { return wr(TPWMTHRS, v20 & 0xFFFFF); }
    bool tcoolthrs(uint32_t v20) { return wr(TCOOLTHRS, v20 & 0xFFFFF); }
    bool thigh(uint32_t v20) { return wr(THIGH, v20 & 0xFFFFF); }
    bool vactual(int32_t v24)
    {
        return wr(VACTUAL, (uint32_t)(v24 & 0xFFFFFF));
    }

    // --- R reads (helpers) ---
    bool ifcnt(uint8_t& out)
    {
        uint32_t v;
        if (!rd(IFCNT, v))
            return false;
        out = uint8_t(v & 0xFF);
        return true;
    }
    bool drv_status_raw(uint32_t& v) { return rd(DRV_STATUS, v); }

private:
    TMC2208_UART_Transport& t_;
    bool rd(uint8_t r, uint32_t& out) { return t_.regRead(r, out); }
    bool wr(uint8_t r, uint32_t v) { return t_.regWrite(r, v); }
    bool setf(uint8_t reg, uint32_t msk, unsigned pos, uint32_t val)
    {
        uint32_t v;
        if (!rd(reg, v))
            return false;
        v = set_field(v, msk, pos, val);
        return wr(reg, v);
    }
};

} // namespace TMC2208