// tmc2208_uart.hpp - header-only helper for TMC2208 UART protocol + register
// access
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

/* ---------- Minimal register map (extend if you like) ---------- */
namespace tmc2208
{

constexpr uint8_t REG_GCONF = 0x00; // RW
constexpr uint8_t REG_GSTAT = 0x01; // R, write '1' to clear
constexpr uint8_t REG_IFCNT = 0x02; // R
constexpr uint8_t REG_NODECONF = 0x03; // W  (reply delay)
constexpr uint8_t REG_IHOLD_IRUN = 0x10; // W
constexpr uint8_t REG_TPOWERDOWN = 0x11; // W
constexpr uint8_t REG_TSTEP = 0x12; // R
constexpr uint8_t REG_TPWMTHRS = 0x13; // W
constexpr uint8_t REG_VACTUAL = 0x22; // W (signed 24-bit)
constexpr uint8_t REG_MSCNT = 0x6A; // R
constexpr uint8_t REG_MSCURACT = 0x6B; // R
constexpr uint8_t REG_CHOPCONF = 0x6C; // RW
constexpr uint8_t REG_DRV_STATUS = 0x6F; // R
constexpr uint8_t REG_PWMCONF = 0x70; // RW
constexpr uint8_t REG_PWM_SCALE = 0x71; // R
constexpr uint8_t REG_PWM_AUTO = 0x72; // R
constexpr uint8_t REG_IOIN = 0x06; // R

// ---- GCONF bits (subset you’ll commonly use)
constexpr uint32_t GCONF_EN_SPREADCYCLE = (1u << 2);
constexpr uint32_t GCONF_SHAFT = (1u << 3);
constexpr uint32_t GCONF_PDN_DISABLE = (1u << 6);
constexpr uint32_t GCONF_MSTEP_REG_SELECT = (1u << 7);
constexpr uint32_t GCONF_MULTISTEP_FILT = (1u << 8);

// ---- CHOPCONF key fields
constexpr uint32_t CHOPCONF_INTPOL = (1u << 28);
constexpr unsigned CHOPCONF_MRES_SHIFT = 24; // 0:256 … 8:full
constexpr uint32_t CHOPCONF_MRES_MASK = (0xFu << CHOPCONF_MRES_SHIFT);
constexpr uint32_t CHOPCONF_VSENSE = (1u << 17);
constexpr unsigned CHOPCONF_TBL_SHIFT = 15; // 0..3
constexpr uint32_t CHOPCONF_TBL_MASK = (0x3u << CHOPCONF_TBL_SHIFT);
constexpr unsigned CHOPCONF_HEND_SHIFT = 8; // 0..15
constexpr uint32_t CHOPCONF_HEND_MASK = (0xFu << CHOPCONF_HEND_SHIFT);
constexpr unsigned CHOPCONF_HSTRT_SHIFT = 4; // 0..7
constexpr uint32_t CHOPCONF_HSTRT_MASK = (0x7u << CHOPCONF_HSTRT_SHIFT);
constexpr uint32_t CHOPCONF_TOFF_MASK = 0xFu; // 1..15 valid (0 disables driver)

// ---- GSTAT bits
constexpr uint32_t GSTAT_RESET = (1u << 0);
constexpr uint32_t GSTAT_DRV_ERR = (1u << 1);
constexpr uint32_t GSTAT_UV_CP = (1u << 2);

// ---- IFCNT mask
constexpr uint32_t IFCNT_MASK = 0xFFu;

// ---- MSCNT mask
constexpr uint32_t MSCNT_MASK = 0x3FFu;

// ---- DRV_STATUS bits
constexpr uint32_t DRV_STATUS_STST = (1u << 31);
constexpr uint32_t DRV_STATUS_OTS = (1u << 26);
constexpr uint32_t DRV_STATUS_OTPW = (1u << 25);
constexpr uint32_t DRV_STATUS_S2GA = (1u << 24);
constexpr uint32_t DRV_STATUS_S2GB = (1u << 23);
constexpr uint32_t DRV_STATUS_S2VSA = (1u << 22);
constexpr uint32_t DRV_STATUS_S2VSB = (1u << 21);
constexpr uint32_t DRV_STATUS_OLA = (1u << 18);
constexpr uint32_t DRV_STATUS_OLB = (1u << 17);
constexpr uint32_t DRV_STATUS_T120 = (1u << 19);
constexpr uint32_t DRV_STATUS_T143 = (1u << 20);
constexpr uint32_t DRV_STATUS_STA = (1u << 12);
// ---- UART framing helpers
constexpr uint8_t SYNC_BASE = 0x05; // Sync byte that starts every frame

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

// ---------- Driver (transport provided by user) ----------
class Driver
{
public:
    enum class Status {
        Ok = 0,
        ShortReply,
        BadHeader,
        CrcMismatch,
        Transport,
        InvalidParam
    };

    using TxFunc = std::function<void(const uint8_t*, size_t)>;
    using RxFunc = std::function<size_t(uint8_t*, size_t, uint32_t)>;

    explicit Driver(TxFunc tx, RxFunc rx, uint8_t slave_address = 0)
        : tx_(std::move(tx))
        , rx_(std::move(rx))
    {
        set_address(slave_address);
    }

    Status last_status() const { return last_status_; }
    static const char* status_cstr(Status status)
    {
        switch (status) {
        case Status::Ok:
            return "ok";
        case Status::ShortReply:
            return "short reply";
        case Status::BadHeader:
            return "bad header";
        case Status::CrcMismatch:
            return "crc mismatch";
        case Status::Transport:
            return "transport error";
        case Status::InvalidParam:
            return "invalid parameter";
        default:
            return "unknown";
        }
    }

    bool set_address(uint8_t slave_address)
    {
        last_status_ = Status::Ok;
        if (slave_address > 3) {
            last_status_ = Status::InvalidParam;
            return false;
        }
        address_ = slave_address;
        return true;
    }
    uint8_t address() const { return address_; }

    // ---- Low-level: build + send a WRITE frame (reg + 32-bit LE value)
    void write_reg(uint8_t reg, uint32_t value)
    {
        last_status_ = Status::Ok;
        std::array<uint8_t, 8> f {};
        f[0] = SYNC_BASE;
        f[1] = address_;
        f[2] = static_cast<uint8_t>(reg | 0x80u); // write access
        f[3] = static_cast<uint8_t>(value & 0xFF);
        f[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
        f[5] = static_cast<uint8_t>((value >> 16) & 0xFF);
        f[6] = static_cast<uint8_t>((value >> 24) & 0xFF);
        f[7] = crc8_tmc(f.data(), 7);
        tx_(f.data(), f.size());
    }

    // ---- Low-level: send a READ request, then receive & verify reply (returns
    // 32-bit LE value)
    uint32_t read_reg(uint8_t reg, uint32_t timeout_us = 2000)
    {
        last_status_ = Status::Ok;
        std::array<uint8_t, 4> req {};
        req[0] = SYNC_BASE;
        req[1] = address_;
        req[2] = static_cast<uint8_t>(reg & 0x7Fu); // read request
        req[3] = crc8_tmc(req.data(), 3);
        tx_(req.data(), req.size());

        std::array<uint8_t, 8> rep {};
        size_t got = rx_(rep.data(), rep.size(), timeout_us);
        if (got != rep.size()) {
            last_status_ = Status::ShortReply;
            return 0;
        }
        const uint8_t expected_header = static_cast<uint8_t>(address_ | 0x80u);
        const bool header_matches = (rep[0] == SYNC_BASE)
            && (rep[1] == expected_header || rep[1] == 0xFF);
        if (!header_matches) {
            last_status_ = Status::BadHeader;
            return 0;
        }
        uint8_t expect = crc8_tmc(rep.data(), 7);
        if (expect != rep[7]) {
            last_status_ = Status::CrcMismatch;
            return 0;
        }
        return (uint32_t(rep[3])) | ((uint32_t(rep[4]) << 8))
            | ((uint32_t(rep[5]) << 16)) | ((uint32_t(rep[6]) << 24));
    }

    // ---- Read-modify-write helper
    void rmw(uint8_t reg, uint32_t clear_mask, uint32_t set_bits)
    {
        uint32_t v = read_reg(reg);
        if (last_status_ != Status::Ok)
            return;
        v &= ~clear_mask;
        v |= set_bits;
        write_reg(reg, v);
    }

    // ================== Convenience APIs ==================

    // Enable single-wire UART control (disable PDN mode) and select MRES via
    // CHOPCONF
    bool enable_uart_control()
    {
        uint32_t gconf = read_reg(REG_GCONF);
        if (last_status_ != Status::Ok)
            return false;
        gconf |= (GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT);
        write_reg(REG_GCONF, gconf);
        return last_status_ == Status::Ok;
    }

    // Set microstep resolution via CHOPCONF.MRES (0:256, 1:128, … 8:full)
    bool set_microsteps_code(uint8_t mres_code)
    {
        uint32_t ch = read_reg(REG_CHOPCONF);
        if (last_status_ != Status::Ok)
            return false;
        ch = (ch & ~CHOPCONF_MRES_MASK)
            | ((uint32_t(mres_code) << CHOPCONF_MRES_SHIFT)
                & CHOPCONF_MRES_MASK);
        write_reg(REG_CHOPCONF, ch);
        return last_status_ == Status::Ok;
    }

    // Handy translator: 256,128,64,32,16,8,4,2,1 → code (0..8)
    static bool microsteps_to_code(unsigned usteps, uint8_t& code)
    {
        switch (usteps) {
        case 256:
            code = 0;
            return true;
        case 128:
            code = 1;
            return true;
        case 64:
            code = 2;
            return true;
        case 32:
            code = 3;
            return true;
        case 16:
            code = 4;
            return true;
        case 8:
            code = 5;
            return true;
        case 4:
            code = 6;
            return true;
        case 2:
            code = 7;
            return true;
        case 1:
            code = 8;
            return true;
        }
        code = 0;
        return false;
    }

    // IHOLD/IRUN/IHOLDDELAY (5/5/4 bits)
    void set_currents(uint8_t ihold, uint8_t irun, uint8_t iholddelay)
    {
        uint32_t v = (uint32_t(ihold) & 0x1F) | ((uint32_t(irun) & 0x1F) << 8)
            | ((uint32_t(iholddelay) & 0x0F) << 16);
        write_reg(REG_IHOLD_IRUN, v);
    }

    // Set chopper essentials: toff(1..15), hstrt(0..7), hend(0..15), tbl(0..3),
    // intpol on/off, vsense on/off
    bool config_chopper(uint8_t toff, uint8_t hstrt, uint8_t hend, uint8_t tbl,
        bool intpol = true, bool vsense = false)
    {
        if (toff == 0 || toff > 15) {
            last_status_ = Status::InvalidParam;
            return false;
        }
        uint32_t ch = read_reg(REG_CHOPCONF);
        if (last_status_ != Status::Ok)
            return false;
        ch &= ~(CHOPCONF_TOFF_MASK | CHOPCONF_HSTRT_MASK | CHOPCONF_HEND_MASK
            | CHOPCONF_TBL_MASK | CHOPCONF_VSENSE | CHOPCONF_INTPOL);
        ch |= (toff & 0x0F);
        ch |= (uint32_t(hstrt & 0x07) << CHOPCONF_HSTRT_SHIFT);
        ch |= (uint32_t(hend & 0x0F) << CHOPCONF_HEND_SHIFT);
        ch |= (uint32_t(tbl & 0x03) << CHOPCONF_TBL_SHIFT);
        if (intpol)
            ch |= CHOPCONF_INTPOL;
        if (vsense)
            ch |= CHOPCONF_VSENSE;
        write_reg(REG_CHOPCONF, ch);
        return last_status_ == Status::Ok;
    }

    // Enable/disable SpreadCycle (vs StealthChop preference flag)
    bool set_spreadcycle(bool enable)
    {
        uint32_t g = read_reg(REG_GCONF);
        if (last_status_ != Status::Ok)
            return false;
        g = enable ? (g | GCONF_EN_SPREADCYCLE) : (g & ~GCONF_EN_SPREADCYCLE);
        write_reg(REG_GCONF, g);
        return last_status_ == Status::Ok;
    }

    // Velocity for internal step generator (signed 24-bit)
    void set_vactual(int32_t v)
    {
        uint32_t u = static_cast<uint32_t>(v) & 0x00FFFFFFu;
        write_reg(REG_VACTUAL, u);
    }

    // TPWMTHRS (threshold to switch to StealthChop): units = TSTEP
    void set_tpwmthrs(uint16_t t) { write_reg(REG_TPWMTHRS, t); }

    // Read microstep counter (0..1023)
    uint16_t mscnt()
    {
        return static_cast<uint16_t>(read_reg(REG_MSCNT) & 0x3FFu);
    }

    // Read DRV_STATUS (raw)
    uint32_t drv_status() { return read_reg(REG_DRV_STATUS); }

private:
    TxFunc tx_;
    RxFunc rx_;
    Status last_status_ = Status::Ok;
    uint8_t address_ = 0;
};

} // namespace tmc2208
