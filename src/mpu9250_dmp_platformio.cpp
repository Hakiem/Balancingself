#include "i2c_bridge.hpp"

static I2CBridge* g_bridge = nullptr;

extern "C" {
    void mpu9250_set_i2c_bridge(I2CBridge* bridge) { g_bridge = bridge; }

    int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char* data) {
        if (!g_bridge) return -1;
        return g_bridge->writeRegisters(slave_addr, reg_addr, data, length) ? 0 : -1;
    }
    int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char* data) {
        if (!g_bridge) return -1;
        return g_bridge->readRegisters(slave_addr, reg_addr, data, length) ? 0 : -1;
    }
}
