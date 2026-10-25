#pragma once
// Device drivers built on the I2CBus abstraction, plus a retry layer that
// counts retries so marginal units (intermittent bus errors) are visible in
// test logs even when they pass.

#include "hwdiag/i2c_bus.hpp"
#include "hwdiag/types.hpp"

#include <cstdint>
#include <optional>

namespace hwdiag {

struct BusStats {
    uint64_t transfers = 0;  // attempts issued, including retries
    uint64_t retries = 0;    // attempts that had to be repeated
    uint64_t failures = 0;   // operations that failed after all retries
};

class RetryingBus {
public:
    RetryingBus(I2CBus& bus, int max_retries, BusStats& stats);

    BusStatus read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len);
    BusStatus write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len);
    BusStatus read_u8(uint8_t addr, uint8_t reg, uint8_t& value) { return read_regs(addr, reg, &value, 1); }
    BusStatus write_u8(uint8_t addr, uint8_t reg, uint8_t value) { return write_regs(addr, reg, &value, 1); }
    bool probe(uint8_t addr);

    I2CBus& raw() { return bus_; }

private:
    template <class Fn>
    BusStatus with_retry(Fn&& fn);

    I2CBus& bus_;
    int max_retries_;
    BusStats& stats_;
};

class Tmp102 {
public:
    static constexpr uint8_t kDefaultAddr = 0x48;
    explicit Tmp102(RetryingBus& bus, uint8_t addr = kDefaultAddr) : bus_(bus), addr_(addr) {}

    std::optional<double> read_celsius();
    // 12-bit two's complement, left-justified in 16 bits, 0.0625 C per LSB.
    static double raw_to_celsius(uint8_t msb, uint8_t lsb);

private:
    RetryingBus& bus_;
    uint8_t addr_;
};

struct ImuSample {
    Vec3 accel_g;
    Vec3 gyro_dps;
    double temp_c = 0.0;
};

class Mpu6050 {
public:
    explicit Mpu6050(RetryingBus& bus, uint8_t addr = 0x68) : bus_(bus), addr_(addr) {}

    std::optional<uint8_t> who_am_i();
    // Wake from sleep, select PLL clock, +-2 g and +-250 dps ranges, verify by readback.
    bool init();
    // One 14-byte burst read: accel, die temperature, gyro.
    std::optional<ImuSample> read_sample();

private:
    RetryingBus& bus_;
    uint8_t addr_;
};

}  // namespace hwdiag
