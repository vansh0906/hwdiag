#pragma once
// Abstract I2C bus. Every driver and test talks to hardware only through this
// interface, so the same code runs against a real Linux adapter (/dev/i2c-N)
// or the simulated sensor board used for development and CI.

#include <cstddef>
#include <cstdint>
#include <string>

namespace hwdiag {

enum class BusStatus {
    Ok,       // transfer completed
    Nack,     // no device acknowledged the address (missing / unpowered / wrong address)
    Timeout,  // adapter timed out (bus stuck low, clock stretching too long)
    IoError,  // any other adapter or driver error
};

const char* to_string(BusStatus s);

class I2CBus {
public:
    virtual ~I2CBus() = default;

    // Register read: write the register pointer, repeated START, read `len` bytes.
    virtual BusStatus read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) = 0;

    // Register write: write the register pointer followed by `len` data bytes.
    virtual BusStatus write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) = 0;

    // Returns true if a device acknowledges `addr`.
    virtual bool probe(uint8_t addr) = 0;

    virtual std::string name() const = 0;

    BusStatus read_u8(uint8_t addr, uint8_t reg, uint8_t& value) { return read_regs(addr, reg, &value, 1); }
    BusStatus write_u8(uint8_t addr, uint8_t reg, uint8_t value) { return write_regs(addr, reg, &value, 1); }
};

}  // namespace hwdiag
