#pragma once
// Real-hardware backend: talks to a Linux I2C adapter through the i2c-dev
// character device (/dev/i2c-N) using the I2C_RDWR ioctl, so a register read is
// one combined write + repeated-START + read transaction.
// Only built on Linux (e.g. Raspberry Pi). macOS builds use the simulator.

#ifdef __linux__

#include "hwdiag/i2c_bus.hpp"

#include <string>

namespace hwdiag {

class LinuxI2CBus : public I2CBus {
public:
    explicit LinuxI2CBus(const std::string& device_path);  // throws std::runtime_error
    ~LinuxI2CBus() override;

    LinuxI2CBus(const LinuxI2CBus&) = delete;
    LinuxI2CBus& operator=(const LinuxI2CBus&) = delete;

    BusStatus read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) override;
    BusStatus write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) override;
    bool probe(uint8_t addr) override;
    std::string name() const override { return path_; }

private:
    std::string path_;
    int fd_ = -1;
};

}  // namespace hwdiag

#endif  // __linux__
