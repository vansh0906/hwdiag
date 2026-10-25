#include "hwdiag/linux_i2c_bus.hpp"

#ifdef __linux__

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace hwdiag {

namespace {

BusStatus map_errno(int err) {
    switch (err) {
        case ENXIO:      // most adapters: address not acknowledged
        case EREMOTEIO:  // some adapters (e.g. bcm2835 on Raspberry Pi) report NACK this way
            return BusStatus::Nack;
        case ETIMEDOUT:
            return BusStatus::Timeout;
        default:
            return BusStatus::IoError;
    }
}

}  // namespace

LinuxI2CBus::LinuxI2CBus(const std::string& device_path) : path_(device_path) {
    fd_ = ::open(path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error("cannot open " + path_ + ": " + std::strerror(errno) +
                                 " (is i2c-dev loaded and are you in the i2c group?)");
    }
    unsigned long funcs = 0;
    if (::ioctl(fd_, I2C_FUNCS, &funcs) < 0 || (funcs & I2C_FUNC_I2C) == 0) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(path_ + ": adapter does not support combined I2C transfers (I2C_FUNC_I2C)");
    }
}

LinuxI2CBus::~LinuxI2CBus() {
    if (fd_ >= 0) ::close(fd_);
}

BusStatus LinuxI2CBus::read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) {
    if (len == 0 || len > 0xFFFF) return BusStatus::IoError;
    uint8_t pointer = reg;
    i2c_msg msgs[2] = {};
    msgs[0].addr = addr;
    msgs[0].flags = 0;  // write register pointer
    msgs[0].len = 1;
    msgs[0].buf = &pointer;
    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD;  // repeated START, then read
    msgs[1].len = static_cast<uint16_t>(len);
    msgs[1].buf = out;

    i2c_rdwr_ioctl_data xfer = {};
    xfer.msgs = msgs;
    xfer.nmsgs = 2;
    if (::ioctl(fd_, I2C_RDWR, &xfer) < 0) return map_errno(errno);
    return BusStatus::Ok;
}

BusStatus LinuxI2CBus::write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) {
    if (len + 1 > 0xFFFF) return BusStatus::IoError;
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    if (len > 0) std::memcpy(buf.data() + 1, data, len);

    i2c_msg msg = {};
    msg.addr = addr;
    msg.flags = 0;
    msg.len = static_cast<uint16_t>(buf.size());
    msg.buf = buf.data();

    i2c_rdwr_ioctl_data xfer = {};
    xfer.msgs = &msg;
    xfer.nmsgs = 1;
    if (::ioctl(fd_, I2C_RDWR, &xfer) < 0) return map_errno(errno);
    return BusStatus::Ok;
}

bool LinuxI2CBus::probe(uint8_t addr) {
    // Single-byte read, like `i2cdetect -r`. Safe for the sensors used here;
    // avoid probing write-sensitive devices (e.g. some EEPROM write-protect ranges).
    uint8_t byte = 0;
    i2c_msg msg = {};
    msg.addr = addr;
    msg.flags = I2C_M_RD;
    msg.len = 1;
    msg.buf = &byte;

    i2c_rdwr_ioctl_data xfer = {};
    xfer.msgs = &msg;
    xfer.nmsgs = 1;
    return ::ioctl(fd_, I2C_RDWR, &xfer) >= 0;
}

}  // namespace hwdiag

#endif  // __linux__
