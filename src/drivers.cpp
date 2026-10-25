#include "hwdiag/drivers.hpp"

#include "hwdiag/mpu6050_regs.hpp"

#include <array>

namespace hwdiag {

// ---------------------------------------------------------------- RetryingBus

RetryingBus::RetryingBus(I2CBus& bus, int max_retries, BusStats& stats)
    : bus_(bus), max_retries_(max_retries < 0 ? 0 : max_retries), stats_(stats) {}

template <class Fn>
BusStatus RetryingBus::with_retry(Fn&& fn) {
    BusStatus status = BusStatus::IoError;
    for (int attempt = 0; attempt <= max_retries_; ++attempt) {
        ++stats_.transfers;
        status = fn();
        if (status == BusStatus::Ok) return status;
        if (attempt < max_retries_) ++stats_.retries;
    }
    ++stats_.failures;
    return status;
}

BusStatus RetryingBus::read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) {
    return with_retry([&] { return bus_.read_regs(addr, reg, out, len); });
}

BusStatus RetryingBus::write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) {
    return with_retry([&] { return bus_.write_regs(addr, reg, data, len); });
}

bool RetryingBus::probe(uint8_t addr) {
    return with_retry([&] { return bus_.probe(addr) ? BusStatus::Ok : BusStatus::Nack; }) == BusStatus::Ok;
}

// ---------------------------------------------------------------- TMP102

double Tmp102::raw_to_celsius(uint8_t msb, uint8_t lsb) {
    // The 12-bit result is left-justified, so the int16 value is counts * 16.
    // counts * 0.0625 C == int16 / 256.
    const auto raw = static_cast<int16_t>(static_cast<uint16_t>((msb << 8) | lsb));
    return static_cast<double>(raw) / 256.0;
}

std::optional<double> Tmp102::read_celsius() {
    std::array<uint8_t, 2> buf{};
    if (bus_.read_regs(addr_, 0x00, buf.data(), buf.size()) != BusStatus::Ok) return std::nullopt;
    return raw_to_celsius(buf[0], buf[1]);
}

// ---------------------------------------------------------------- MPU-6050

std::optional<uint8_t> Mpu6050::who_am_i() {
    uint8_t id = 0;
    if (bus_.read_u8(addr_, mpu6050::WHO_AM_I, id) != BusStatus::Ok) return std::nullopt;
    return id;
}

bool Mpu6050::init() {
    using namespace mpu6050;
    if (bus_.write_u8(addr_, PWR_MGMT_1, PWR_CLK_PLL_XGYRO) != BusStatus::Ok) return false;
    if (bus_.write_u8(addr_, ACCEL_CONFIG, 0x00) != BusStatus::Ok) return false;  // +-2 g
    if (bus_.write_u8(addr_, GYRO_CONFIG, 0x00) != BusStatus::Ok) return false;   // +-250 dps

    uint8_t pwr = 0;
    if (bus_.read_u8(addr_, PWR_MGMT_1, pwr) != BusStatus::Ok) return false;
    return (pwr & PWR_SLEEP) == 0;  // confirm the device actually woke up
}

std::optional<ImuSample> Mpu6050::read_sample() {
    using namespace mpu6050;
    std::array<uint8_t, 14> b{};
    if (bus_.read_regs(addr_, ACCEL_XOUT_H, b.data(), b.size()) != BusStatus::Ok) return std::nullopt;

    auto be16 = [&](std::size_t i) {
        return static_cast<double>(static_cast<int16_t>(static_cast<uint16_t>((b[i] << 8) | b[i + 1])));
    };
    ImuSample s;
    s.accel_g = {be16(0) / kAccelLsbPerG_2g, be16(2) / kAccelLsbPerG_2g, be16(4) / kAccelLsbPerG_2g};
    s.temp_c = be16(6) / kTempLsbPerC + kTempOffsetC;
    s.gyro_dps = {be16(8) / kGyroLsbPerDps_250, be16(10) / kGyroLsbPerDps_250, be16(12) / kGyroLsbPerDps_250};
    return s;
}

}  // namespace hwdiag
