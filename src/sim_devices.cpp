#include "hwdiag/sim_devices.hpp"

#include "hwdiag/mpu6050_regs.hpp"

#include <algorithm>
#include <cmath>

namespace hwdiag {

namespace {

int16_t to_i16(double value) {
    const double clamped = std::clamp(std::round(value), -32768.0, 32767.0);
    return static_cast<int16_t>(clamped);
}

double gaussian(std::mt19937& rng, double sigma) {
    if (sigma <= 0.0) return 0.0;
    std::normal_distribution<double> dist(0.0, sigma);
    return dist(rng);
}

}  // namespace

// ---------------------------------------------------------------- TMP102

SimTmp102::SimTmp102(Tmp102Params p) : p_(p), rng_(p.seed) {}

uint16_t SimTmp102::temperature_raw() {
    const double t = p_.ambient_c + p_.offset_c + gaussian(rng_, p_.noise_c);
    long counts = std::lround(t / 0.0625);
    counts = std::clamp(counts, -2048L, 2047L);  // 12-bit signed range
    const auto left_justified = static_cast<int16_t>(counts * 16);
    return static_cast<uint16_t>(left_justified);
}

bool SimTmp102::read(uint8_t reg, uint8_t* out, std::size_t len) {
    uint16_t value = 0;
    switch (reg & 0x03) {  // pointer register: only the low 2 bits are decoded
        case 0x00: value = temperature_raw(); break;
        case 0x01: value = config_; break;
        case 0x02: value = tlow_; break;
        default: value = thigh_; break;
    }
    for (std::size_t i = 0; i < len; ++i) {
        out[i] = (i % 2 == 0) ? static_cast<uint8_t>(value >> 8) : static_cast<uint8_t>(value & 0xFF);
    }
    return true;
}

bool SimTmp102::write(uint8_t reg, const uint8_t* data, std::size_t len) {
    if (len < 2) return true;  // pointer-only write: ACK, nothing to store
    const auto value = static_cast<uint16_t>((data[0] << 8) | data[1]);
    switch (reg & 0x03) {
        case 0x01: config_ = value; break;
        case 0x02: tlow_ = value; break;
        case 0x03: thigh_ = value; break;
        default: break;  // temperature register is read-only
    }
    return true;
}

// ---------------------------------------------------------------- MPU-6050

SimMpu6050::SimMpu6050(Mpu6050Params p) : p_(p), rng_(p.seed) { reset_registers(); }

void SimMpu6050::reset_registers() {
    regs_.fill(0);
    regs_[mpu6050::PWR_MGMT_1] = mpu6050::PWR_SLEEP;  // powers up asleep
    regs_[mpu6050::WHO_AM_I] = p_.who_am_i;
}

void SimMpu6050::put_i16(uint8_t reg, double value) {
    const auto v = static_cast<uint16_t>(to_i16(value));
    regs_[reg] = static_cast<uint8_t>(v >> 8);
    regs_[reg + 1] = static_cast<uint8_t>(v & 0xFF);
}

void SimMpu6050::latch_sample() {
    using namespace mpu6050;
    if (regs_[PWR_MGMT_1] & PWR_SLEEP) {
        std::fill(regs_.begin() + ACCEL_XOUT_H, regs_.begin() + GYRO_ZOUT_L + 1, uint8_t{0});
        return;
    }
    const int afs = (regs_[ACCEL_CONFIG] >> 3) & 0x03;
    const int gfs = (regs_[GYRO_CONFIG] >> 3) & 0x03;
    const double accel_lsb = kAccelLsbPerG_2g / static_cast<double>(1 << afs);
    const double gyro_lsb = kGyroLsbPerDps_250 / static_cast<double>(1 << gfs);

    for (int axis = 0; axis < 3; ++axis) {
        double a = p_.gravity_g[axis] + p_.accel_bias_g[axis] + gaussian(rng_, p_.accel_noise_g);
        if (axis == p_.stuck_axis) a = 0.0;
        put_i16(static_cast<uint8_t>(ACCEL_XOUT_H + 2 * axis), a * accel_lsb);

        const double g = p_.gyro_bias_dps[axis] + gaussian(rng_, p_.gyro_noise_dps);
        put_i16(static_cast<uint8_t>(GYRO_XOUT_H + 2 * axis), g * gyro_lsb);
    }
    const double t = p_.die_temp_c + gaussian(rng_, 0.05);
    put_i16(TEMP_OUT_H, (t - kTempOffsetC) * kTempLsbPerC);
}

bool SimMpu6050::read(uint8_t reg, uint8_t* out, std::size_t len) {
    if (static_cast<std::size_t>(reg) + len > regs_.size()) return false;
    const std::size_t last = reg + len - 1;
    if (len > 0 && last >= mpu6050::ACCEL_XOUT_H && reg <= mpu6050::GYRO_ZOUT_L) latch_sample();
    std::copy_n(regs_.begin() + reg, len, out);
    return true;
}

bool SimMpu6050::write(uint8_t reg, const uint8_t* data, std::size_t len) {
    if (static_cast<std::size_t>(reg) + len > regs_.size()) return false;
    for (std::size_t i = 0; i < len; ++i) {
        const auto r = static_cast<uint8_t>(reg + i);
        if (r == mpu6050::WHO_AM_I) continue;                                       // read-only
        if (r >= mpu6050::ACCEL_XOUT_H && r <= mpu6050::GYRO_ZOUT_L) continue;      // read-only
        if (r == mpu6050::PWR_MGMT_1 && (data[i] & mpu6050::PWR_RESET)) {
            reset_registers();  // DEVICE_RESET restores power-on defaults
            continue;
        }
        regs_[r] = data[i];
    }
    return true;
}

}  // namespace hwdiag
