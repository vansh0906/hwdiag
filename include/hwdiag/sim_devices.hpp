#pragma once
// Register-accurate models of two real sensors, each with tunable error
// sources so the test station can be validated against known defects.
//   TMP102  - TI digital temperature sensor (0x48)
//   MPU-6050 - InvenSense 6-axis IMU (0x68)

#include "hwdiag/sim_bus.hpp"
#include "hwdiag/types.hpp"

#include <array>
#include <cstdint>
#include <random>

namespace hwdiag {

struct Tmp102Params {
    double ambient_c = 25.0;
    double offset_c = 0.0;   // sensor error (e.g. bad thermal path, wrong part grade)
    double noise_c = 0.02;
    uint32_t seed = 7;
};

class SimTmp102 : public SimDevice {
public:
    static constexpr uint8_t kAddr = 0x48;
    explicit SimTmp102(Tmp102Params p);

    bool read(uint8_t reg, uint8_t* out, std::size_t len) override;
    bool write(uint8_t reg, const uint8_t* data, std::size_t len) override;

private:
    uint16_t temperature_raw();

    Tmp102Params p_;
    uint16_t config_ = 0x60A0;  // power-on default
    uint16_t tlow_ = 0x4B00;    // 75 C
    uint16_t thigh_ = 0x5000;   // 80 C
    std::mt19937 rng_;
};

struct Mpu6050Params {
    uint8_t who_am_i = 0x68;
    Vec3 gravity_g{0.0, 0.0, 1.0};  // unit lying flat in the fixture
    Vec3 accel_bias_g;
    Vec3 gyro_bias_dps;
    double accel_noise_g = 0.004;
    double gyro_noise_dps = 0.05;
    double die_temp_c = 28.0;
    int stuck_axis = -1;  // 0/1/2 = accel X/Y/Z output stuck at zero
    uint32_t seed = 11;
};

class SimMpu6050 : public SimDevice {
public:
    static constexpr uint8_t kAddr = 0x68;
    explicit SimMpu6050(Mpu6050Params p);

    bool read(uint8_t reg, uint8_t* out, std::size_t len) override;
    bool write(uint8_t reg, const uint8_t* data, std::size_t len) override;

private:
    void reset_registers();
    void latch_sample();  // models the sensor-register snapshot taken at the start of a burst read
    void put_i16(uint8_t reg, double value);

    Mpu6050Params p_;
    std::array<uint8_t, 128> regs_{};
    std::mt19937 rng_;
};

}  // namespace hwdiag
