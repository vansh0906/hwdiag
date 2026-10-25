#include "hwdiag/calibration.hpp"

#include <array>
#include <cmath>

namespace hwdiag {

void Welford::add(double x) {
    ++n_;
    const double delta = x - mean_;
    mean_ += delta / static_cast<double>(n_);
    m2_ += delta * (x - mean_);
}

double Welford::stddev() const {
    return n_ > 1 ? std::sqrt(m2_ / static_cast<double>(n_ - 1)) : 0.0;
}

std::optional<ImuStats> collect_imu_stats(Mpu6050& imu, std::size_t samples, const ImuCalibration* cal) {
    std::array<Welford, 3> accel{};
    std::array<Welford, 3> gyro{};
    for (std::size_t i = 0; i < samples; ++i) {
        auto s = imu.read_sample();
        if (!s) return std::nullopt;
        const ImuSample v = cal ? apply_calibration(*cal, *s) : *s;
        for (int axis = 0; axis < 3; ++axis) {
            accel[axis].add(v.accel_g[axis]);
            gyro[axis].add(v.gyro_dps[axis]);
        }
    }
    ImuStats out;
    out.samples = samples;
    for (int axis = 0; axis < 3; ++axis) {
        out.accel.mean[axis] = accel[axis].mean();
        out.accel.stddev[axis] = accel[axis].stddev();
        out.gyro.mean[axis] = gyro[axis].mean();
        out.gyro.stddev[axis] = gyro[axis].stddev();
    }
    return out;
}

ImuCalibration compute_calibration(const ImuStats& stats, const Vec3& expected_gravity_g) {
    ImuCalibration cal;
    cal.accel_bias_g = stats.accel.mean - expected_gravity_g;  // static: accel should read gravity only
    cal.gyro_bias_dps = stats.gyro.mean;                       // static: gyro should read zero
    return cal;
}

ImuSample apply_calibration(const ImuCalibration& cal, const ImuSample& s) {
    ImuSample out = s;
    out.accel_g = s.accel_g - cal.accel_bias_g;
    out.gyro_dps = s.gyro_dps - cal.gyro_bias_dps;
    return out;
}

}  // namespace hwdiag
