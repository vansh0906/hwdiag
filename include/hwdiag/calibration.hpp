#pragma once
// IMU characterization and calibration: collect N samples with the unit at a
// known orientation, estimate per-axis bias and noise (Welford's algorithm),
// and produce offsets that are verified by re-measuring with them applied.

#include "hwdiag/drivers.hpp"
#include "hwdiag/types.hpp"

#include <cstddef>
#include <optional>

namespace hwdiag {

struct AxisStats {
    Vec3 mean;
    Vec3 stddev;
};

struct ImuStats {
    AxisStats accel;
    AxisStats gyro;
    std::size_t samples = 0;
};

struct ImuCalibration {
    Vec3 accel_bias_g;
    Vec3 gyro_bias_dps;
};

// Streaming mean / sample standard deviation.
class Welford {
public:
    void add(double x);
    double mean() const { return mean_; }
    double stddev() const;
    std::size_t count() const { return n_; }

private:
    std::size_t n_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;
};

// Returns nullopt if any sample read fails. If `cal` is given, it is applied to each sample.
std::optional<ImuStats> collect_imu_stats(Mpu6050& imu, std::size_t samples,
                                          const ImuCalibration* cal = nullptr);

ImuCalibration compute_calibration(const ImuStats& stats, const Vec3& expected_gravity_g);

ImuSample apply_calibration(const ImuCalibration& cal, const ImuSample& s);

}  // namespace hwdiag
