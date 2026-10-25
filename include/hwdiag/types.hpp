#pragma once
// Small value types shared by drivers, simulation and calibration code.

#include <cmath>

namespace hwdiag {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    double max_abs() const {
        return std::fmax(std::fabs(x), std::fmax(std::fabs(y), std::fabs(z)));
    }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

}  // namespace hwdiag
