#include "hwdiag/runner.hpp"

#include "hwdiag/mpu6050_regs.hpp"
#include "hwdiag/report.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <exception>
#include <map>
#include <sstream>

namespace hwdiag {

const char* to_string(Verdict v) {
    switch (v) {
        case Verdict::Pass: return "PASS";
        case Verdict::Fail: return "FAIL";
        case Verdict::Error: return "ERROR";
    }
    return "?";
}

namespace {

std::string fmt(double v, int precision = 3) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", precision, v);
    return buf;
}

std::string hex8(long v) {
    char buf[8];
    std::snprintf(buf, sizeof buf, "0x%02lX", v & 0xFF);
    return buf;
}

std::string utc_now() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

// Everything one unit's test sequence shares: the bus, lazily initialized drivers, results.
struct Context {
    Context(I2CBus& raw, int retries, BusStats& stats, const RunOptions& opts, std::string sn)
        : bus(raw, retries, stats), options(opts), serial(std::move(sn)), tmp(bus), imu(bus) {}

    RetryingBus bus;
    const RunOptions& options;
    std::string serial;
    Tmp102 tmp;
    Mpu6050 imu;
    bool imu_attempted = false;
    bool imu_ready = false;
    std::optional<ImuCalibration> calibration;

    // IMU tests share one init; if it fails every IMU test reports Error, not Fail.
    Mpu6050* ensure_imu(TestResult& r) {
        if (!imu_attempted) {
            imu_attempted = true;
            imu_ready = imu.init();
        }
        if (!imu_ready) {
            r.verdict = Verdict::Error;
            r.message = "IMU init failed (no ACK or did not leave sleep)";
            return nullptr;
        }
        return &imu;
    }
};

void set(TestResult& r, bool ok, std::string measured, std::string limit, std::string message = "") {
    r.verdict = ok ? Verdict::Pass : Verdict::Fail;
    r.measured = std::move(measured);
    r.limit = std::move(limit);
    r.message = std::move(message);
}

void bus_error(TestResult& r, const std::string& what) {
    r.verdict = Verdict::Error;
    r.message = what;
}

uint8_t to_addr(long v, const TestSpec& t, const char* key) {
    if (v < 0x03 || v > 0x77) throw SpecError("test '" + t.name + "' " + key + " out of 7-bit range: " + hex8(v));
    return static_cast<uint8_t>(v);
}

// ---------------------------------------------------------------- test types

// probe: every listed address must ACK.
void t_probe(Context& c, const TestSpec& t, TestResult& r) {
    std::string missing;
    std::string present;
    for (long a : t.int_list("addrs")) {
        const uint8_t addr = to_addr(a, t, "addrs");
        std::string& dst = c.bus.probe(addr) ? present : missing;
        dst += (dst.empty() ? "" : ",") + hex8(addr);
    }
    if (missing.empty()) {
        set(r, true, present + " present", "all present");
    } else {
        set(r, false, "missing " + missing, "all present", "device(s) did not ACK");
    }
}

// reg_equals: a register (usually an ID register) must read an exact value.
void t_reg_equals(Context& c, const TestSpec& t, TestResult& r) {
    const uint8_t addr = to_addr(t.integer("addr"), t, "addr");
    const auto reg = static_cast<uint8_t>(t.integer("reg"));
    const long expect = t.integer("expect");
    uint8_t value = 0;
    const BusStatus s = c.bus.read_u8(addr, reg, value);
    if (s != BusStatus::Ok) return bus_error(r, std::string("read failed: ") + to_string(s));
    set(r, value == expect, hex8(value), "== " + hex8(expect),
        value == expect ? "" : "unexpected value (wrong part or bad assembly?)");
}

// temp_range: board temperature sensor must read within [min, max] C.
void t_temp_range(Context& c, const TestSpec& t, TestResult& r) {
    const double lo = t.num("min");
    const double hi = t.num("max");
    auto temp = c.tmp.read_celsius();
    if (!temp) return bus_error(r, "TMP102 read failed");
    set(r, *temp >= lo && *temp <= hi, fmt(*temp, 2) + " C", "[" + fmt(lo, 1) + ", " + fmt(hi, 1) + "] C");
}

// temp_agree: two independent sensors must agree - catches a sensor that is
// in range but wrong, which a range check alone would miss.
void t_temp_agree(Context& c, const TestSpec& t, TestResult& r) {
    const double max_delta = t.num("max_delta");
    auto board = c.tmp.read_celsius();
    if (!board) return bus_error(r, "TMP102 read failed");
    Mpu6050* imu = c.ensure_imu(r);
    if (!imu) return;
    auto s = imu->read_sample();
    if (!s) return bus_error(r, "IMU read failed");
    const double delta = std::fabs(*board - s->temp_c);
    set(r, delta <= max_delta, "delta " + fmt(delta, 2) + " C (TMP102 " + fmt(*board, 1) + ", IMU " + fmt(s->temp_c, 1) + ")",
        "<= " + fmt(max_delta, 1) + " C");
}

// accel_gravity: with the unit at rest, |mean accel| must equal 1 g.
void t_accel_gravity(Context& c, const TestSpec& t, TestResult& r) {
    const auto n = static_cast<std::size_t>(t.integer_or("samples", 50));
    const double tol = t.num("tolerance_g");
    Mpu6050* imu = c.ensure_imu(r);
    if (!imu) return;
    auto stats = collect_imu_stats(*imu, n);
    if (!stats) return bus_error(r, "IMU read failed during sampling");
    const double mag = stats->accel.mean.norm();
    set(r, std::fabs(mag - 1.0) <= tol, "|a| = " + fmt(mag, 4) + " g", "1.000 +/- " + fmt(tol, 3) + " g");
}

// imu_noise: worst-axis standard deviation must stay below limits.
void t_imu_noise(Context& c, const TestSpec& t, TestResult& r) {
    const auto n = static_cast<std::size_t>(t.integer_or("samples", 200));
    const double max_a = t.num("max_accel_std_g");
    const double max_g = t.num("max_gyro_std_dps");
    Mpu6050* imu = c.ensure_imu(r);
    if (!imu) return;
    auto stats = collect_imu_stats(*imu, n);
    if (!stats) return bus_error(r, "IMU read failed during sampling");
    const double a = stats->accel.stddev.max_abs();
    const double g = stats->gyro.stddev.max_abs();
    set(r, a <= max_a && g <= max_g, "accel " + fmt(a, 4) + " g, gyro " + fmt(g, 3) + " dps",
        "<= " + fmt(max_a, 3) + " g, <= " + fmt(max_g, 2) + " dps");
}

// imu_calibrate: estimate offsets, check they are within limits, then verify
// by re-measuring with the offsets applied.
void t_imu_calibrate(Context& c, const TestSpec& t, TestResult& r) {
    const auto n = static_cast<std::size_t>(t.integer_or("samples", 200));
    const double max_ab = t.num("max_accel_bias_g");
    const double max_gb = t.num("max_gyro_bias_dps");
    const double verify_tol = t.num("verify_tolerance_g");
    const Vec3 gravity{0.0, 0.0, 1.0};

    Mpu6050* imu = c.ensure_imu(r);
    if (!imu) return;
    auto stats = collect_imu_stats(*imu, n);
    if (!stats) return bus_error(r, "IMU read failed during sampling");
    const ImuCalibration cal = compute_calibration(*stats, gravity);
    const double ab = cal.accel_bias_g.max_abs();
    const double gb = cal.gyro_bias_dps.max_abs();
    const std::string limit = "bias <= " + fmt(max_ab, 3) + " g / " + fmt(max_gb, 1) + " dps; residual <= " + fmt(verify_tol, 3) + " g";

    if (ab > max_ab || gb > max_gb) {
        set(r, false, "bias " + fmt(ab, 4) + " g / " + fmt(gb, 2) + " dps", limit, "offset too large to calibrate out");
        return;
    }

    auto verify = collect_imu_stats(*imu, n, &cal);
    if (!verify) return bus_error(r, "IMU read failed during verification");
    const double residual = (verify->accel.mean - gravity).max_abs();
    const bool ok = residual <= verify_tol;
    set(r, ok, "bias " + fmt(ab, 4) + " g / " + fmt(gb, 2) + " dps; residual " + fmt(residual, 4) + " g", limit,
        ok ? "" : "calibration did not converge");
    if (!ok) return;

    c.calibration = cal;
    if (!c.options.cal_dir.empty()) {
        write_text_file(c.options.cal_dir + "/" + c.serial + ".cal.json", calibration_json(c.serial, cal));
    }
}

using TestFn = void (*)(Context&, const TestSpec&, TestResult&);

const std::map<std::string, TestFn>& registry() {
    static const std::map<std::string, TestFn> tests = {
        {"probe", t_probe},
        {"reg_equals", t_reg_equals},
        {"temp_range", t_temp_range},
        {"temp_agree", t_temp_agree},
        {"accel_gravity", t_accel_gravity},
        {"imu_noise", t_imu_noise},
        {"imu_calibrate", t_imu_calibrate},
    };
    return tests;
}

double ms_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

std::vector<std::string> supported_test_types() {
    std::vector<std::string> out;
    for (const auto& kv : registry()) out.push_back(kv.first);
    return out;
}

UnitReport run_unit(I2CBus& bus, const StationSpec& spec, const std::string& serial, const RunOptions& options) {
    UnitReport report;
    report.serial = serial;
    report.station = spec.station_name();
    report.bus = bus.name();
    report.started_utc = utc_now();
    const auto unit_start = std::chrono::steady_clock::now();

    int retries = options.retries;
    if (retries < 0) {
        try {
            retries = spec.retries();
        } catch (const std::exception&) {
            retries = 3;
        }
    }
    Context ctx(bus, retries, report.bus_stats, options, serial);

    for (const auto& test : spec.tests) {
        TestResult r;
        r.name = test.name;
        r.type = test.type;
        const auto start = std::chrono::steady_clock::now();
        try {
            auto it = registry().find(test.type);
            if (it == registry().end()) {
                r.verdict = Verdict::Error;
                r.message = "unknown test type '" + test.type + "'";
            } else {
                it->second(ctx, test, r);
            }
        } catch (const std::exception& e) {
            r.verdict = Verdict::Error;
            r.message = e.what();
        }
        r.duration_ms = ms_since(start);

        if (r.verdict != Verdict::Pass && report.first_failure.empty()) report.first_failure = r.name;
        report.results.push_back(std::move(r));
        if (options.stop_on_fail && !report.first_failure.empty()) break;
    }

    report.passed = report.first_failure.empty();
    report.calibration = ctx.calibration;
    report.duration_ms = ms_since(unit_start);
    return report;
}

}  // namespace hwdiag
