// Unit tests for hwdiag. Self-contained (no external test framework) so the
// project builds anywhere with just CMake and a C++17 compiler.

#include "hwdiag/batch.hpp"
#include "hwdiag/calibration.hpp"
#include "hwdiag/drivers.hpp"
#include "hwdiag/faults.hpp"
#include "hwdiag/mpu6050_regs.hpp"
#include "hwdiag/report.hpp"
#include "hwdiag/runner.hpp"
#include "hwdiag/sim_devices.hpp"
#include "hwdiag/spec.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace hwdiag;

namespace {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

int g_failures = 0;

struct Register {
    Register(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

#define TEST(name)                                     \
    void name();                                       \
    const Register reg_##name(#name, name);            \
    void name()

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            ++g_failures;                                                                  \
            std::cerr << "    CHECK failed: " #cond " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        }                                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

template <class Fn>
bool throws_spec_error(Fn fn) {
    try {
        fn();
    } catch (const SpecError&) {
        return true;
    }
    return false;
}

StationSpec bundled_spec() { return load_spec(HWDIAG_SPEC); }

const TestResult* find_result(const UnitReport& r, const std::string& name) {
    for (const auto& t : r.results) {
        if (t.name == name) return &t;
    }
    return nullptr;
}

// ---------------------------------------------------------------- spec parsing

TEST(spec_parses_sections_params_and_hex) {
    std::istringstream in(
        "# comment\n"
        "[station]\nname = Bench\nretries = 2\n"
        "[test scan]\ntype = probe\naddrs = 0x48, 0x68  ; trailing comment\n"
        "[test id]\ntype = reg_equals\naddr = 0x68\nreg = 117\nexpect = 0x68\n");
    const StationSpec s = parse_spec(in, "inline");
    CHECK(s.station_name() == "Bench");
    CHECK(s.retries() == 2);
    CHECK(s.tests.size() == 2);
    CHECK(s.tests[0].type == "probe");
    const auto addrs = s.tests[0].int_list("addrs");
    CHECK(addrs.size() == 2 && addrs[0] == 0x48 && addrs[1] == 0x68);
    CHECK(s.tests[1].integer("reg") == 0x75);
}

TEST(spec_rejects_malformed_input_with_line_numbers) {
    auto parse = [](const std::string& text) {
        std::istringstream in(text);
        return parse_spec(in, "bad");
    };
    CHECK(throws_spec_error([&] { parse("key = value\n"); }));                          // before any section
    CHECK(throws_spec_error([&] { parse("[test a]\naddr = 1\n"); }));                   // missing type
    CHECK(throws_spec_error([&] { parse("[test a]\ntype=probe\n[test a]\ntype=probe\n"); }));  // duplicate
    CHECK(throws_spec_error([&] { parse("[bogus]\n"); }));
    CHECK(throws_spec_error([&] { parse("[test a]\ntype = probe\nno equals sign\n"); }));
    try {
        parse("[station]\nname = x\n[test t]\ntype = probe\ngarbage\n");
        CHECK(false);
    } catch (const SpecError& e) {
        CHECK(std::string(e.what()).find("bad:5:") != std::string::npos);
    }
}

TEST(spec_number_parsing_is_strict) {
    CHECK(parse_long("0x1C") == 28);
    CHECK(parse_long(" 42 ") == 42);
    CHECK_NEAR(parse_double("-1.5e-2"), -0.015, 1e-12);
    CHECK(throws_spec_error([] { parse_long("12abc"); }));
    CHECK(throws_spec_error([] { parse_double(""); }));
}

TEST(bundled_spec_loads_and_uses_known_test_types) {
    const StationSpec s = bundled_spec();
    const auto types = supported_test_types();
    for (const auto& t : s.tests) {
        bool known = false;
        for (const auto& k : types) known = known || (k == t.type);
        CHECK(known);
    }
}

// ---------------------------------------------------------------- bus + devices

TEST(sim_bus_nacks_missing_devices) {
    SimBus bus;
    uint8_t v = 0;
    CHECK(!bus.probe(0x50));
    CHECK(bus.read_u8(0x50, 0x00, v) == BusStatus::Nack);
    bus.attach(0x48, std::make_unique<SimTmp102>(Tmp102Params{}));
    CHECK(bus.probe(0x48));
}

TEST(tmp102_conversion_matches_datasheet_examples) {
    // TMP102 datasheet table: 0x7FF0 = 127.9375 C, 0x1900 = 25 C, 0xFFC0 = -0.25 C, 0xE700 = -25 C.
    CHECK_NEAR(Tmp102::raw_to_celsius(0x7F, 0xF0), 127.9375, 1e-9);
    CHECK_NEAR(Tmp102::raw_to_celsius(0x19, 0x00), 25.0, 1e-9);
    CHECK_NEAR(Tmp102::raw_to_celsius(0xFF, 0xC0), -0.25, 1e-9);
    CHECK_NEAR(Tmp102::raw_to_celsius(0xE7, 0x00), -25.0, 1e-9);
}

TEST(tmp102_driver_reads_simulated_temperature_including_negative) {
    for (double t : {25.0, -10.5, 60.25}) {
        SimBus bus;
        Tmp102Params p;
        p.ambient_c = t;
        p.noise_c = 0.0;
        bus.attach(0x48, std::make_unique<SimTmp102>(p));
        BusStats stats;
        RetryingBus rb(bus, 0, stats);
        Tmp102 tmp(rb);
        auto c = tmp.read_celsius();
        CHECK(c.has_value());
        CHECK_NEAR(*c, t, 0.0625);
    }
}

TEST(mpu6050_starts_asleep_and_reports_gravity_after_init) {
    SimBus bus;
    Mpu6050Params p;
    p.accel_noise_g = 0.0;
    p.gyro_noise_dps = 0.0;
    bus.attach(0x68, std::make_unique<SimMpu6050>(p));
    BusStats stats;
    RetryingBus rb(bus, 0, stats);
    Mpu6050 imu(rb);

    CHECK(imu.who_am_i() == std::optional<uint8_t>(mpu6050::kExpectedId));
    auto asleep = imu.read_sample();
    CHECK(asleep && asleep->accel_g.norm() == 0.0);  // sensor registers read zero while asleep

    CHECK(imu.init());
    auto s = imu.read_sample();
    CHECK(s.has_value());
    CHECK_NEAR(s->accel_g.z, 1.0, 1e-3);
    CHECK_NEAR(s->temp_c, p.die_temp_c, 0.2);
}

TEST(mpu6050_device_reset_restores_sleep) {
    SimBus bus;
    bus.attach(0x68, std::make_unique<SimMpu6050>(Mpu6050Params{}));
    BusStats stats;
    RetryingBus rb(bus, 0, stats);
    Mpu6050 imu(rb);
    CHECK(imu.init());
    CHECK(rb.write_u8(0x68, mpu6050::PWR_MGMT_1, mpu6050::PWR_RESET) == BusStatus::Ok);
    uint8_t pwr = 0;
    CHECK(rb.read_u8(0x68, mpu6050::PWR_MGMT_1, pwr) == BusStatus::Ok);
    CHECK(pwr == mpu6050::PWR_SLEEP);
}

TEST(retrying_bus_recovers_from_intermittent_nacks_and_counts_retries) {
    SimBusConfig cfg;
    cfg.nack_probability = 0.3;
    cfg.seed = 123;
    SimBus bus(cfg);
    bus.attach(0x48, std::make_unique<SimTmp102>(Tmp102Params{}));

    BusStats with_retries;
    RetryingBus rb(bus, 8, with_retries);
    Tmp102 tmp(rb);
    int ok = 0;
    for (int i = 0; i < 200; ++i) ok += tmp.read_celsius().has_value() ? 1 : 0;
    CHECK(ok == 200);  // 0.3^9 failure chance per read
    CHECK(with_retries.retries > 0);
    CHECK(with_retries.failures == 0);

    BusStats no_retries;
    RetryingBus rb0(bus, 0, no_retries);
    Tmp102 tmp0(rb0);
    int ok0 = 0;
    for (int i = 0; i < 200; ++i) ok0 += tmp0.read_celsius().has_value() ? 1 : 0;
    CHECK(ok0 < 200);
    CHECK(no_retries.failures == static_cast<uint64_t>(200 - ok0));
}

// ---------------------------------------------------------------- calibration

TEST(welford_matches_textbook_mean_and_stddev) {
    Welford w;
    for (double x : {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0}) w.add(x);
    CHECK_NEAR(w.mean(), 5.0, 1e-12);
    CHECK_NEAR(w.stddev(), std::sqrt(32.0 / 7.0), 1e-12);  // sample (n-1) standard deviation
}

TEST(calibration_recovers_injected_bias_and_verifies) {
    SimBus bus;
    Mpu6050Params p;
    p.accel_bias_g = {0.025, -0.018, 0.011};
    p.gyro_bias_dps = {1.5, -0.7, 0.3};
    bus.attach(0x68, std::make_unique<SimMpu6050>(p));
    BusStats stats;
    RetryingBus rb(bus, 0, stats);
    Mpu6050 imu(rb);
    CHECK(imu.init());

    auto st = collect_imu_stats(imu, 400);
    CHECK(st.has_value());
    const ImuCalibration cal = compute_calibration(*st, {0.0, 0.0, 1.0});
    for (int axis = 0; axis < 3; ++axis) {
        CHECK_NEAR(cal.accel_bias_g[axis], p.accel_bias_g[axis], 0.002);
        CHECK_NEAR(cal.gyro_bias_dps[axis], p.gyro_bias_dps[axis], 0.05);
    }
    auto corrected = collect_imu_stats(imu, 400, &cal);
    CHECK(corrected.has_value());
    CHECK((corrected->accel.mean - Vec3{0.0, 0.0, 1.0}).max_abs() < 0.002);
    CHECK(corrected->gyro.mean.max_abs() < 0.05);
}

// ---------------------------------------------------------------- runner + faults

TEST(good_board_passes_every_test) {
    const StationSpec spec = bundled_spec();
    for (uint32_t seed = 1; seed <= 25; ++seed) {
        auto bus = build_sim_board(Fault::None, seed);
        const UnitReport r = run_unit(*bus, spec, "GOOD", RunOptions{});
        CHECK(r.passed);
        CHECK(r.results.size() == spec.tests.size());
        CHECK(r.calibration.has_value());
        if (!r.passed) std::cerr << "    seed " << seed << " failed at " << r.first_failure << "\n";
    }
}

TEST(every_injected_fault_is_caught_by_the_expected_test) {
    const StationSpec spec = bundled_spec();
    struct Expect {
        Fault fault;
        const char* first_failure;
    };
    const Expect cases[] = {
        {Fault::ImuMissing, "bus_scan"},     {Fault::TempMissing, "bus_scan"},
        {Fault::ImuWrongPart, "imu_id"},     {Fault::TempOffset, "temp_agree"},
        {Fault::ImuStuckZ, "imu_gravity"},   {Fault::ImuNoisy, "imu_noise"},
        {Fault::ImuHighBias, "imu_calibrate"},
    };
    for (const auto& c : cases) {
        for (uint32_t seed = 1; seed <= 10; ++seed) {
            auto bus = build_sim_board(c.fault, seed);
            const UnitReport r = run_unit(*bus, spec, "BAD", RunOptions{});
            CHECK(!r.passed);
            CHECK(r.first_failure == c.first_failure);
            if (r.first_failure != c.first_failure) {
                std::cerr << "    " << fault_name(c.fault) << " seed " << seed << " first failure: " << r.first_failure << "\n";
            }
        }
    }
}

TEST(missing_imu_reports_errors_not_crashes_for_dependent_tests) {
    auto bus = build_sim_board(Fault::ImuMissing, 3);
    const UnitReport r = run_unit(*bus, bundled_spec(), "SN", RunOptions{});
    const TestResult* gravity = find_result(r, "imu_gravity");
    CHECK(gravity != nullptr && gravity->verdict == Verdict::Error);
    CHECK(r.bus_stats.failures > 0);
}

TEST(intermittent_bus_passes_but_is_flagged_by_retry_count) {
    const StationSpec spec = bundled_spec();
    int passed = 0;
    for (uint32_t seed = 1; seed <= 10; ++seed) {
        auto bus = build_sim_board(Fault::BusIntermittent, seed);
        const UnitReport r = run_unit(*bus, spec, "MARGINAL", RunOptions{});
        passed += r.passed ? 1 : 0;
        CHECK(r.bus_stats.retries > 0);
    }
    CHECK(passed >= 9);
}

TEST(stop_on_fail_halts_the_sequence) {
    auto bus = build_sim_board(Fault::ImuWrongPart, 1);
    RunOptions opt;
    opt.stop_on_fail = true;
    const UnitReport r = run_unit(*bus, bundled_spec(), "SN", opt);
    CHECK(!r.passed);
    CHECK(r.results.size() == 2);  // bus_scan passes, imu_id fails, then stop
}

TEST(unknown_test_type_and_bad_parameters_become_errors) {
    std::istringstream in("[test weird]\ntype = flux_capacitor\n[test badaddr]\ntype = reg_equals\naddr = 0x99\nreg = 0\nexpect = 0\n");
    const StationSpec spec = parse_spec(in, "inline");
    auto bus = build_sim_board(Fault::None, 1);
    const UnitReport r = run_unit(*bus, spec, "SN", RunOptions{});
    CHECK(r.results.size() == 2);
    CHECK(r.results[0].verdict == Verdict::Error);
    CHECK(r.results[1].verdict == Verdict::Error);
}

TEST(batch_is_deterministic_across_thread_counts_with_no_escapes) {
    const StationSpec spec = bundled_spec();
    BatchOptions o;
    o.units = 120;
    o.fault_rate = 0.3;
    o.threads = 1;
    const BatchResult one = run_batch(spec, o);
    o.threads = 8;
    const BatchResult many = run_batch(spec, o);

    CHECK(one.units.size() == many.units.size());
    for (std::size_t i = 0; i < one.units.size(); ++i) {
        CHECK(one.units[i].serial == many.units[i].serial);
        CHECK(one.units[i].passed == many.units[i].passed);
        CHECK(one.units[i].fault_injected == many.units[i].fault_injected);
        CHECK(one.units[i].first_failure == many.units[i].first_failure);
    }
    const BatchSummary s = summarize(many);
    CHECK(s.escapes == 0);
    CHECK(s.false_failures == 0);
    CHECK(s.passed < s.units);  // with a 30% fault rate some units must fail
}

// ---------------------------------------------------------------- reports

TEST(json_and_csv_escape_special_characters) {
    CHECK(json_escape("a\"b\\c\n\t") == "a\\\"b\\\\c\\n\\t");
    CHECK(json_escape(std::string(1, '\x01')) == "\\u0001");
    UnitReport r;
    r.serial = "SN,1";
    r.station = "Station \"A\"";
    const std::string row = to_csv_row(r);
    CHECK(row.find("\"SN,1\"") != std::string::npos);
    CHECK(row.find("\"Station \"\"A\"\"\"") != std::string::npos);
    const std::string json = to_json(r);
    CHECK(json.find("\"station\": \"Station \\\"A\\\"\"") != std::string::npos);
}

}  // namespace

int main() {
    int failed_tests = 0;
    for (const auto& t : registry()) {
        const int before = g_failures;
        try {
            t.fn();
        } catch (const std::exception& e) {
            ++g_failures;
            std::cerr << "    unexpected exception: " << e.what() << "\n";
        }
        const bool ok = g_failures == before;
        failed_tests += ok ? 0 : 1;
        std::cout << (ok ? "[ PASS ] " : "[ FAIL ] ") << t.name << "\n";
    }
    std::cout << "\n" << (registry().size() - static_cast<std::size_t>(failed_tests)) << "/" << registry().size()
              << " tests passed\n";
    return failed_tests == 0 ? 0 : 1;
}
