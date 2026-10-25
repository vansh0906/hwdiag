#pragma once
// Executes a StationSpec against one unit under test and produces a report.

#include "hwdiag/calibration.hpp"
#include "hwdiag/drivers.hpp"
#include "hwdiag/i2c_bus.hpp"
#include "hwdiag/spec.hpp"

#include <optional>
#include <string>
#include <vector>

namespace hwdiag {

enum class Verdict { Pass, Fail, Error };
const char* to_string(Verdict v);

struct TestResult {
    std::string name;
    std::string type;
    Verdict verdict = Verdict::Error;
    std::string measured;
    std::string limit;
    std::string message;
    double duration_ms = 0.0;
};

struct UnitReport {
    std::string serial;
    std::string station;
    std::string bus;
    std::string started_utc;
    std::string fault_injected = "none";  // set by simulation runs only
    std::vector<TestResult> results;
    bool passed = false;
    std::string first_failure;  // name of first non-passing test
    BusStats bus_stats;
    std::optional<ImuCalibration> calibration;
    double duration_ms = 0.0;
};

struct RunOptions {
    bool stop_on_fail = false;
    int retries = -1;        // -1 = use the spec's [station] retries
    std::string cal_dir;     // if set, write <serial>.cal.json here after calibration
};

// Never throws for test failures; spec or bus problems become Error results.
UnitReport run_unit(I2CBus& bus, const StationSpec& spec, const std::string& serial,
                    const RunOptions& options);

// Test types this runner understands (for --help and spec validation).
std::vector<std::string> supported_test_types();

}  // namespace hwdiag
