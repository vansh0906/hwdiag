#pragma once
// Multi-unit test run: tests many simulated units in parallel on a worker
// thread pool (like a multi-slot production fixture) and summarizes yield,
// failure Pareto and fault-injection coverage.

#include "hwdiag/runner.hpp"
#include "hwdiag/spec.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace hwdiag {

struct BatchOptions {
    std::size_t units = 50;
    unsigned threads = 4;
    double fault_rate = 0.15;
    uint32_t seed = 42;
    std::chrono::microseconds latency{0};
    std::string serial_prefix = "HWD1-";
    RunOptions run;
};

struct BatchResult {
    std::vector<UnitReport> units;
    unsigned threads_used = 0;
    double wall_ms = 0.0;
};

BatchResult run_batch(const StationSpec& spec, const BatchOptions& options);

struct BatchSummary {
    std::size_t units = 0;
    std::size_t passed = 0;
    std::size_t passed_with_retries = 0;
    std::size_t escapes = 0;         // defective unit that passed
    std::size_t false_failures = 0;  // good unit that failed
    double yield_pct() const { return units ? 100.0 * static_cast<double>(passed) / static_cast<double>(units) : 0.0; }
};

BatchSummary summarize(const BatchResult& result);
void print_batch_summary(std::ostream& os, const StationSpec& spec, const BatchResult& result);

}  // namespace hwdiag
