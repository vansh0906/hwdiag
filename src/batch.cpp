#include "hwdiag/batch.hpp"

#include "hwdiag/faults.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <map>
#include <random>
#include <thread>
#include <vector>

namespace hwdiag {

namespace {

std::string make_serial(const std::string& prefix, std::size_t n) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "%04zu", n);
    return prefix + buf;
}

}  // namespace

BatchResult run_batch(const StationSpec& spec, const BatchOptions& o) {
    BatchResult result;
    result.units.resize(o.units);

    // Assign faults and seeds up front from one RNG so the outcome is identical
    // no matter how many threads run or how they are scheduled.
    std::mt19937 rng(o.seed);
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    const auto& faults = injectable_faults();
    std::uniform_int_distribution<std::size_t> pick(0, faults.size() - 1);
    std::vector<Fault> plan(o.units, Fault::None);
    std::vector<uint32_t> seeds(o.units);
    for (std::size_t i = 0; i < o.units; ++i) {
        if (coin(rng) < o.fault_rate) plan[i] = faults[pick(rng)];
        seeds[i] = static_cast<uint32_t>(rng());
    }

    std::atomic<std::size_t> next{0};
    auto worker = [&] {
        for (std::size_t i = next.fetch_add(1); i < o.units; i = next.fetch_add(1)) {
            const std::string serial = make_serial(o.serial_prefix, i + 1);
            UnitReport report;
            try {
                auto bus = build_sim_board(plan[i], seeds[i], o.latency);
                report = run_unit(*bus, spec, serial, o.run);
            } catch (const std::exception& e) {  // keep the pool alive; record the unit as failed
                report.serial = serial;
                report.station = spec.station_name();
                report.first_failure = std::string("station error: ") + e.what();
                report.passed = false;
            }
            report.fault_injected = fault_name(plan[i]);
            result.units[i] = std::move(report);  // each index written by exactly one thread
        }
    };

    const unsigned threads = std::max(1u, std::min<unsigned>(o.threads, static_cast<unsigned>(std::max<std::size_t>(o.units, 1))));
    result.threads_used = threads;
    const auto start = std::chrono::steady_clock::now();
    std::vector<std::thread> pool;
    pool.reserve(threads);
    for (unsigned t = 0; t < threads; ++t) pool.emplace_back(worker);
    for (auto& th : pool) th.join();
    result.wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return result;
}

BatchSummary summarize(const BatchResult& result) {
    BatchSummary s;
    s.units = result.units.size();
    for (const auto& u : result.units) {
        const bool defective = u.fault_injected != "none" && u.fault_injected != "bus_intermittent";
        if (u.passed) {
            ++s.passed;
            if (u.bus_stats.retries > 0) ++s.passed_with_retries;
            if (defective) ++s.escapes;
        } else if (u.fault_injected == "none") {
            ++s.false_failures;
        }
    }
    return s;
}

void print_batch_summary(std::ostream& os, const StationSpec& spec, const BatchResult& result) {
    const BatchSummary s = summarize(result);
    const double secs = result.wall_ms / 1000.0;

    os << "Station:     " << spec.station_name() << "\n";
    os << "Units:       " << s.units << "   threads: " << result.threads_used << "   wall: " << std::fixed
       << std::setprecision(1) << result.wall_ms << " ms   throughput: " << std::setprecision(1)
       << (secs > 0 ? static_cast<double>(s.units) / secs : 0.0) << " units/s\n";
    os << "Yield:       " << s.passed << "/" << s.units << " = " << std::setprecision(1) << s.yield_pct() << "%"
       << "   (passed but needed bus retries: " << s.passed_with_retries << ")\n\n";

    // Pareto of first failing test.
    std::map<std::string, std::size_t> pareto;
    for (const auto& u : result.units) {
        if (!u.passed) ++pareto[u.first_failure];
    }
    std::vector<std::pair<std::string, std::size_t>> ranked(pareto.begin(), pareto.end());
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
    os << "Failure Pareto (first failing test):\n";
    if (ranked.empty()) os << "  (no failures)\n";
    for (const auto& [name, count] : ranked) {
        os << "  " << std::left << std::setw(18) << name << std::right << std::setw(4) << count << "  "
           << std::string(count, '#') << "\n";
    }

    // Fault-injection coverage: did each injected defect get caught, and by which test?
    struct Row {
        std::size_t units = 0, failed = 0;
        std::map<std::string, std::size_t> caught_by;
    };
    std::map<std::string, Row> coverage;
    for (const auto& u : result.units) {
        Row& row = coverage[u.fault_injected];
        ++row.units;
        if (!u.passed) {
            ++row.failed;
            ++row.caught_by[u.first_failure];
        }
    }
    os << "\nFault-injection coverage:\n";
    os << "  " << std::left << std::setw(18) << "injected" << std::right << std::setw(6) << "units" << std::setw(8)
       << "failed" << "   first failing test\n";
    for (const auto& [fault, row] : coverage) {
        std::string by;
        for (const auto& [test, n] : row.caught_by) by += (by.empty() ? "" : ", ") + test + " x" + std::to_string(n);
        os << "  " << std::left << std::setw(18) << fault << std::right << std::setw(6) << row.units << std::setw(8)
           << row.failed << "   " << (by.empty() ? "-" : by) << "\n";
    }
    os << "\nTest escapes (defective unit passed): " << s.escapes
       << "   False failures (good unit failed): " << s.false_failures << "\n";
    os.unsetf(std::ios::fixed);
    os << std::setprecision(6);
}

}  // namespace hwdiag
