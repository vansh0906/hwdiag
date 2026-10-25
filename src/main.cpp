// hwdiag - hardware diagnostics, calibration and production test station.
//
//   hwdiag scan   --bus sim|/dev/i2c-1 [--fault NAME]
//   hwdiag dump   --bus BUS --addr 0x68 [--start 0x00] [--len 128]
//   hwdiag run    --spec FILE --bus BUS [--serial SN] [--fault NAME] [--out DIR] [--stop-on-fail]
//   hwdiag batch  --spec FILE [--units N] [--threads T] [--fault-rate P] [--seed S]
//                 [--latency-us US] [--out DIR] [--min-yield PCT]
//   hwdiag faults

#include "hwdiag/batch.hpp"
#include "hwdiag/faults.hpp"
#include "hwdiag/linux_i2c_bus.hpp"
#include "hwdiag/report.hpp"
#include "hwdiag/runner.hpp"
#include "hwdiag/sim_bus.hpp"
#include "hwdiag/spec.hpp"

#include <chrono>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace hwdiag;

namespace {

constexpr int kExitPass = 0;
constexpr int kExitFail = 1;
constexpr int kExitUsage = 2;

struct Args {
    std::string command;
    std::map<std::string, std::string> opts;
    std::set<std::string> flags;

    bool has(const std::string& k) const { return opts.count(k) != 0; }
    std::string get(const std::string& k, const std::string& fallback = "") const {
        auto it = opts.find(k);
        return it == opts.end() ? fallback : it->second;
    }
};

class UsageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

void print_usage(std::ostream& os) {
    os << "hwdiag - hardware diagnostics, calibration and production test\n\n"
          "Usage:\n"
          "  hwdiag scan   --bus sim|/dev/i2c-N [--fault NAME]      list responding I2C addresses\n"
          "  hwdiag dump   --bus BUS --addr ADDR [--start R] [--len N]  hex dump device registers\n"
          "  hwdiag run    --spec FILE --bus BUS [--serial SN] [--fault NAME]\n"
          "                [--out DIR] [--retries N] [--stop-on-fail]  test one unit (exit 0 pass, 1 fail)\n"
          "  hwdiag batch  --spec FILE [--units N] [--threads T] [--fault-rate P] [--seed S]\n"
          "                [--latency-us US] [--out DIR] [--min-yield PCT]  test many simulated units\n"
          "  hwdiag faults                                          list injectable faults\n\n"
          "BUS is 'sim' (simulated HWD-1 board) or a Linux adapter such as /dev/i2c-1.\n";
}

Args parse_args(int argc, char** argv) {
    static const std::set<std::string> kFlags = {"stop-on-fail", "help"};
    Args a;
    if (argc < 2) throw UsageError("missing command");
    a.command = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string tok = argv[i];
        if (tok.rfind("--", 0) != 0) throw UsageError("unexpected argument '" + tok + "'");
        tok = tok.substr(2);
        if (kFlags.count(tok)) {
            a.flags.insert(tok);
        } else {
            if (i + 1 >= argc) throw UsageError("--" + tok + " needs a value");
            a.opts[tok] = argv[++i];
        }
    }
    return a;
}

Fault fault_arg(const Args& a) {
    const std::string name = a.get("fault", "none");
    auto f = parse_fault(name);
    if (!f) throw UsageError("unknown fault '" + name + "' (see: hwdiag faults)");
    return *f;
}

long int_arg(const Args& a, const std::string& key, long fallback) {
    if (!a.has(key)) return fallback;
    try {
        return parse_long(a.get(key));
    } catch (const SpecError&) {
        throw UsageError("--" + key + " expects an integer");
    }
}

double double_arg(const Args& a, const std::string& key, double fallback) {
    if (!a.has(key)) return fallback;
    try {
        return parse_double(a.get(key));
    } catch (const SpecError&) {
        throw UsageError("--" + key + " expects a number");
    }
}

std::unique_ptr<I2CBus> open_bus(const Args& a) {
    const std::string bus = a.get("bus", "sim");
    if (bus == "sim") {
        const auto seed = static_cast<uint32_t>(int_arg(a, "seed", 1));
        return build_sim_board(fault_arg(a), seed);
    }
    if (a.has("fault")) throw UsageError("--fault only applies to --bus sim");
#ifdef __linux__
    return std::make_unique<LinuxI2CBus>(bus);
#else
    throw UsageError("real I2C adapters are supported on Linux only; use --bus sim on this platform");
#endif
}

StationSpec spec_arg(const Args& a) {
    if (!a.has("spec")) throw UsageError("--spec FILE is required");
    return load_spec(a.get("spec"));
}

// i2cdetect-style address grid.
int cmd_scan(const Args& a) {
    auto bus = open_bus(a);
    std::cout << "Scanning " << bus->name() << " (0x03-0x77)\n     ";
    for (int col = 0; col < 16; ++col) std::printf("%3x", col);
    std::cout << "\n";
    int found = 0;
    for (int row = 0; row < 0x80; row += 16) {
        std::printf("%02x: ", row);
        for (int col = 0; col < 16; ++col) {
            const int addr = row + col;
            if (addr < 0x03 || addr > 0x77) {
                std::printf("   ");
            } else if (bus->probe(static_cast<uint8_t>(addr))) {
                std::printf(" %02x", addr);
                ++found;
            } else {
                std::printf(" --");
            }
        }
        std::printf("\n");
    }
    std::cout << found << " device(s) found\n";
    return kExitPass;
}

// i2cdump-style register dump.
int cmd_dump(const Args& a) {
    if (!a.has("addr")) throw UsageError("--addr is required");
    const long addr = int_arg(a, "addr", 0);
    const long start = int_arg(a, "start", 0);
    const long len = int_arg(a, "len", 128);
    if (addr < 0x03 || addr > 0x77) throw UsageError("--addr must be a 7-bit address (0x03-0x77)");
    if (start < 0 || len <= 0 || start + len > 256) throw UsageError("--start/--len must stay within 0x00-0xFF");

    auto bus = open_bus(a);
    std::vector<uint8_t> buf(static_cast<std::size_t>(len));
    const BusStatus s = bus->read_regs(static_cast<uint8_t>(addr), static_cast<uint8_t>(start), buf.data(), buf.size());
    if (s != BusStatus::Ok) {
        std::cerr << "read from 0x" << std::hex << addr << std::dec << " failed: " << to_string(s) << "\n";
        return kExitFail;
    }
    std::printf("Device 0x%02lx on %s\n     ", addr, bus->name().c_str());
    for (int col = 0; col < 16; ++col) std::printf(" %2x", col);
    std::printf("\n");
    for (long i = 0; i < len; ++i) {
        const long reg = start + i;
        if (i == 0 || reg % 16 == 0) {
            if (i != 0) std::printf("\n");
            std::printf("%02lx: ", reg - reg % 16);
            for (long pad = 0; pad < reg % 16; ++pad) std::printf("   ");
        }
        std::printf(" %02x", buf[static_cast<std::size_t>(i)]);
    }
    std::printf("\n");
    return kExitPass;
}

int cmd_run(const Args& a) {
    const StationSpec spec = spec_arg(a);
    auto bus = open_bus(a);

    RunOptions opt;
    opt.stop_on_fail = a.flags.count("stop-on-fail") != 0;
    opt.retries = static_cast<int>(int_arg(a, "retries", -1));
    const std::string out = a.get("out", "reports");
    opt.cal_dir = out + "/cal";

    UnitReport report = run_unit(*bus, spec, a.get("serial", "HWD1-0001"), opt);
    if (a.get("bus", "sim") == "sim") report.fault_injected = fault_name(fault_arg(a));

    print_unit_table(std::cout, report);
    write_text_file(out + "/" + report.serial + ".json", to_json(report));
    append_csv(out + "/test_log.csv", report);
    std::cout << "Report: " << out << "/" << report.serial << ".json\n";
    return report.passed ? kExitPass : kExitFail;
}

int cmd_batch(const Args& a) {
    const StationSpec spec = spec_arg(a);
    BatchOptions o;
    o.units = static_cast<std::size_t>(int_arg(a, "units", 50));
    o.threads = static_cast<unsigned>(int_arg(a, "threads", 4));
    o.fault_rate = double_arg(a, "fault-rate", 0.15);
    o.seed = static_cast<uint32_t>(int_arg(a, "seed", 42));
    o.latency = std::chrono::microseconds(int_arg(a, "latency-us", 0));
    if (o.units == 0 || o.threads == 0) throw UsageError("--units and --threads must be > 0");
    if (o.fault_rate < 0.0 || o.fault_rate > 1.0) throw UsageError("--fault-rate must be between 0 and 1");

    const std::string out = a.get("out", "reports");
    o.run.cal_dir = out + "/cal";
    o.run.stop_on_fail = a.flags.count("stop-on-fail") != 0;
    std::filesystem::create_directories(out + "/units");
    std::filesystem::create_directories(o.run.cal_dir);

    const BatchResult result = run_batch(spec, o);

    // Write logs from the main thread after the pool finishes.
    for (const auto& u : result.units) {
        write_text_file(out + "/units/" + u.serial + ".json", to_json(u));
        append_csv(out + "/test_log.csv", u);
    }
    print_batch_summary(std::cout, spec, result);
    std::cout << "Logs: " << out << "/test_log.csv, " << out << "/units/, " << out << "/cal/\n";

    if (a.has("min-yield")) {
        const double min_yield = double_arg(a, "min-yield", 0.0);
        const double y = summarize(result).yield_pct();
        if (y < min_yield) {
            std::printf("YIELD GATE FAILED: %.1f%% < %.1f%%\n", y, min_yield);
            return kExitFail;
        }
        std::printf("Yield gate passed: %.1f%% >= %.1f%%\n", y, min_yield);
    }
    return kExitPass;
}

int cmd_faults() {
    for (Fault f : {Fault::None}) std::cout << std::left << std::setw(18) << fault_name(f) << fault_description(f) << "\n";
    for (Fault f : injectable_faults()) {
        std::cout << std::left << std::setw(18) << fault_name(f) << fault_description(f) << "\n";
    }
    return kExitPass;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        if (args.flags.count("help") || args.command == "help" || args.command == "-h") {
            print_usage(std::cout);
            return kExitPass;
        }
        if (args.command == "scan") return cmd_scan(args);
        if (args.command == "dump") return cmd_dump(args);
        if (args.command == "run") return cmd_run(args);
        if (args.command == "batch") return cmd_batch(args);
        if (args.command == "faults") return cmd_faults();
        throw UsageError("unknown command '" + args.command + "'");
    } catch (const UsageError& e) {
        std::cerr << "error: " << e.what() << "\n\n";
        print_usage(std::cerr);
        return kExitUsage;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return kExitUsage;
    }
}
