#pragma once
// Test specification files: an INI-style text format that lets test engineers
// change limits and sequences without recompiling the station software.
//
//   [station]
//   name = HWD-1 Final Functional Test
//   retries = 3
//
//   [test imu_id]
//   type = reg_equals
//   addr = 0x68
//   reg = 0x75
//   expect = 0x68

#include <istream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace hwdiag {

class SpecError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct TestSpec {
    std::string name;
    std::string type;
    std::map<std::string, std::string> params;
    int line = 0;

    bool has(const std::string& key) const { return params.count(key) != 0; }
    const std::string& str(const std::string& key) const;
    double num(const std::string& key) const;
    double num_or(const std::string& key, double fallback) const;
    long integer(const std::string& key) const;  // accepts decimal or 0x hex
    long integer_or(const std::string& key, long fallback) const;
    std::vector<long> int_list(const std::string& key) const;  // "0x48, 0x68"
};

struct StationSpec {
    std::string source;
    std::map<std::string, std::string> station;
    std::vector<TestSpec> tests;

    std::string station_name() const;
    int retries() const;
};

StationSpec parse_spec(std::istream& in, const std::string& source_name);
StationSpec load_spec(const std::string& path);

double parse_double(const std::string& text);  // throws SpecError
long parse_long(const std::string& text);      // throws SpecError; base auto-detected

}  // namespace hwdiag
