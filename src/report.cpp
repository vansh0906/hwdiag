#include "hwdiag/report.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace hwdiag {

namespace {

std::string num(double v, int precision = 6) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", precision, v);
    return buf;
}

std::string quoted(const std::string& s) { return "\"" + json_escape(s) + "\""; }

std::string vec_json(const Vec3& v) {
    return "[" + num(v.x) + ", " + num(v.y) + ", " + num(v.z) + "]";
}

std::string csv_field(const std::string& s) {
    if (s.find_first_of(",\"\n") == std::string::npos) return s;
    std::string out = "\"";
    for (char ch : s) out += (ch == '"') ? std::string("\"\"") : std::string(1, ch);
    return out + "\"";
}

}  // namespace

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (unsigned char ch : s) {
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (ch < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", ch);
                    out += buf;
                } else {
                    out += static_cast<char>(ch);
                }
        }
    }
    return out;
}

std::string to_json(const UnitReport& r) {
    std::ostringstream o;
    o << "{\n";
    o << "  \"serial\": " << quoted(r.serial) << ",\n";
    o << "  \"station\": " << quoted(r.station) << ",\n";
    o << "  \"bus\": " << quoted(r.bus) << ",\n";
    o << "  \"started_utc\": " << quoted(r.started_utc) << ",\n";
    o << "  \"fault_injected\": " << quoted(r.fault_injected) << ",\n";
    o << "  \"result\": " << quoted(r.passed ? "PASS" : "FAIL") << ",\n";
    o << "  \"first_failure\": " << (r.first_failure.empty() ? "null" : quoted(r.first_failure)) << ",\n";
    o << "  \"duration_ms\": " << num(r.duration_ms, 3) << ",\n";
    o << "  \"bus_stats\": {\"transfers\": " << r.bus_stats.transfers << ", \"retries\": " << r.bus_stats.retries
      << ", \"failures\": " << r.bus_stats.failures << "},\n";
    if (r.calibration) {
        o << "  \"calibration\": {\"accel_bias_g\": " << vec_json(r.calibration->accel_bias_g)
          << ", \"gyro_bias_dps\": " << vec_json(r.calibration->gyro_bias_dps) << "},\n";
    } else {
        o << "  \"calibration\": null,\n";
    }
    o << "  \"tests\": [\n";
    for (std::size_t i = 0; i < r.results.size(); ++i) {
        const auto& t = r.results[i];
        o << "    {\"name\": " << quoted(t.name) << ", \"type\": " << quoted(t.type) << ", \"verdict\": "
          << quoted(to_string(t.verdict)) << ", \"measured\": " << quoted(t.measured) << ", \"limit\": "
          << quoted(t.limit) << ", \"message\": " << quoted(t.message) << ", \"duration_ms\": " << num(t.duration_ms, 3)
          << "}" << (i + 1 < r.results.size() ? "," : "") << "\n";
    }
    o << "  ]\n}\n";
    return o.str();
}

std::string calibration_json(const std::string& serial, const ImuCalibration& cal) {
    std::ostringstream o;
    o << "{\n  \"serial\": " << quoted(serial) << ",\n  \"format\": \"hwdiag-imu-cal-v1\",\n"
      << "  \"accel_bias_g\": " << vec_json(cal.accel_bias_g) << ",\n"
      << "  \"gyro_bias_dps\": " << vec_json(cal.gyro_bias_dps) << "\n}\n";
    return o.str();
}

std::string csv_header() {
    return "started_utc,serial,station,result,first_failure,fault_injected,duration_ms,transfers,retries,failures";
}

std::string to_csv_row(const UnitReport& r) {
    std::ostringstream o;
    o << r.started_utc << ',' << csv_field(r.serial) << ',' << csv_field(r.station) << ','
      << (r.passed ? "PASS" : "FAIL") << ',' << csv_field(r.first_failure) << ',' << r.fault_injected << ','
      << num(r.duration_ms, 3) << ',' << r.bus_stats.transfers << ',' << r.bus_stats.retries << ','
      << r.bus_stats.failures;
    return o.str();
}

void print_unit_table(std::ostream& os, const UnitReport& r) {
    os << "Unit " << r.serial << "  |  " << r.station << "  |  bus: " << r.bus;
    if (r.fault_injected != "none") os << "  |  injected fault: " << r.fault_injected;
    os << "\n\n";
    os << std::left << "  " << std::setw(16) << "TEST" << std::setw(8) << "RESULT" << std::setw(52) << "MEASURED"
       << "LIMIT\n";
    std::size_t passed = 0;
    for (const auto& t : r.results) {
        if (t.verdict == Verdict::Pass) ++passed;
        os << "  " << std::setw(16) << t.name << std::setw(8) << to_string(t.verdict) << std::setw(52)
           << (t.measured.empty() ? "-" : t.measured) << (t.limit.empty() ? "-" : t.limit) << "\n";
        if (!t.message.empty() && t.verdict != Verdict::Pass) os << "  " << std::setw(24) << "" << "-> " << t.message << "\n";
    }
    os << "\nRESULT: " << (r.passed ? "PASS" : "FAIL") << "  (" << passed << "/" << r.results.size() << " passed, "
       << r.bus_stats.retries << " bus retries, " << num(r.duration_ms, 1) << " ms)\n";
}

void write_text_file(const std::string& path, const std::string& text) {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << text;
}

void append_csv(const std::string& path, const UnitReport& r) {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    const bool is_new = !std::filesystem::exists(p) || std::filesystem::file_size(p) == 0;
    std::ofstream out(p, std::ios::app);
    if (!out) throw std::runtime_error("cannot append to " + path);
    if (is_new) out << csv_header() << "\n";
    out << to_csv_row(r) << "\n";
}

}  // namespace hwdiag
