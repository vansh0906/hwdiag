#include "hwdiag/spec.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>

namespace hwdiag {

namespace {

std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::string strip_comment(const std::string& s) {
    const auto pos = s.find_first_of("#;");
    return pos == std::string::npos ? s : s.substr(0, pos);
}

[[noreturn]] void fail(const std::string& source, int line, const std::string& msg) {
    throw SpecError(source + ":" + std::to_string(line) + ": " + msg);
}

}  // namespace

double parse_double(const std::string& text) {
    const std::string t = trim(text);
    if (t.empty()) throw SpecError("expected a number, got empty value");
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(t.c_str(), &end);
    if (errno != 0 || end == t.c_str() || *end != '\0') throw SpecError("not a number: '" + t + "'");
    return v;
}

long parse_long(const std::string& text) {
    const std::string t = trim(text);
    if (t.empty()) throw SpecError("expected an integer, got empty value");
    char* end = nullptr;
    errno = 0;
    const long v = std::strtol(t.c_str(), &end, 0);  // base 0: "0x68" or "104"
    if (errno != 0 || end == t.c_str() || *end != '\0') throw SpecError("not an integer: '" + t + "'");
    return v;
}

const std::string& TestSpec::str(const std::string& key) const {
    auto it = params.find(key);
    if (it == params.end()) {
        throw SpecError("test '" + name + "' (line " + std::to_string(line) + "): missing parameter '" + key + "'");
    }
    return it->second;
}

double TestSpec::num(const std::string& key) const {
    try {
        return parse_double(str(key));
    } catch (const SpecError& e) {
        throw SpecError("test '" + name + "' parameter '" + key + "': " + e.what());
    }
}

double TestSpec::num_or(const std::string& key, double fallback) const {
    return has(key) ? num(key) : fallback;
}

long TestSpec::integer(const std::string& key) const {
    try {
        return parse_long(str(key));
    } catch (const SpecError& e) {
        throw SpecError("test '" + name + "' parameter '" + key + "': " + e.what());
    }
}

long TestSpec::integer_or(const std::string& key, long fallback) const {
    return has(key) ? integer(key) : fallback;
}

std::vector<long> TestSpec::int_list(const std::string& key) const {
    std::vector<long> out;
    std::stringstream ss(str(key));
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (trim(item).empty()) continue;
        out.push_back(parse_long(item));
    }
    if (out.empty()) throw SpecError("test '" + name + "' parameter '" + key + "': empty list");
    return out;
}

std::string StationSpec::station_name() const {
    auto it = station.find("name");
    return it == station.end() ? "unnamed station" : it->second;
}

int StationSpec::retries() const {
    auto it = station.find("retries");
    return it == station.end() ? 3 : static_cast<int>(parse_long(it->second));
}

StationSpec parse_spec(std::istream& in, const std::string& source_name) {
    enum class Section { None, Station, Test };

    StationSpec spec;
    spec.source = source_name;
    Section section = Section::None;
    std::set<std::string> names;
    std::string raw;
    int lineno = 0;

    while (std::getline(in, raw)) {
        ++lineno;
        const std::string line = trim(strip_comment(raw));
        if (line.empty()) continue;

        if (line.front() == '[') {
            if (line.back() != ']') fail(source_name, lineno, "section header missing ']'");
            const std::string inner = trim(line.substr(1, line.size() - 2));
            if (inner == "station") {
                section = Section::Station;
            } else if (inner.rfind("test", 0) == 0 && inner.size() > 4 && (inner[4] == ' ' || inner[4] == '\t')) {
                const std::string name = trim(inner.substr(5));
                if (name.empty()) fail(source_name, lineno, "test section needs a name: [test <name>]");
                if (!names.insert(name).second) fail(source_name, lineno, "duplicate test name '" + name + "'");
                TestSpec t;
                t.name = name;
                t.line = lineno;
                spec.tests.push_back(std::move(t));
                section = Section::Test;
            } else {
                fail(source_name, lineno, "unknown section [" + inner + "]");
            }
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) fail(source_name, lineno, "expected 'key = value'");
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        if (key.empty()) fail(source_name, lineno, "empty key");

        switch (section) {
            case Section::None:
                fail(source_name, lineno, "'" + key + "' appears before any [section]");
            case Section::Station:
                spec.station[key] = value;
                break;
            case Section::Test:
                if (key == "type") {
                    spec.tests.back().type = value;
                } else {
                    spec.tests.back().params[key] = value;
                }
                break;
        }
    }

    for (const auto& t : spec.tests) {
        if (t.type.empty()) fail(source_name, t.line, "test '" + t.name + "' has no 'type'");
    }
    if (spec.tests.empty()) throw SpecError(source_name + ": no [test ...] sections found");
    return spec;
}

StationSpec load_spec(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw SpecError("cannot open spec file '" + path + "'");
    return parse_spec(in, path);
}

}  // namespace hwdiag
