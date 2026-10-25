#pragma once
// Output formats: JSON per unit (machine-readable, for a factory database),
// one CSV row per unit (for quick yield analysis in Excel), and a console table.

#include "hwdiag/runner.hpp"

#include <ostream>
#include <string>

namespace hwdiag {

std::string json_escape(const std::string& s);
std::string to_json(const UnitReport& r);
std::string calibration_json(const std::string& serial, const ImuCalibration& cal);

std::string csv_header();
std::string to_csv_row(const UnitReport& r);

void print_unit_table(std::ostream& os, const UnitReport& r);

// Writes text to a file, creating parent directories. Throws std::runtime_error on failure.
void write_text_file(const std::string& path, const std::string& text);
// Appends a CSV row, writing the header first if the file is new.
void append_csv(const std::string& path, const UnitReport& r);

}  // namespace hwdiag
