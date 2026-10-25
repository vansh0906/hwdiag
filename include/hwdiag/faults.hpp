#pragma once
// Fault catalog for the simulated sensor board. Each fault models a real
// production defect; injecting them proves the test spec catches every one
// (test coverage) without failing good units (false failures).

#include "hwdiag/sim_bus.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hwdiag {

enum class Fault {
    None,
    ImuMissing,       // IMU not placed / cold solder joint on SDA
    ImuWrongPart,     // wrong part number populated (WHO_AM_I = 0x70, an MPU-6500)
    ImuHighBias,      // accelerometer/gyro offset out of spec (mechanical stress on package)
    ImuStuckZ,        // accel Z output stuck at zero
    ImuNoisy,         // excessive noise (bad decoupling capacitor)
    TempMissing,      // temperature sensor not placed
    TempOffset,       // temperature sensor reads +15 C high
    BusIntermittent,  // marginal pull-ups: ~5% of transfers NACK
};

const std::vector<Fault>& injectable_faults();  // every fault except None
const char* fault_name(Fault f);
const char* fault_description(Fault f);
std::optional<Fault> parse_fault(const std::string& name);

// Builds one simulated HWD-1 sensor board (TMP102 + MPU-6050) with normal
// unit-to-unit variation drawn from `seed`, plus the requested fault.
std::unique_ptr<SimBus> build_sim_board(Fault fault, uint32_t seed,
                                        std::chrono::microseconds latency = std::chrono::microseconds{0});

}  // namespace hwdiag
