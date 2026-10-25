#include "hwdiag/faults.hpp"

#include "hwdiag/sim_devices.hpp"

#include <random>

namespace hwdiag {

namespace {

struct FaultInfo {
    Fault fault;
    const char* name;
    const char* description;
};

const FaultInfo kFaults[] = {
    {Fault::None, "none", "good unit (normal part-to-part variation only)"},
    {Fault::ImuMissing, "imu_missing", "IMU not placed or open solder joint"},
    {Fault::ImuWrongPart, "imu_wrong_part", "wrong IMU populated (WHO_AM_I 0x70)"},
    {Fault::ImuHighBias, "imu_high_bias", "accel/gyro offset out of spec"},
    {Fault::ImuStuckZ, "imu_stuck_z", "accel Z output stuck at zero"},
    {Fault::ImuNoisy, "imu_noisy", "excessive sensor noise (bad decoupling)"},
    {Fault::TempMissing, "temp_missing", "temperature sensor not placed"},
    {Fault::TempOffset, "temp_offset", "temperature sensor reads 15 C high"},
    {Fault::BusIntermittent, "bus_intermittent", "marginal pull-ups, ~5% of transfers NACK"},
};

}  // namespace

const std::vector<Fault>& injectable_faults() {
    static const std::vector<Fault> faults = [] {
        std::vector<Fault> v;
        for (const auto& f : kFaults) {
            if (f.fault != Fault::None) v.push_back(f.fault);
        }
        return v;
    }();
    return faults;
}

const char* fault_name(Fault f) {
    for (const auto& info : kFaults) {
        if (info.fault == f) return info.name;
    }
    return "unknown";
}

const char* fault_description(Fault f) {
    for (const auto& info : kFaults) {
        if (info.fault == f) return info.description;
    }
    return "";
}

std::optional<Fault> parse_fault(const std::string& name) {
    for (const auto& info : kFaults) {
        if (name == info.name) return info.fault;
    }
    return std::nullopt;
}

std::unique_ptr<SimBus> build_sim_board(Fault fault, uint32_t seed, std::chrono::microseconds latency) {
    std::mt19937 rng(seed);
    auto uniform = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };

    SimBusConfig bus_cfg;
    bus_cfg.seed = static_cast<uint32_t>(rng());
    bus_cfg.latency = latency;
    if (fault == Fault::BusIntermittent) bus_cfg.nack_probability = 0.05;
    auto bus = std::make_unique<SimBus>(bus_cfg);

    // Normal unit-to-unit variation for a good board.
    const double ambient = uniform(22.0, 28.0);

    Tmp102Params tmp;
    tmp.ambient_c = ambient;
    tmp.seed = static_cast<uint32_t>(rng());

    Mpu6050Params imu;
    imu.die_temp_c = ambient + uniform(2.0, 4.0);  // IMU die runs a few degrees warm
    imu.accel_bias_g = {uniform(-0.03, 0.03), uniform(-0.03, 0.03), uniform(-0.03, 0.03)};
    imu.gyro_bias_dps = {uniform(-2.0, 2.0), uniform(-2.0, 2.0), uniform(-2.0, 2.0)};
    imu.seed = static_cast<uint32_t>(rng());

    switch (fault) {
        case Fault::ImuWrongPart: imu.who_am_i = 0x70; break;
        case Fault::ImuHighBias:
            imu.accel_bias_g.x = 0.20;
            imu.gyro_bias_dps.z = 12.0;
            break;
        case Fault::ImuStuckZ: imu.stuck_axis = 2; break;
        case Fault::ImuNoisy:
            imu.accel_noise_g = 0.05;
            imu.gyro_noise_dps = 2.0;
            break;
        case Fault::TempOffset: tmp.offset_c = 15.0; break;
        default: break;
    }

    if (fault != Fault::TempMissing) bus->attach(SimTmp102::kAddr, std::make_unique<SimTmp102>(tmp));
    if (fault != Fault::ImuMissing) bus->attach(SimMpu6050::kAddr, std::make_unique<SimMpu6050>(imu));
    return bus;
}

}  // namespace hwdiag
