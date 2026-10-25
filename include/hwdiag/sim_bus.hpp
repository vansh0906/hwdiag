#pragma once
// Simulated I2C bus. Devices are attached at addresses; the bus can inject
// intermittent NACKs and per-transaction latency to model a marginal board or
// real bus timing. One SimBus models one unit under test and is not shared
// between threads.

#include "hwdiag/i2c_bus.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <random>

namespace hwdiag {

class SimDevice {
public:
    virtual ~SimDevice() = default;
    // Return false to NACK the transfer.
    virtual bool read(uint8_t reg, uint8_t* out, std::size_t len) = 0;
    virtual bool write(uint8_t reg, const uint8_t* data, std::size_t len) = 0;
};

struct SimBusConfig {
    double nack_probability = 0.0;          // chance any single transfer is NACKed
    std::chrono::microseconds latency{0};   // added to every transfer
    uint32_t seed = 1;
};

class SimBus : public I2CBus {
public:
    explicit SimBus(SimBusConfig cfg = {});

    void attach(uint8_t addr, std::unique_ptr<SimDevice> device);
    SimDevice* device(uint8_t addr);

    BusStatus read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) override;
    BusStatus write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) override;
    bool probe(uint8_t addr) override;
    std::string name() const override { return "sim"; }

    uint64_t transfers() const { return transfers_; }

private:
    bool inject_nack();
    void wait() const;

    SimBusConfig cfg_;
    std::map<uint8_t, std::unique_ptr<SimDevice>> devices_;
    std::mt19937 rng_;
    uint64_t transfers_ = 0;
};

}  // namespace hwdiag
