#include "hwdiag/sim_bus.hpp"

#include <thread>
#include <utility>

namespace hwdiag {

SimBus::SimBus(SimBusConfig cfg) : cfg_(cfg), rng_(cfg.seed) {}

void SimBus::attach(uint8_t addr, std::unique_ptr<SimDevice> device) {
    devices_[addr] = std::move(device);
}

SimDevice* SimBus::device(uint8_t addr) {
    auto it = devices_.find(addr);
    return it == devices_.end() ? nullptr : it->second.get();
}

bool SimBus::inject_nack() {
    if (cfg_.nack_probability <= 0.0) return false;
    std::bernoulli_distribution nack(cfg_.nack_probability);
    return nack(rng_);
}

void SimBus::wait() const {
    if (cfg_.latency.count() > 0) std::this_thread::sleep_for(cfg_.latency);
}

BusStatus SimBus::read_regs(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) {
    ++transfers_;
    wait();
    SimDevice* dev = device(addr);
    if (dev == nullptr || inject_nack()) return BusStatus::Nack;
    return dev->read(reg, out, len) ? BusStatus::Ok : BusStatus::Nack;
}

BusStatus SimBus::write_regs(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) {
    ++transfers_;
    wait();
    SimDevice* dev = device(addr);
    if (dev == nullptr || inject_nack()) return BusStatus::Nack;
    return dev->write(reg, data, len) ? BusStatus::Ok : BusStatus::Nack;
}

bool SimBus::probe(uint8_t addr) {
    ++transfers_;
    wait();
    return device(addr) != nullptr && !inject_nack();
}

}  // namespace hwdiag
