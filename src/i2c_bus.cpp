#include "hwdiag/i2c_bus.hpp"

namespace hwdiag {

const char* to_string(BusStatus s) {
    switch (s) {
        case BusStatus::Ok: return "ok";
        case BusStatus::Nack: return "nack";
        case BusStatus::Timeout: return "timeout";
        case BusStatus::IoError: return "io_error";
    }
    return "unknown";
}

}  // namespace hwdiag
