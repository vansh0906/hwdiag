#pragma once
// MPU-6050 register map (from the InvenSense register map, rev 4.2).
// Shared by the driver and the simulated device so both agree on layout.

#include <cstdint>

namespace hwdiag::mpu6050 {

constexpr uint8_t kDefaultAddr = 0x68;  // AD0 pin low
constexpr uint8_t kExpectedId = 0x68;   // WHO_AM_I value for a genuine MPU-6050

constexpr uint8_t GYRO_CONFIG = 0x1B;   // bits 4:3 FS_SEL  (0 = +-250 dps)
constexpr uint8_t ACCEL_CONFIG = 0x1C;  // bits 4:3 AFS_SEL (0 = +-2 g)
constexpr uint8_t ACCEL_XOUT_H = 0x3B;  // 0x3B..0x40 accel X/Y/Z, big-endian int16
constexpr uint8_t TEMP_OUT_H = 0x41;    // 0x41..0x42 die temperature
constexpr uint8_t GYRO_XOUT_H = 0x43;   // 0x43..0x48 gyro X/Y/Z
constexpr uint8_t GYRO_ZOUT_L = 0x48;
constexpr uint8_t PWR_MGMT_1 = 0x6B;    // bit 7 DEVICE_RESET, bit 6 SLEEP, bits 2:0 CLKSEL
constexpr uint8_t WHO_AM_I = 0x75;

constexpr uint8_t PWR_SLEEP = 0x40;
constexpr uint8_t PWR_RESET = 0x80;
constexpr uint8_t PWR_CLK_PLL_XGYRO = 0x01;

constexpr double kAccelLsbPerG_2g = 16384.0;
constexpr double kGyroLsbPerDps_250 = 131.0;
constexpr double kTempLsbPerC = 340.0;
constexpr double kTempOffsetC = 36.53;

}  // namespace hwdiag::mpu6050
