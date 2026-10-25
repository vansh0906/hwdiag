# HWD-1 sensor board - final functional test
# Board: TMP102 temperature sensor (0x48) + MPU-6050 IMU (0x68) on one I2C bus.
# Fixture holds the board flat (Z axis up) at room temperature.
# Tests run in order; every test runs unless --stop-on-fail is given.

[station]
name = HWD-1 Final Functional Test
retries = 3          # bus retries per transfer before a test reports an error

[test bus_scan]
type = probe
addrs = 0x48, 0x68   # every populated device must ACK

[test imu_id]
type = reg_equals    # catches a wrong part on the reel
addr = 0x68
reg = 0x75           # WHO_AM_I
expect = 0x68

[test temp_range]
type = temp_range
min = 10             # C, fixture is in a climate-controlled room
max = 45

[test temp_agree]
type = temp_agree    # TMP102 and IMU die temperature must roughly agree
max_delta = 8        # C, IMU die runs 2-4 C warm

[test imu_gravity]
type = accel_gravity
samples = 50
tolerance_g = 0.10   # |a| must be 1 g +/- 0.10 g at rest

[test imu_noise]
type = imu_noise
samples = 200
max_accel_std_g = 0.02
max_gyro_std_dps = 0.5

[test imu_calibrate]
type = imu_calibrate
samples = 200
max_accel_bias_g = 0.08     # larger offsets indicate a damaged or stressed part
max_gyro_bias_dps = 5.0
verify_tolerance_g = 0.01   # residual after applying the calibration
