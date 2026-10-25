# hwdiag — hardware diagnostics, calibration and production test station

`hwdiag` is a C++17 command-line test station for an I2C sensor board (TMP102 temperature
sensor + MPU-6050 IMU). It probes the bus, verifies part IDs, characterizes sensor noise,
calibrates IMU offsets and verifies the calibration, then logs results as JSON and CSV the way a
factory test station would. It runs on real hardware through the Linux `i2c-dev` driver
interface (for example a Raspberry Pi), or on a register-accurate simulated board on macOS or
Linux, with injectable manufacturing defects to prove the test plan catches them.

```
Unit HWD1-0002  |  HWD-1 Final Functional Test  |  bus: sim  |  injected fault: temp_offset

  TEST            RESULT  MEASURED                                  LIMIT
  bus_scan        PASS    0x48,0x68 present                         all present
  imu_id          PASS    0x68                                      == 0x68
  temp_range      PASS    41.38 C                                   [10.0, 45.0] C
  temp_agree      FAIL    delta 12.69 C (TMP102 41.3, IMU 28.6)     <= 8.0 C
  imu_gravity     PASS    |a| = 0.9943 g                            1.000 +/- 0.100 g
  imu_noise       PASS    accel 0.0042 g, gyro 0.053 dps            <= 0.020 g, <= 0.50 dps
  imu_calibrate   PASS    bias 0.0292 g / 1.74 dps; residual ...    bias <= 0.080 g / 5.0 dps ...

RESULT: FAIL  (6/7 passed, 0 bus retries, 0.2 ms)
```

The temperature sensor above reads within its range limit, so a range check alone would ship it.
The cross-check against the IMU's die temperature catches it.

## Features

- **Hardware abstraction**: one `I2CBus` interface with two backends — `LinuxI2CBus`
  (`/dev/i2c-N`, combined write/repeated-START/read via the `I2C_RDWR` ioctl, errno mapped to
  NACK / timeout / I/O error) and `SimBus`.
- **Drivers** for TMP102 and MPU-6050 written against the datasheet register maps, including
  wake-from-sleep with readback verification and 14-byte burst reads.
- **Retry layer** that counts retries, so marginal boards (intermittent NACKs) pass but are
  flagged in the logs.
- **Spec-driven tests**: limits and sequence live in `specs/sensor_board.spec`, so test limits
  change without recompiling. Seven test types: `probe`, `reg_equals`, `temp_range`,
  `temp_agree`, `accel_gravity`, `imu_noise`, `imu_calibrate`.
- **Calibration**: Welford mean/stddev over N samples, per-axis offset estimation, limit checks,
  verification pass with offsets applied, per-serial `.cal.json` output.
- **Multi-unit batch runs** on a worker thread pool (like a multi-slot fixture), deterministic
  regardless of thread count, with yield, failure Pareto and fault-injection coverage.
- **Command-line utilities**: `scan` (like `i2cdetect`), `dump` (like `i2cdump`), `run`, `batch`.
- **Bash station script** that runs a batch, archives logs, cross-checks yield from the CSV with
  `awk`, and gates on a minimum yield through its exit code.
- **Tests and CI**: 20 unit tests + CLI end-to-end tests, GitHub Actions on Ubuntu and macOS,
  plus AddressSanitizer, UndefinedBehaviorSanitizer and ThreadSanitizer jobs.

## Build and test

Requires CMake 3.16+ and a C++17 compiler (Apple Clang, Clang or GCC). No other dependencies.

```bash
cmake -S . -B build -DHWDIAG_WERROR=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Usage

```bash
./build/hwdiag faults                                    # list injectable defects
./build/hwdiag scan --bus sim                            # i2cdetect-style grid
./build/hwdiag dump --bus sim --addr 0x68 --start 0x60 --len 32
./build/hwdiag run --spec specs/sensor_board.spec --bus sim
./build/hwdiag run --spec specs/sensor_board.spec --bus sim --fault imu_noisy --serial HWD1-0007
./build/hwdiag batch --spec specs/sensor_board.spec --units 200 --threads 8 --min-yield 80
scripts/station.sh -n 200 -j 8 -y 80
```

`run` exits 0 on pass, 1 on fail, 2 on usage or setup errors, so it can be scripted.

### Batch output (200 simulated units, 15% defect rate, seed 42)

```
Units:       200   threads: 8   wall: 21.8 ms   throughput: 9189.4 units/s
Yield:       179/200 = 89.5%   (passed but needed bus retries: 7)

Failure Pareto (first failing test):
  imu_calibrate        9  #########
  imu_id               5  #####
  imu_noise            3  ###
  bus_scan             2  ##
  imu_gravity          2  ##

Fault-injection coverage:
  injected           units  failed   first failing test
  bus_intermittent       7       0   -
  imu_high_bias          9       9   imu_calibrate x9
  imu_missing            2       2   bus_scan x2
  imu_noisy              3       3   imu_noise x3
  imu_stuck_z            2       2   imu_gravity x2
  imu_wrong_part         5       5   imu_id x5
  none                 172       0   -

Test escapes (defective unit passed): 0   False failures (good unit failed): 0
```

With 20 µs of simulated bus latency per transfer (`--latency-us 20`), 40 units took 2145 ms on
1 thread and 281 ms on 8 threads (7.6x) on a cloud Linux VM.

## Running on a Raspberry Pi

1. Enable I2C: `sudo raspi-config` → Interface Options → I2C, then reboot.
2. Wire a TMP102 breakout and an MPU-6050 (GY-521) breakout in parallel: 3.3 V → pin 1,
   GND → pin 6, SDA → pin 3 (GPIO2), SCL → pin 5 (GPIO3).
3. Lay the board flat, then:

```bash
sudo apt install cmake g++ i2c-tools
cmake -S . -B build && cmake --build build -j
i2cdetect -y 1                                   # should show 48 and 68
./build/hwdiag scan --bus /dev/i2c-1
./build/hwdiag run --spec specs/sensor_board.spec --bus /dev/i2c-1 --serial PI-0001
```

## Layout

```
include/hwdiag/   public headers (bus, drivers, calibration, spec, runner, report, batch, faults)
src/              implementation + main.cpp (CLI)
specs/            test specification files
scripts/          station.sh production wrapper
tests/            unit tests (self-contained, no framework dependency)
.github/          CI: Ubuntu + macOS builds, ASan/UBSan/TSan
```

## License

MIT
