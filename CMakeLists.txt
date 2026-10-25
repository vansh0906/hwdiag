cmake_minimum_required(VERSION 3.16)
project(hwdiag VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(HWDIAG_WERROR "Treat compiler warnings as errors" OFF)
set(HWDIAG_SANITIZE "" CACHE STRING "Sanitizer to enable: address, thread or undefined")

if(HWDIAG_SANITIZE)
  add_compile_options(-fsanitize=${HWDIAG_SANITIZE} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${HWDIAG_SANITIZE})
endif()

find_package(Threads REQUIRED)

function(hwdiag_warnings target)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
  if(HWDIAG_WERROR)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
endfunction()

add_library(hwdiag_core STATIC
  src/i2c_bus.cpp
  src/linux_i2c_bus.cpp
  src/sim_bus.cpp
  src/sim_devices.cpp
  src/drivers.cpp
  src/calibration.cpp
  src/spec.cpp
  src/runner.cpp
  src/report.cpp
  src/faults.cpp
  src/batch.cpp
)
target_include_directories(hwdiag_core PUBLIC include)
target_link_libraries(hwdiag_core PUBLIC Threads::Threads)
hwdiag_warnings(hwdiag_core)

add_executable(hwdiag src/main.cpp)
target_link_libraries(hwdiag PRIVATE hwdiag_core)
hwdiag_warnings(hwdiag)

enable_testing()

add_executable(hwdiag_tests tests/test_main.cpp)
target_link_libraries(hwdiag_tests PRIVATE hwdiag_core)
target_compile_definitions(hwdiag_tests PRIVATE HWDIAG_SPEC="${CMAKE_SOURCE_DIR}/specs/sensor_board.spec")
hwdiag_warnings(hwdiag_tests)

add_test(NAME unit_tests COMMAND hwdiag_tests)

# End-to-end CLI checks: a good unit must pass (exit 0) and a defective one must fail (exit 1).
add_test(NAME cli_good_unit_passes
  COMMAND hwdiag run --spec ${CMAKE_SOURCE_DIR}/specs/sensor_board.spec --bus sim
          --out ${CMAKE_BINARY_DIR}/test-reports)
add_test(NAME cli_defective_unit_fails
  COMMAND hwdiag run --spec ${CMAKE_SOURCE_DIR}/specs/sensor_board.spec --bus sim
          --fault imu_stuck_z --serial HWD1-BAD1 --out ${CMAKE_BINARY_DIR}/test-reports)
set_tests_properties(cli_defective_unit_fails PROPERTIES WILL_FAIL TRUE)
add_test(NAME cli_batch_yield_gate
  COMMAND hwdiag batch --spec ${CMAKE_SOURCE_DIR}/specs/sensor_board.spec --units 120 --threads 4
          --out ${CMAKE_BINARY_DIR}/test-reports/batch --min-yield 75)
