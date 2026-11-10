# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(ekf_gnss_localization VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EKF_BUILD_TESTS "Build GoogleTest suite" ON)
option(EKF_BUILD_WASM "Build WASM module for web" OFF)

add_library(ekf_core STATIC
  cpp/src/core/geodesy.cpp
  cpp/src/core/filter.cpp
  cpp/src/core/initializer.cpp
)
target_include_directories(ekf_core PUBLIC cpp/include)
target_compile_options(ekf_core PRIVATE
  -Wall -Wextra -Wpedantic -Werror
  -ffp-contract=off
)

if(EKF_BUILD_TESTS OR NOT EKF_BUILD_WASM)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG 063de7e9578f82b369302001269680b4b1553359
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG 55f93686c01528224f448c19128836e7df245f72
)
FetchContent_MakeAvailable(nlohmann_json)

set(EKF_SIM_SOURCES
  cpp/src/sim/rng.cpp
  cpp/src/sim/trajectory.cpp
  cpp/src/sim/sensors.cpp
  cpp/src/sim/scenario.cpp
  cpp/src/sim/scheduler.cpp
  cpp/src/sim/baselines.cpp
  cpp/src/sim/metrics.cpp
  cpp/src/sim/trace.cpp
)

add_library(ekf_sim STATIC ${EKF_SIM_SOURCES})
target_include_directories(ekf_sim PUBLIC cpp/include)
target_link_libraries(ekf_sim PUBLIC ekf_core nlohmann_json::nlohmann_json)
target_compile_options(ekf_sim PRIVATE -Wall -Wextra -Wpedantic)
if(NOT EKF_BUILD_WASM)
  target_compile_options(ekf_sim PRIVATE -Werror)
endif()

if(EKF_BUILD_TESTS OR NOT EKF_BUILD_WASM)
  if(NOT TARGET ekf_cli)
    add_executable(ekf_cli cpp/tools/ekf_cli.cpp)
    target_link_libraries(ekf_cli PRIVATE ekf_sim)
  endif()
endif()

if(EKF_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)

  macro(ekf_gtest target src)
    add_executable(${target} ${src})
    target_link_libraries(${target} PRIVATE ekf_core ekf_sim GTest::gtest_main)
    gtest_discover_tests(${target})
  endmacro()

  add_executable(test_unit cpp/test/test_unit.cpp)
  target_link_libraries(test_unit PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_unit)

  add_executable(test_exact_2state cpp/test/test_exact_2state.cpp)
  target_link_libraries(test_exact_2state PRIVATE GTest::gtest_main)
  gtest_discover_tests(test_exact_2state)

  add_executable(test_filter_frozen cpp/test/test_filter_frozen.cpp)
  target_link_libraries(test_filter_frozen PRIVATE ekf_core GTest::gtest_main)
  gtest_discover_tests(test_filter_frozen)

  add_executable(test_jacobian cpp/test/test_jacobian.cpp)
  target_link_libraries(test_jacobian PRIVATE ekf_core GTest::gtest_main)
  gtest_discover_tests(test_jacobian)

  add_executable(test_geodesy cpp/test/test_geodesy.cpp)
  target_link_libraries(test_geodesy PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_geodesy)

  add_executable(test_invalid_inputs cpp/test/test_invalid_inputs.cpp)
  target_link_libraries(test_invalid_inputs PRIVATE ekf_core GTest::gtest_main)
  gtest_discover_tests(test_invalid_inputs)

  add_executable(test_gates cpp/test/test_gates.cpp)
  target_link_libraries(test_gates PRIVATE ekf_core GTest::gtest_main)
  gtest_discover_tests(test_gates)

  ekf_gtest(test_scenario cpp/test/test_scenario.cpp)
  ekf_gtest(test_reacquisition cpp/test/test_reacquisition.cpp)

  add_executable(test_monte_carlo cpp/test/test_monte_carlo.cpp)
  target_link_libraries(test_monte_carlo PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_monte_carlo PROPERTIES LABELS "mc")

  ekf_gtest(test_determinism cpp/test/test_determinism.cpp)

  add_executable(test_no_heap_operator cpp/test/test_no_heap_operator.cpp)
  target_link_libraries(test_no_heap_operator PRIVATE ekf_core)
  target_compile_options(test_no_heap_operator PRIVATE -fno-exceptions)
  add_test(NAME NoHeapOperator.FilterStepsWithoutAlloc COMMAND test_no_heap_operator)

  add_test(NAME Determinism.O0O2Parity COMMAND bash ${CMAKE_SOURCE_DIR}/tools/o0_o2_parity.sh)
endif()

if(EKF_BUILD_WASM)
  add_executable(ekf_wasm cpp/src/wasm/ekf_wasm.cpp)
  target_link_libraries(ekf_wasm PRIVATE ekf_sim)
  set_target_properties(ekf_wasm PROPERTIES
    OUTPUT_NAME "ekf"
    SUFFIX ".js"
  )
  target_link_options(ekf_wasm PRIVATE
    "-sMODULARIZE=1"
    "-sEXPORT_NAME=createEkfModule"
    "-sENVIRONMENT=web"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sEXPORTED_FUNCTIONS=['_malloc','_free']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue','HEAPF64']"
    "--bind"
  )
endif()
