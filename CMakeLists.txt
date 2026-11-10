# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(ekf_gnss_localization VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EKF_BUILD_TESTS "Build GoogleTest suite" ON)
option(EKF_BUILD_PYTHON "Build pybind11 module" OFF)
option(EKF_BUILD_ROS "Build ROS 2 package" OFF)
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

if(EKF_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG 97bf890db679505a14dfe547a5e77bb2bd05dc90
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(EKF_BUILD_TESTS OR EKF_BUILD_WASM OR NOT EKF_BUILD_ROS)
  if(NOT EKF_BUILD_WASM)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG 063de7e9578f82b369302001269680b4b1553359
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()
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

if(EKF_BUILD_TESTS OR (NOT EKF_BUILD_WASM AND NOT EKF_BUILD_ROS))
  if(NOT TARGET ekf_cli)
    add_executable(ekf_cli cpp/tools/ekf_cli.cpp)
    target_link_libraries(ekf_cli PRIVATE ekf_sim)
  endif()
endif()

if(EKF_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)

  add_executable(test_unit cpp/test/test_unit.cpp)
  target_link_libraries(test_unit PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_unit)

  add_executable(test_filter_frozen cpp/test/test_filter_frozen.cpp)
  target_link_libraries(test_filter_frozen PRIVATE ekf_core GTest::gtest_main)
  gtest_discover_tests(test_filter_frozen)

  add_executable(test_scenario cpp/test/test_scenario.cpp)
  target_link_libraries(test_scenario PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_scenario)

  add_executable(test_monte_carlo cpp/test/test_monte_carlo.cpp)
  target_link_libraries(test_monte_carlo PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_monte_carlo
    PROPERTIES LABELS "mc"
  )

  add_executable(test_determinism cpp/test/test_determinism.cpp)
  target_link_libraries(test_determinism PRIVATE ekf_core ekf_sim GTest::gtest_main)
  gtest_discover_tests(test_determinism)

  add_executable(test_no_heap_operator cpp/test/test_no_heap_operator.cpp)
  target_link_libraries(test_no_heap_operator PRIVATE ekf_core GTest::gtest_main)
  target_compile_options(test_no_heap_operator PRIVATE -fno-exceptions)
  gtest_discover_tests(test_no_heap_operator)
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

if(EKF_BUILD_PYTHON)
  pybind11_add_module(ekf_py cpp/src/python/ekf_py.cpp)
  target_link_libraries(ekf_py PRIVATE ekf_sim)
endif()
