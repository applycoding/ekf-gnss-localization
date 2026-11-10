# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(ekf_gnss_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(EKF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(EKF_BUILD_WASM OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../.. ${CMAKE_BINARY_DIR}/ekf_root EXCLUDE_FROM_ALL)

add_executable(sim_sensor_node src/sim_sensor_node.cpp)
ament_target_dependencies(sim_sensor_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)
target_link_libraries(sim_sensor_node ekf_sim)
target_compile_features(sim_sensor_node PUBLIC cxx_std_17)

add_executable(ekf_gnss_demo_node src/ekf_gnss_demo_node.cpp)
ament_target_dependencies(ekf_gnss_demo_node rclcpp nav_msgs geometry_msgs std_msgs)
target_link_libraries(ekf_gnss_demo_node ekf_sim)
target_compile_features(ekf_gnss_demo_node PUBLIC cxx_std_17)

install(TARGETS sim_sensor_node ekf_gnss_demo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(
    NAME smoke_ekf_node
    COMMAND ekf_gnss_demo_node --ros-args -p replay_rate_hz:=8000.0
  )
  set_tests_properties(smoke_ekf_node PROPERTIES
    TIMEOUT 60
    PASS_REGULAR_EXPRESSION "ekf_gnss_demo replay finished"
  )
  add_test(
    NAME smoke_sim_node
    COMMAND sim_sensor_node --ros-args -p replay_rate_hz:=8000.0
  )
  set_tests_properties(smoke_sim_node PROPERTIES
    TIMEOUT 60
    PASS_REGULAR_EXPRESSION "sim_sensor replay"
  )
endif()

ament_package()
