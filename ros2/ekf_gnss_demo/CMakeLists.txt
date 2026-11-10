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
ament_target_dependencies(ekf_gnss_demo_node rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs)
target_link_libraries(ekf_gnss_demo_node ekf_sim)
target_compile_features(ekf_gnss_demo_node PUBLIC cxx_std_17)

install(TARGETS sim_sensor_node ekf_gnss_demo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS test/check_odom_rmse.py
  DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ekf_gnss_demo.launch.py TIMEOUT 180)
endif()

ament_package()
