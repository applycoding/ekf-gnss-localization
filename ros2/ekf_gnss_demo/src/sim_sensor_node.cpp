// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/constants.hpp"
#include "ekf_sim/scenario.hpp"
#include "ekf_sim/scheduler.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

rclcpp::Time time_from_seconds(double t_s) {
  const int64_t ns = static_cast<int64_t>(std::llround(t_s * 1.0e9));
  return rclcpp::Time(ns);
}

void fill_nav_sat(const ekf::GnssSample& gnss, sensor_msgs::msg::NavSatFix& fix) {
  constexpr double kRadToDeg = 180.0 / ekf::kPi;
  fix.latitude = gnss.lat_rad * kRadToDeg;
  fix.longitude = gnss.lon_rad * kRadToDeg;
  fix.altitude = gnss.h_m;
  fix.position_covariance[0] = 1.5 * 1.5;
  fix.position_covariance[4] = 1.5 * 1.5;
  fix.position_covariance[8] = 4.0 * 4.0;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
}

}  // namespace

class SimSensorNode final : public rclcpp::Node {
 public:
  SimSensorNode() : Node("sim_sensor_node") {
    declare_parameter<std::string>("scenario", "matched");
    declare_parameter<int>("seed", 1);
    declare_parameter<double>("replay_rate_hz", 100.0);
    const auto scenario = get_parameter("scenario").as_string();
    const auto seed = static_cast<std::uint64_t>(get_parameter("seed").as_int());
    const double hz = get_parameter("replay_rate_hz").as_double();

    const ekf::sim::ScenarioSpec spec = ekf::sim::LoadScenarioByName(scenario.c_str());
    ekf::sim::RunSimulation(spec, seed, spec.filter.gate, &log_);
    idx_ = 0;

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sim/imu", 10);
    wheel_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/wheel", 10);
    gnss_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/sim/gnss/fix", 10);
    gnss_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/gnss/pose", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(hz, 1.0));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&SimSensorNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "sim_sensor replay %s seed %llu (%zu ticks)", scenario.c_str(),
                static_cast<unsigned long long>(seed), log_.ticks.size());
  }

 private:
  void on_timer() {
    if (idx_ >= log_.ticks.size()) {
      timer_->cancel();
      return;
    }
    const std::size_t tick_ix = idx_++;
    const auto& m = log_.ticks[tick_ix];
    const double t_s = static_cast<double>(tick_ix) * ekf::sim::kDtS;
    const auto stamp = time_from_seconds(t_s);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.angular_velocity.z = m.imu.gyro_z_rad_s;
    imu.linear_acceleration.x = m.imu.accel_x_m_s2;
    for (int i = 0; i < 9; ++i) {
      imu.orientation_covariance[i] = -1.0;
    }
    imu_pub_->publish(imu);

    if (m.have_wheel) {
      nav_msgs::msg::Odometry wheel;
      wheel.header.stamp = stamp;
      wheel.header.frame_id = "map";
      wheel.child_frame_id = "base_link";
      wheel.twist.twist.linear.x = m.wheel.speed_m_s;
      wheel_pub_->publish(wheel);
    }

    if (m.have_gnss) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = stamp;
      fix.header.frame_id = "gps_link";
      fill_nav_sat(m.gnss, fix);
      gnss_fix_pub_->publish(fix);

      nav_msgs::msg::Odometry pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = "map";
      pose.child_frame_id = "gps_link";
      pose.pose.pose.position.x = m.gnss_meas_e;
      pose.pose.pose.position.y = m.gnss_meas_n;
      pose.pose.covariance[0] = 1.5 * 1.5;
      pose.pose.covariance[7] = 1.5 * 1.5;
      gnss_pose_pub_->publish(pose);
    }
  }

  ekf::sim::MeasurementLog log_{};
  std::size_t idx_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSensorNode>());
  rclcpp::shutdown();
  return 0;
}
