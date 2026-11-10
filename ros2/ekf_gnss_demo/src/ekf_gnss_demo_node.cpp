// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

rclcpp::Time time_from_seconds(double t_s) {
  const int64_t ns = static_cast<int64_t>(std::llround(t_s * 1.0e9));
  return rclcpp::Time(ns);
}

}  // namespace

class EkfGnssDemoNode final : public rclcpp::Node {
 public:
  EkfGnssDemoNode() : Node("ekf_gnss_demo_node") {
    declare_parameter<std::string>("scenario", "matched");
    declare_parameter<int>("seed", 1);
    declare_parameter<double>("replay_rate_hz", 100.0);
    const auto scenario = get_parameter("scenario").as_string();
    const auto seed = static_cast<std::uint64_t>(get_parameter("seed").as_int());
    const double hz = get_parameter("replay_rate_hz").as_double();
    result_ = ekf::sim::RunScenario(scenario.c_str(), seed);
    idx_ = 0;

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ekf_gnss/odometry", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_gnss/pose", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(hz, 1.0));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&EkfGnssDemoNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "ekf_gnss_demo replay %s seed %llu (%zu ticks)", scenario.c_str(),
                static_cast<unsigned long long>(seed), result_.trace.ticks.size());
  }

 private:
  void on_timer() {
    while (idx_ < result_.trace.ticks.size()) {
      const auto& tick = result_.trace.ticks[idx_++];
      if (!tick.filter_initialized) {
        continue;
      }
      const auto stamp = time_from_seconds(tick.t_s);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "map";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = tick.est_e_m;
      odom.pose.pose.position.y = tick.est_n_m;
      const double half = 0.5 * tick.est_psi_rad;
      odom.pose.pose.orientation.z = std::sin(half);
      odom.pose.pose.orientation.w = std::cos(half);
      odom.twist.twist.linear.x = tick.est_v_m_s;
      odom_pub_->publish(odom);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = odom.header;
      pose.pose = odom.pose.pose;
      pose_pub_->publish(pose);
      return;
    }
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "ekf_gnss_demo replay finished (rmse %.3f m)",
                result_.metrics.rmse_horizontal_m);
  }

  ekf::sim::RunResult result_{};
  std::size_t idx_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfGnssDemoNode>());
  rclcpp::shutdown();
  return 0;
}
