// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"
#include "ekf/geodesy.hpp"
#include "ekf_sim/constants.hpp"
#include "ekf_sim/scenario.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool same_stamp(const builtin_interfaces::msg::Time& a, const builtin_interfaces::msg::Time& b) {
  return a.sec == b.sec && a.nanosec == b.nanosec;
}

ekf::GnssSample gnss_from_pose(const nav_msgs::msg::Odometry& pose, const ekf::Origin& origin) {
  ekf::GnssSample gnss{};
  gnss.t_s = stamp_to_sec(pose.header.stamp);
  const ekf::Enu enu{pose.pose.pose.position.x, pose.pose.pose.position.y, 0.0};
  double ecef[3];
  ekf::EnuToEcef(origin, enu, ecef);
  ekf::Llh llh{};
  if (ekf::EcefToGeodetic(ecef, llh) != ekf::Status::Ok) {
    gnss.lat_rad = ekf::sim::kOriginLatRad;
    gnss.lon_rad = ekf::sim::kOriginLonRad;
    gnss.h_m = ekf::sim::kOriginHeightM;
    return gnss;
  }
  gnss.lat_rad = llh.lat_rad;
  gnss.lon_rad = llh.lon_rad;
  gnss.h_m = llh.h_m;
  return gnss;
}

}  // namespace

class EkfGnssDemoNode final : public rclcpp::Node {
 public:
  EkfGnssDemoNode() : Node("ekf_gnss_demo_node") {
    declare_parameter<std::string>("scenario", "matched");

    const auto scenario = get_parameter("scenario").as_string();
    spec_ = ekf::sim::LoadScenarioByName(scenario.c_str());
    cfg_ = spec_.filter;

    const ekf::Llh origin_llh{ekf::sim::kOriginLatRad, ekf::sim::kOriginLonRad,
                              ekf::sim::kOriginHeightM};
    ekf::MakeOrigin(origin_llh, origin_);
    ekf::Reset(filter_, cfg_, 0.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ekf_gnss/odometry", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_gnss/pose", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/sim/imu", rclcpp::SensorDataQoS(),
        std::bind(&EkfGnssDemoNode::on_imu, this, std::placeholders::_1));
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/wheel", 10, std::bind(&EkfGnssDemoNode::on_wheel, this, std::placeholders::_1));
    gnss_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/gnss/pose", 10, std::bind(&EkfGnssDemoNode::on_gnss_pose, this, std::placeholders::_1));
    complete_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/sim/complete", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          RCLCPP_INFO(get_logger(), "ekf_gnss_demo shutting down after /sim/complete");
          rclcpp::shutdown();
        });

    RCLCPP_INFO(get_logger(), "ekf_gnss_demo live filter scenario=%s", scenario.c_str());
  }

 private:
  void apply_wheel(const nav_msgs::msg::Odometry& msg) {
    ekf::WheelSample wheel{};
    wheel.t_s = stamp_to_sec(msg.header.stamp);
    wheel.speed_m_s = msg.twist.twist.linear.x;
    (void)ekf::UpdateWheel(filter_, cfg_, wheel);
  }

  void apply_gnss(const nav_msgs::msg::Odometry& msg) {
    const ekf::GnssSample gnss = gnss_from_pose(msg, origin_);
    (void)ekf::UpdateGnss(filter_, cfg_, origin_, gnss);
  }

  void on_wheel(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (have_last_imu_stamp_ && same_stamp(msg->header.stamp, last_imu_stamp_)) {
      apply_wheel(*msg);
      return;
    }
    pending_wheel_ = *msg;
    have_pending_wheel_ = true;
  }

  void on_gnss_pose(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (have_last_imu_stamp_ && same_stamp(msg->header.stamp, last_imu_stamp_)) {
      apply_gnss(*msg);
      return;
    }
    pending_gnss_ = *msg;
    have_pending_gnss_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ekf::ImuSample imu{};
    imu.t_s = stamp_to_sec(msg->header.stamp);
    imu.gyro_z_rad_s = msg->angular_velocity.z;
    imu.accel_x_m_s2 = msg->linear_acceleration.x;

    double dt = ekf::sim::kDtS;
    if (last_imu_t_s_ >= 0.0) {
      dt = imu.t_s - last_imu_t_s_;
    }
  last_imu_t_s_ = imu.t_s;
    if (dt <= 0.0 || dt > 0.5) {
      dt = ekf::sim::kDtS;
    }

    if (filter_.lifecycle == ekf::Lifecycle::Initialized) {
      imu.t_s = filter_.t_s + dt;
    }
    (void)ekf::Predict(filter_, cfg_, imu, dt);

    const auto& stamp = msg->header.stamp;
    last_imu_stamp_ = stamp;
    have_last_imu_stamp_ = true;

    if (have_pending_wheel_ && same_stamp(pending_wheel_.header.stamp, stamp)) {
      apply_wheel(pending_wheel_);
      have_pending_wheel_ = false;
    }

    if (have_pending_gnss_ && same_stamp(pending_gnss_.header.stamp, stamp)) {
      apply_gnss(pending_gnss_);
      have_pending_gnss_ = false;
    }

    publish_estimate(stamp);
  }

  void publish_estimate(const builtin_interfaces::msg::Time& stamp) {
    if (filter_.lifecycle != ekf::Lifecycle::Initialized) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = filter_.x[0];
    odom.pose.pose.position.y = filter_.x[1];
    const double half = 0.5 * filter_.x[2];
    odom.pose.pose.orientation.z = std::sin(half);
    odom.pose.pose.orientation.w = std::cos(half);
    odom.twist.twist.linear.x = filter_.x[3];
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pose_pub_->publish(pose);
  }

  ekf::sim::ScenarioSpec spec_{};
  ekf::Config cfg_{};
  ekf::Origin origin_{};
  ekf::Filter filter_{};
  double last_imu_t_s_ = -1.0;
  builtin_interfaces::msg::Time last_imu_stamp_{};
  bool have_last_imu_stamp_ = false;
  nav_msgs::msg::Odometry pending_wheel_{};
  nav_msgs::msg::Odometry pending_gnss_{};
  bool have_pending_wheel_ = false;
  bool have_pending_gnss_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr complete_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfGnssDemoNode>());
  rclcpp::shutdown();
  return 0;
}
