#include <algorithm>
#include <cmath>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "leo_bringup/heading_controller.hpp"

namespace leo_bringup
{

HeadingController::HeadingController(const rclcpp::NodeOptions & options)
: Node("heading_controller", options),
  param_listener_(get_node_parameters_interface())
{
  params_ = param_listener_.get_params();

  rclcpp::SubscriptionOptions cmd_sub_options;
  cmd_sub_options.event_callbacks.matched_callback =
    std::bind(&HeadingController::cmd_vel_matched_callback, this, std::placeholders::_1);

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", 1,
    std::bind(&HeadingController::cmd_callback, this, std::placeholders::_1),
    cmd_sub_options);
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}

void HeadingController::check_dynamic_parameters()
{
  if (param_listener_.is_old(params_)) {
    param_listener_.refresh_dynamic_parameters();
    params_ = param_listener_.get_params();
  }
}

void HeadingController::cmd_vel_matched_callback(rclcpp::MatchedInfo & matched_info)
{
  if (matched_info.current_count > 0) {
    if (imu_sub_) {
      return;
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(),
      std::bind(&HeadingController::imu_callback, this, std::placeholders::_1));
    return;
  }

  heading_ref_.reset();
  prev_raw_angular_z_.reset();
  current_yaw_.reset();
  imu_sub_.reset();
}

void HeadingController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto & q_msg = msg->orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

void HeadingController::cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  check_dynamic_parameters();

  const rclcpp::Time now = get_clock()->now();
  double dt = 0.0;
  if (last_cmd_time_) {
    // Time delta in seconds
    dt = (now - *last_cmd_time_).seconds();
  }
  last_cmd_time_ = now;

  const bool heading_hold_request =
    std::abs(msg->angular.z) <= params_.angular_hold_deadband &&
    std::abs(msg->linear.x) >= params_.linear_hold_deadband;

  if (!heading_hold_request) {
    heading_ref_.reset();
    last_cmd_time_.reset();
    prev_raw_angular_z_.reset();
    cmd_pub_->publish(*msg);
    return;
  }

  // No IMU data yet
  if (!current_yaw_) {
    cmd_pub_->publish(*msg);
    return;
  }

  if (!heading_ref_) {
    heading_ref_ = *current_yaw_;
  }

  // Normalized heading error to [-pi, pi]
  const double heading_error = std::atan2(
    std::sin(*current_yaw_ - *heading_ref_),
    std::cos(*current_yaw_ - *heading_ref_));

  double correction = 0.0;
  if (std::abs(heading_error) >= params_.yaw_deadband) {
    correction = std::clamp(
      -params_.kp * heading_error,
      -params_.max_correction,
      params_.max_correction);
  }

  // Update heading reference for next iteration to prevent drift when turning
  if (prev_raw_angular_z_) {
    *heading_ref_ += *prev_raw_angular_z_ * dt;
  }
  prev_raw_angular_z_ = msg->angular.z;

  msg->angular.z += correction;
  cmd_pub_->publish(*msg);
}

}  // namespace leo_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(leo_bringup::HeadingController)
