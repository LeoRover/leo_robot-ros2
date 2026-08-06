#pragma once

#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "leo_bringup/heading_controller_parameters.hpp"

namespace leo_bringup
{

class HeadingController : public rclcpp::Node
{
public:
  explicit HeadingController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void check_dynamic_parameters();
  void cmd_vel_matched_callback(rclcpp::MatchedInfo & matched_info);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  heading_controller::ParamListener param_listener_;
  heading_controller::Params params_;

  std::optional<double> current_yaw_{std::nullopt};
  std::optional<double> heading_ref_{std::nullopt};
  std::optional<rclcpp::Time> last_cmd_time_{std::nullopt};
  std::optional<double> prev_raw_angular_z_{std::nullopt};
};

}  // namespace leo_bringup
