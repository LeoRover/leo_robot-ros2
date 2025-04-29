// Copyright 2025 Fictionlab sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <optional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace leo_filters
{

constexpr double PI = 3.141592653;
class OdomFilter : public rclcpp::Node
{
public:
  explicit OdomFilter(rclcpp::NodeOptions options);

private:
  void odom_merged_callback();
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void reset_odom_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  // ROS entities
  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_merged_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_srv_;

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_odom_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr odom_merged_timer_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // Messages
  nav_msgs::msg::Odometry odom_merged_msg_;

  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // Time
  std::optional<rclcpp::Time> last_call_time_ {std::nullopt};

  // Flags and variables
  double odom_merged_yaw_{};
  std::string tf_frame_prefix_{};
  bool imu_received_{};
  bool odom_received_{};
};
} // namespace leo_filters
