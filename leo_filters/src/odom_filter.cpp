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

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "odom_filter.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace leo_filters
{
OdomFilter::OdomFilter(rclcpp::NodeOptions options)
: Node("odom_filter", options)
{
  declare_parameter("publish_tf", true);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::QoS(5).best_effort(),
    std::bind(&OdomFilter::imu_callback, this, _1));
  wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "wheel_odom_with_covariance", rclcpp::QoS(5).best_effort(),
    std::bind(&OdomFilter::odom_callback, this, _1));

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  client_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  reset_odom_client_ = create_client<std_srvs::srv::Trigger>(
    "firmware/reset_odometry", rclcpp::ServicesQoS(),
    client_cb_group_);
  reset_odom_srv_ = create_service<std_srvs::srv::Trigger>(
    "reset_odometry",
    std::bind(&OdomFilter::reset_odom_callback, this, _1, _2));

  odom_merged_pub_ =
    create_publisher<nav_msgs::msg::Odometry>("merged_odom", 10);

  odom_merged_timer_ = create_wall_timer(
    10ms, std::bind(&OdomFilter::odom_merged_callback, this));

  RCLCPP_INFO(get_logger(), "Started node");
}

void OdomFilter::odom_merged_callback()
{
  if (!imu_received_ || !odom_received_) {
    return;
  }

  rclcpp::Time current_time = get_clock()->now();

  if (!last_call_time_.has_value()) {
    last_call_time_ = current_time;
    return;
  }

  odom_merged_msg_.header.stamp = current_time;

  double vel_x = odom_merged_msg_.twist.twist.linear.x;
  double vel_y = odom_merged_msg_.twist.twist.linear.y;

  const double move_x =
    vel_x * std::cos(odom_merged_yaw_) - vel_y * std::sin(odom_merged_yaw_);
  const double move_y =
    vel_x * std::sin(odom_merged_yaw_) + vel_y * std::cos(odom_merged_yaw_);

  double dt = (current_time - *last_call_time_).seconds();
  odom_merged_msg_.pose.pose.position.x += move_x * dt;
  odom_merged_msg_.pose.pose.position.y += move_y * dt;

  odom_merged_yaw_ += odom_merged_msg_.twist.twist.angular.z * dt;

  if (odom_merged_yaw_ > 2.0 * PI) {
    odom_merged_yaw_ -= 2.0 * PI;
  } else if (odom_merged_yaw_ < 0.0) {
    odom_merged_yaw_ += 2.0 * PI;
  }
  odom_merged_msg_.pose.pose.orientation.z = std::sin(odom_merged_yaw_ * 0.5);
  odom_merged_msg_.pose.pose.orientation.w = std::cos(odom_merged_yaw_ * 0.5);

  odom_merged_pub_->publish(odom_merged_msg_);

  if (get_parameter("publish_tf").as_bool()) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_merged_msg_.header;
    tf_msg.child_frame_id = odom_merged_msg_.child_frame_id;

    tf_msg.transform.translation.x = odom_merged_msg_.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_merged_msg_.pose.pose.position.y;
    tf_msg.transform.rotation = odom_merged_msg_.pose.pose.orientation;

    broadcaster_->sendTransform(tf_msg);
  }
  last_call_time_ = current_time;
}

void OdomFilter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  odom_merged_msg_.twist.twist.angular.z = msg->angular_velocity.z;
  odom_merged_msg_.twist.covariance[35] = msg->angular_velocity_covariance[8];

  imu_received_ = true;
}

void OdomFilter::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_merged_msg_.header.frame_id = msg->header.frame_id;
  odom_merged_msg_.child_frame_id = msg->child_frame_id;
  odom_merged_msg_.twist.twist.linear.x = msg->twist.twist.linear.x;
  odom_merged_msg_.twist.twist.linear.y = msg->twist.twist.linear.y;

  for (int i = 0; i < 5; i++) {
    odom_merged_msg_.twist.covariance[i * 7] = msg->twist.covariance[i * 7];
  }

  odom_received_ = true;
}

void OdomFilter::reset_odom_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  constexpr std::chrono::seconds callback_timeout = std::chrono::seconds(3);
  odom_merged_msg_.pose.pose.position.x = 0.0;
  odom_merged_msg_.pose.pose.position.y = 0.0;
  odom_merged_yaw_ = 0.0;

  auto reset_request = std::make_shared<std_srvs::srv::Trigger_Request>();
  auto future = reset_odom_client_->async_send_request(reset_request);
  auto result_status = future.wait_for(callback_timeout);

  if (result_status == std::future_status::ready) {
    if (future.get()->success) {
      res->success = true;
      res->message = "Odometry reset successful.";
    } else {
      res->success = false;
      res->message = "Failed to reset odometry.";
    }
  } else if (result_status == std::future_status::timeout) {
    res->success = false;
    res->message = "Firmware service timeout.";
  } else {
    res->success = false;
    res->message = "Firmware service call deffered.";
  }
}
} // namespace leo_filters

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leo_filters::OdomFilter)
