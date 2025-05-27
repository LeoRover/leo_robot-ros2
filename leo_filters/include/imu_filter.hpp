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

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "complementary_filter.hpp"

#include "leo_filters/imu_filter_parameters.hpp"

namespace leo_filters
{

class ImuFilter : public rclcpp::Node
{
public:
  explicit ImuFilter(rclcpp::NodeOptions options);

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void reset_calibration_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  std::string get_bias_file_path();
  void load_bias();
  void save_bias();
  void check_dynamic_parameters();
  void update_filter_params();
  void publish(sensor_msgs::msg::Imu::SharedPtr);

  // ROS entities
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_calibration_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr bias_save_timer_;

  // Parameters
  imu_filter::ParamListener param_listener_;
  imu_filter::Params params_;

  // Time
  std::optional<rclcpp::Time> prev_time_{std::nullopt};

  // Filters:
  imu_tools::ComplementaryFilter filter_;
};
}  // namespace leo_filters
