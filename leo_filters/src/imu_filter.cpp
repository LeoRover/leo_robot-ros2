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
#include <fstream>

#include "yaml-cpp/yaml.h"

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

#include "imu_filter.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace leo_filters
{
inline tf2::Quaternion hamiltonToTFQuaternion(
  double q0, double q1,
  double q2, double q3)
{
    // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
    // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf2::Quaternion(q1, q2, q3, q0);
}


ImuFilter::ImuFilter(rclcpp::NodeOptions options)
: Node("imu_filter", options),
  param_listener_(get_node_parameters_interface())
{
  params_ = param_listener_.get_params();
  update_filter_params();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 5);

  rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 5);

  reset_calibration_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_calibration", std::bind(
    &ImuFilter::reset_calibration_callback, this, _1, _2));

  if (params_.do_save_bias) {
    load_bias();
    bias_save_timer_ =
      create_wall_timer(std::chrono::seconds(params_.bias_save_period),
        std::bind(&ImuFilter::save_bias, this));
  }

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions{{
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  }};

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", 10,
    std::bind(&ImuFilter::imu_callback, this, _1), sub_opts);

  RCLCPP_INFO(get_logger(), "Started node");
}

void ImuFilter::save_bias()
{
  YAML::Node node;
  std::string file_path = get_bias_file_path();

  try {
    node = YAML::LoadFile(file_path);
    node["gyro_bias_x"] = filter_.getAngularVelocityBiasX();
    node["gyro_bias_y"] = filter_.getAngularVelocityBiasY();
    node["gyro_bias_z"] = filter_.getAngularVelocityBiasZ();
  } catch (const YAML::BadFile & e) {
    RCLCPP_WARN(
      get_logger(),
      "IMU bias file doesn't exist or couldn't be opened: %s",
      e.what());
    RCLCPP_INFO_STREAM(get_logger(),
        "Creating '" << file_path << "' file with current gyrometer bias.");

    node["gyro_bias_x"] = filter_.getAngularVelocityBiasX();
    node["gyro_bias_y"] = filter_.getAngularVelocityBiasY();
    node["gyro_bias_z"] = filter_.getAngularVelocityBiasZ();
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "YAML error while loading bias file: %s",
      e.what());
    return;
  }

  try {
    std::ofstream fout(file_path);
    fout << node;

    if (!fout.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "Failed to open bias file for writing: %s",
        file_path.c_str());
      return;
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "YAML error while saving bias file: %s",
      e.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "Gyrometer bias save successful.");
}

void ImuFilter::load_bias()
{
  YAML::Node node;
  std::string file_path = get_bias_file_path();

  try {
    node = YAML::LoadFile(file_path);

    if (node["gyro_bias_x"]) {
      filter_.setAngularVelocityBiasX(node["gyro_bias_x"].as<double>());
    }

    if (node["gyro_bias_y"]) {
      filter_.setAngularVelocityBiasY(node["gyro_bias_y"].as<double>());
    }

    if (node["gyro_bias_z"]) {
      filter_.setAngularVelocityBiasZ(node["gyro_bias_z"].as<double>());
    }
    RCLCPP_INFO(get_logger(), "Gyrometer bias load successful.");

  } catch (YAML::BadFile & e) {
    RCLCPP_WARN(
      get_logger(),
      "IMU bias file doesn't exist or couldn't be opened: %s",
      e.what());
    RCLCPP_WARN(get_logger(), "Gyrometer bias load failed.");
  }
}

std::string ImuFilter::get_bias_file_path()
{
  std::string ros_home;
  char * ros_home_env;
  if (ros_home_env = std::getenv("ROS_HOME")) {
    ros_home = ros_home_env;
  } else if (ros_home_env = std::getenv("HOME")) {
    ros_home = ros_home_env;
    ros_home += "/.ros";
  }

  return ros_home + "/imu_calibration.yaml";
}

void ImuFilter::update_filter_params()
{
  if (filter_.getDoBiasEstimation() != params_.do_bias_estimation) {
    filter_.setDoBiasEstimation(params_.do_bias_estimation);
  }

  if (filter_.getDoAdaptiveGain() != params_.do_adaptive_gain) {
    filter_.setDoAdaptiveGain(params_.do_adaptive_gain);
  }

  if (filter_.getGainAcc() != params_.gain_acc) {
    if (!filter_.setGainAcc(params_.gain_acc)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid gain_acc passed to ComplementaryFilter.");
    }
  }

  if (filter_.getBiasAlpha() != params_.bias_alpha) {
    if (!filter_.setBiasAlpha(params_.bias_alpha)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid bias_alpha passed to ComplementaryFilter.");
    }
  }

  if (filter_.getSteadyStateAngularVelocityThreshold() !=
    params_.steady_state_angular_velocity_threshold)
  {
    filter_.setSteadyStateAngularVelocityThreshold(params_.steady_state_angular_velocity_threshold);
  }

  if (filter_.getSteadyStateAccelerationThreshold() !=
    params_.steady_state_acceleration_threshold)
  {
    filter_.setSteadyStateAccelerationThreshold(params_.steady_state_acceleration_threshold);
  }

  if (filter_.getSteadyStateDeltaAngularVelocityThreshold() !=
    params_.steady_state_delta_angular_velocity_threshold)
  {
    filter_.setSteadyStateDeltaAngularVelocityThreshold(
        params_.steady_state_delta_angular_velocity_threshold);
  }

  if (filter_.getSteadyStateRequiredSteadyTime() != params_.steady_state_required_steady_time) {
    filter_.setSteadyStateRequiredSteadyTime(params_.steady_state_required_steady_time);
  }
}

void ImuFilter::check_dynamic_parameters()
{
  if (param_listener_.is_old(params_)) {
    param_listener_.refresh_dynamic_parameters();
    params_ = param_listener_.get_params();
    update_filter_params();
  }
}

void ImuFilter::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
  const geometry_msgs::msg::Vector3 & a = msg->linear_acceleration;
  const geometry_msgs::msg::Vector3 & w = msg->angular_velocity;
  const rclcpp::Time & current_time = msg->header.stamp;

  check_dynamic_parameters();

  if (!prev_time_.has_value()) {
    prev_time_ = current_time;
    return;
  }

  double dt = (current_time - *prev_time_).seconds();

  prev_time_ = current_time;

  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  publish(msg);
}

void ImuFilter::reset_calibration_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  filter_.setAngularVelocityBiasX(0.0);
  filter_.setAngularVelocityBiasY(0.0);
  filter_.setAngularVelocityBiasZ(0.0);

  RCLCPP_INFO(get_logger(), "IMU calibration reset.");

  if (params_.do_save_bias) {
    save_bias();
    bias_save_timer_->reset();
  }

  res->success = true;
  res->message = "IMU calibration reset successfully.";
}

void ImuFilter::publish(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf2::Quaternion q = leo_filters::hamiltonToTFQuaternion(q0, q1, q2, q3);

  imu_msg->orientation.x = q1;
  imu_msg->orientation.y = q2;
  imu_msg->orientation.z = q3;
  imu_msg->orientation.w = q0;

  imu_msg->orientation_covariance[0] = params_.orientation_variance;
  imu_msg->orientation_covariance[4] = params_.orientation_variance;
  imu_msg->orientation_covariance[8] = params_.orientation_variance;

  // Account for biases.
  if (params_.do_bias_estimation) {
    imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
    imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
    imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
  }

  imu_pub_->publish(*imu_msg);

  geometry_msgs::msg::Vector3Stamped rpy;
  rpy.header = imu_msg->header;

  tf2::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
  rpy_pub_->publish(rpy);
}

} // namespace leo_filters

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leo_filters::ImuFilter)
