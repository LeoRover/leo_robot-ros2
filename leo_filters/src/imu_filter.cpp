// Copyright 2022-2023 Fictionlab sp. z o.o.
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
#include "tf2/LinearMath/Matrix3x3.h"

#include "imu_filter.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace leo_filters
{
ImuFilter::ImuFilter(rclcpp::NodeOptions options)
: Node("imu_filter", options),
  param_listener_(get_node_parameters_interface())
{
  params_ = param_listener_.get_params();
  update_filter_params();
  load_bias();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 5);

  rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 5);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions{{
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  }};

  auto period = std::chrono::duration<double>(params_.bias_save_period * 60.0);

  bias_save_timer_ =
    create_wall_timer(period, std::bind(&ImuFilter::save_bias, this));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", rclcpp::QoS(5).best_effort(),
    std::bind(&ImuFilter::imu_callback, this, _1));

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
    RCLCPP_ERROR(
      get_logger(),
      "IMU bias file doesn't exist or couldn't be opened: %s",
      e.what());
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
      get_logger(), "YAML error while writing bias file: %s",
      e.what());
  }
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

  } catch (YAML::BadFile & e) {
    RCLCPP_ERROR(get_logger(), "IMU bias file doesn't exist.\n");
    RCLCPP_ERROR(get_logger(), "Creating IMU bias file with current gyrometer bias.\n");

    node["gyro_bias_x"] = filter_.getAngularVelocityBiasX();
    node["gyro_bias_y"] = filter_.getAngularVelocityBiasY();
    node["gyro_bias_z"] = filter_.getAngularVelocityBiasZ();

    std::ofstream fout(file_path);
    fout << node;
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

  double gain = filter_.getGainAcc();
  if (std::fabs(gain - params_.gain_acc) > 0.001) {
    if (!filter_.setGainAcc(params_.gain_acc)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid gain_acc passed to ComplementaryFilter.");
    }
  }

  double bias_alpha = filter_.getBiasAlpha();
  if (std::fabs(bias_alpha - params_.bias_alpha) > 0.001) {
    if (!filter_.setBiasAlpha(params_.bias_alpha)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid bias_alpha passed to ComplementaryFilter.");
    }
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
  const rclcpp::Time & time = msg->header.stamp;

  check_dynamic_parameters();

  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  double dt;
  if (params_.constant_dt > 0.0) {
    dt = params_.constant_dt;
  } else {
    dt = (time - time_prev_).nanoseconds() * 1e-9;
  }

  time_prev_ = time;

  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  publish(msg);
}

tf2::Quaternion ImuFilter::hamiltonToTFQuaternion(
  double q0, double q1,
  double q2, double q3) const
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf2::Quaternion(q1, q2, q3, q0);
}

void ImuFilter::publish(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf2::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

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
