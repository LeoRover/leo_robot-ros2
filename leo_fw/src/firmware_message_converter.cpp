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

#include <cmath>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "leo_msgs/msg/imu.hpp"
#include "leo_msgs/msg/wheel_odom.hpp"
#include "leo_msgs/msg/wheel_odom_mecanum.hpp"
#include "leo_msgs/msg/wheel_states.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace leo_fw
{

class FirmwareMessageConverter : public rclcpp::Node
{
public:
  FirmwareMessageConverter(rclcpp::NodeOptions options)
  : Node("firmware_message_converter", options.use_intra_process_comms(true))
  {
    robot_frame_id_ = declare_parameter("robot_frame_id", robot_frame_id_);
    odom_frame_id_ = declare_parameter("odom_frame_id", odom_frame_id_);
    imu_frame_id_ = declare_parameter("imu_frame_id", imu_frame_id_);
    wheel_joint_names_ = declare_parameter("wheel_joint_names", wheel_joint_names_);
    wheel_odom_twist_covariance_diagonal_ = declare_parameter(
      "wheel_odom_twist_covariance_diagonal", wheel_odom_twist_covariance_diagonal_);
    wheel_odom_mecanum_twist_covariance_diagonal_ = declare_parameter(
      "wheel_odom_mecanum_twist_covariance_diagonal",
      wheel_odom_mecanum_twist_covariance_diagonal_);
    imu_angular_velocity_covariance_diagonal_ = declare_parameter(
      "imu_angular_velocity_covariance_diagonal", imu_angular_velocity_covariance_diagonal_);
    imu_linear_acceleration_covariance_diagonal_ = declare_parameter(
      "imu_linear_acceleration_covariance_diagonal", imu_linear_acceleration_covariance_diagonal_);
    tf_frame_prefix_ = declare_parameter("tf_frame_prefix", tf_frame_prefix_);

    auto node_topics = get_node_topics_interface();
    wheel_states_topic_ = node_topics->resolve_topic_name("firmware/wheel_states");
    wheel_odom_topic_ = node_topics->resolve_topic_name("firmware/wheel_odom");
    wheel_odom_mecanum_topic_ = node_topics->resolve_topic_name("firmware/wheel_odom_mecanum");
    imu_topic_ = node_topics->resolve_topic_name("firmware/imu");

    timer_ = create_wall_timer(500ms, std::bind(&FirmwareMessageConverter::timer_callback, this));
  }

private:
  void wheel_states_callback(const leo_msgs::msg::WheelStates::SharedPtr msg) const
  {
    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = msg->stamp;
    joint_states.name = wheel_joint_names_;
    joint_states.position = std::vector<double>(msg->position.begin(), msg->position.end());
    joint_states.velocity = std::vector<double>(msg->velocity.begin(), msg->velocity.end());
    joint_states.effort = std::vector<double>(msg->torque.begin(), msg->torque.end());
    joint_states_pub_->publish(joint_states);
  }

  void wheel_odom_callback(const leo_msgs::msg::WheelOdom::SharedPtr msg) const
  {
    nav_msgs::msg::Odometry wheel_odom;
    wheel_odom.header.frame_id = tf_frame_prefix_ + odom_frame_id_;
    wheel_odom.child_frame_id = tf_frame_prefix_ + robot_frame_id_;
    wheel_odom.header.stamp = msg->stamp;
    wheel_odom.twist.twist.linear.x = msg->velocity_lin;
    wheel_odom.twist.twist.angular.z = msg->velocity_ang;
    wheel_odom.pose.pose.position.x = msg->pose_x;
    wheel_odom.pose.pose.position.y = msg->pose_y;
    wheel_odom.pose.pose.orientation.z = std::sin(msg->pose_yaw * 0.5F);
    wheel_odom.pose.pose.orientation.w = std::cos(msg->pose_yaw * 0.5F);

    for (int i = 0; i < 6; i++) {
      wheel_odom.twist.covariance[i * 7] =
        wheel_odom_twist_covariance_diagonal_[i];
    }

    wheel_odom_pub_->publish(wheel_odom);
  }

  void mecanum_odom_callback(const leo_msgs::msg::WheelOdomMecanum::SharedPtr msg) const
  {
    nav_msgs::msg::Odometry wheel_odom;
    wheel_odom.header.frame_id = tf_frame_prefix_ + odom_frame_id_;
    wheel_odom.child_frame_id = tf_frame_prefix_ + robot_frame_id_;
    wheel_odom.header.stamp = msg->stamp;
    wheel_odom.twist.twist.linear.x = msg->velocity_lin_x;
    wheel_odom.twist.twist.linear.y = msg->velocity_lin_y;
    wheel_odom.twist.twist.angular.z = msg->velocity_ang;
    wheel_odom.pose.pose.position.x = msg->pose_x;
    wheel_odom.pose.pose.position.y = msg->pose_y;
    wheel_odom.pose.pose.orientation.z = std::sin(msg->pose_yaw * 0.5F);
    wheel_odom.pose.pose.orientation.w = std::cos(msg->pose_yaw * 0.5F);

    for (int i = 0; i < 6; i++) {
      wheel_odom.twist.covariance[i * 7] =
        wheel_odom_mecanum_twist_covariance_diagonal_[i];
    }

    wheel_odom_mecanum_pub_->publish(wheel_odom);
  }

  void imu_callback(const leo_msgs::msg::Imu::SharedPtr msg) const
  {
    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = tf_frame_prefix_ + imu_frame_id_;
    imu.header.stamp = msg->stamp;
    imu.angular_velocity.x = msg->gyro_x;
    imu.angular_velocity.y = msg->gyro_y;
    imu.angular_velocity.z = msg->gyro_z;
    imu.linear_acceleration.x = msg->accel_x;
    imu.linear_acceleration.y = msg->accel_y;
    imu.linear_acceleration.z = msg->accel_z;

    for (int i = 0; i < 3; i++) {
      imu.angular_velocity_covariance[i * 4] =
        imu_angular_velocity_covariance_diagonal_[i];
      imu.linear_acceleration_covariance[i * 4] =
        imu_linear_acceleration_covariance_diagonal_[i];
    }

    imu_pub_->publish(imu);
  }

  void timer_callback()
  {
    size_t wheel_states_publishers = count_publishers(wheel_states_topic_);

    if (joint_states_pub_ && wheel_states_publishers == 0) {
      RCLCPP_INFO(
        get_logger(), "firmware/wheel_states topic no longer has any publishers. "
        "Shutting down joint_states publisher.");
      wheel_states_sub_.reset();
      joint_states_pub_.reset();
    }

    if (!joint_states_pub_ && wheel_states_publishers > 0) {
      RCLCPP_INFO(
        get_logger(), "Detected a publisher on firmware/wheel_states topic. "
        "Starting publishing on joint_states topic.");
      joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      wheel_states_sub_ = create_subscription<leo_msgs::msg::WheelStates>(
        wheel_states_topic_, rclcpp::QoS(5).best_effort(),
        std::bind(&FirmwareMessageConverter::wheel_states_callback, this, _1));
    }

    size_t wheel_odom_publishers = count_publishers(wheel_odom_topic_);

    if (wheel_odom_pub_ && wheel_odom_publishers == 0) {
      RCLCPP_INFO(
        get_logger(), "firmware/wheel_odom topic no longer has any publishers. "
        "Shutting down wheel_odom publisher.");
      wheel_odom_sub_.reset();
      wheel_odom_pub_.reset();
    }

    if (!wheel_odom_pub_ && wheel_odom_publishers > 0) {
      RCLCPP_INFO(
        get_logger(), "Detected a publisher on firmware/wheel_odom topic. "
        "Starting publishing on wheel_odom topic.");
      wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
      wheel_odom_sub_ = create_subscription<leo_msgs::msg::WheelOdom>(
        wheel_odom_topic_, rclcpp::QoS(5).best_effort(),
        std::bind(&FirmwareMessageConverter::wheel_odom_callback, this, _1));
    }

    size_t wheel_odom_mecanum_publishers = count_publishers(wheel_odom_mecanum_topic_);

    if (wheel_odom_mecanum_pub_ && wheel_odom_mecanum_publishers == 0) {
      RCLCPP_INFO(
        get_logger(),
        "firmware/wheel_odom_mecanum topic no longer has any publishers. "
        "Shutting down wheel_odom publisher.");
      wheel_odom_mecanum_sub_.reset();
      wheel_odom_mecanum_pub_.reset();
    }

    if (!wheel_odom_mecanum_pub_ && wheel_odom_mecanum_publishers > 0) {
      RCLCPP_INFO(
        get_logger(),
        "Detected a publisher on firmware/wheel_odom_mecanum topic. "
        "Starting publishing on wheel_odom topic.");
      wheel_odom_mecanum_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
      wheel_odom_mecanum_sub_ =
        create_subscription<leo_msgs::msg::WheelOdomMecanum>(
        wheel_odom_mecanum_topic_, rclcpp::QoS(5).best_effort(),
        std::bind(
          &FirmwareMessageConverter::mecanum_odom_callback, this,
          _1));
    }

    size_t imu_publishers = count_publishers(imu_topic_);

    if (imu_pub_ && imu_publishers == 0) {
      RCLCPP_INFO(
        get_logger(), "firmware/imu topic no longer has any publishers. "
        "Shutting down imu/data_raw publisher.");
      imu_sub_.reset();
      imu_pub_.reset();
    }

    if (!imu_pub_ && imu_publishers > 0) {
      RCLCPP_INFO(
        get_logger(), "Detected a publisher on firmware/imu topic. "
        "Starting publishing on imu/data_raw topic.");
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
      imu_sub_ = create_subscription<leo_msgs::msg::Imu>(
        imu_topic_, rclcpp::QoS(5).best_effort(),
        std::bind(&FirmwareMessageConverter::imu_callback, this, _1));
    }
  }

  // Parameters
  std::string robot_frame_id_ = "base_footprint";
  std::string odom_frame_id_ = "odom";
  std::string imu_frame_id_ = "imu_frame";
  std::vector<std::string> wheel_joint_names_ = {
    "wheel_FL_joint", "wheel_RL_joint", "wheel_FR_joint", "wheel_RR_joint"};
  std::vector<double> wheel_odom_twist_covariance_diagonal_ = {0.0001, 0.0, 0.0,
    0.0, 0.0, 0.001};
  std::vector<double> wheel_odom_mecanum_twist_covariance_diagonal_ = {
    0.0001, 0.0001, 0.0, 0.0, 0.0, 0.001};
  std::vector<double> imu_angular_velocity_covariance_diagonal_ = {
    0.000001, 0.000001, 0.00001};
  std::vector<double> imu_linear_acceleration_covariance_diagonal_ = {0.001, 0.001,
    0.001};
  std::string tf_frame_prefix_ = "";

  // Topic names
  std::string wheel_states_topic_;
  std::string wheel_odom_topic_;
  std::string wheel_odom_mecanum_topic_;
  std::string imu_topic_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_mecanum_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Subscriptions
  rclcpp::Subscription<leo_msgs::msg::WheelStates>::SharedPtr wheel_states_sub_;
  rclcpp::Subscription<leo_msgs::msg::WheelOdom>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<leo_msgs::msg::WheelOdomMecanum>::SharedPtr wheel_odom_mecanum_sub_;
  rclcpp::Subscription<leo_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace leo_fw

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leo_fw::FirmwareMessageConverter)
