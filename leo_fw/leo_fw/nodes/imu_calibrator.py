# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from leo_msgs.srv import SetImuCalibration
from leo_msgs.msg import Imu


class ImuCalibrator(Node):
    def __init__(self, time=2.0):
        super().__init__("imu_calibrator")

        self.service_client = self.create_client(
            SetImuCalibration, "set_imu_calibration"
        )

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.data = []
        self.end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=time)

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.running = True

        self.imu_sub = self.create_subscription(
            Imu, "firmware/imu", self.imu_sub_callback, qos
        )

        self.get_logger().info(f"Collecting IMU measurements for {time} seconds...")

    def imu_sub_callback(self, data: Imu):
        if self.get_clock().now() >= self.end_time:
            self.running = False
            self.destroy_subscription(self.imu_sub)

        self.data.append([data.gyro_x, data.gyro_y, data.gyro_z])

    def send_bias(self):
        self.get_logger().info(f"Calculating bias from {len(self.data)} samples.")

        matrix = np.matrix(self.data)
        bias = matrix.mean(0) * -1.0
        bias = bias.tolist()[0]

        self.get_logger().info(f"Calculated bias: {bias}")

        req = SetImuCalibration.Request()
        req.gyro_bias_x, req.gyro_bias_y, req.gyro_bias_z = bias

        future = self.service_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            self.get_logger().error("Failed to set new imu calibration.")
