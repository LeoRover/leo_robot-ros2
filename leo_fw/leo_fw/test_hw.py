# Copyright 2022-2023 Fictionlab sp. z o.o.
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

import os
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from leo_msgs.msg import Imu
from std_msgs.msg import Float32

from .board import BoardType, check_firmware_node
from .utils import write_flush, spin_for, parse_yaml, print_ok, print_test_result

# Time given to the ROS graph to be discovered before it gets inspected
NODE_DISCOVERY_TIME = 3.0

# Time given to the subscriptions to match with the firmware publishers
TOPIC_DISCOVERY_TIME = 2.0


class TestMode(Enum):
    IMU = "imu"
    BATTERY = "battery"
    ALL = "all"

    def __str__(self):
        return self.value


class HardwareTester:
    """Validates the sensors that do not require the robot to move."""

    def __init__(self, node: Node):
        self.path = os.path.join(
            get_package_share_directory("leo_fw"), "data", "hw_tests"
        )

        self.node = node

        self.is_new_imu_data = False
        self.is_new_battery_data = False

        self.imu_data = Imu()
        self.battery_data = Float32()

        ### Subscriptions

        self.battery_sub = node.create_subscription(
            Float32, "firmware/battery", self.battery_callback, qos_profile_sensor_data
        )
        self.imu_sub = node.create_subscription(
            Imu, "firmware/imu", self.imu_callback, qos_profile_sensor_data
        )

        spin_for(self.node, TOPIC_DISCOVERY_TIME)

    def battery_callback(self, data: Float32) -> None:
        self.battery_data = data
        self.is_new_battery_data = True

    def imu_callback(self, data: Imu) -> None:
        self.imu_data = data
        self.is_new_imu_data = True

    def test_imu(self) -> tuple[bool, Optional[str]]:
        msg_cnt = 0
        time_last_msg = time.monotonic()
        imu_valid = parse_yaml(os.path.join(self.path, "imu.yaml"))

        accel_del = imu_valid["imu"]["accel_del"]
        accel_x = imu_valid["imu"]["accel_x"]
        accel_y = imu_valid["imu"]["accel_y"]
        accel_z = imu_valid["imu"]["accel_z"]

        gyro_del = imu_valid["imu"]["gyro_del"]
        gyro_x = imu_valid["imu"]["gyro_x"]
        gyro_y = imu_valid["imu"]["gyro_y"]
        gyro_z = imu_valid["imu"]["gyro_z"]

        while msg_cnt < 50:
            rclpy.spin_once(self.node, timeout_sec=imu_valid["imu"]["timeout"])

            time_now = time.monotonic()
            if time_last_msg + imu_valid["imu"]["timeout"] < time_now:
                return False, "TIMEOUT"

            if self.is_new_imu_data:
                time_last_msg = time_now
                self.is_new_imu_data = False
                msg_cnt += 1

                if not (
                    accel_x - accel_del < self.imu_data.accel_x < accel_x + accel_del
                    and accel_y - accel_del
                    < self.imu_data.accel_y
                    < accel_y + accel_del
                    and accel_z - accel_del
                    < abs(self.imu_data.accel_z)
                    < accel_z + accel_del
                    and gyro_x - gyro_del < self.imu_data.gyro_x < gyro_x + gyro_del
                    and gyro_y - gyro_del < self.imu_data.gyro_y < gyro_y + gyro_del
                    and gyro_z - gyro_del < self.imu_data.gyro_z < gyro_z + gyro_del
                ):
                    return False, "INVALID DATA"

        return True, None

    def test_battery(self) -> tuple[bool, Optional[str]]:
        msg_cnt = 0
        time_last_msg = time.monotonic()
        batt_valid = parse_yaml(os.path.join(self.path, "battery.yaml"))

        while msg_cnt < 50:
            rclpy.spin_once(self.node, timeout_sec=batt_valid["battery"]["timeout"])

            time_now = time.monotonic()
            if time_last_msg + batt_valid["battery"]["timeout"] < time_now:
                return False, "TIMEOUT"

            if self.is_new_battery_data:
                time_last_msg = time_now
                self.is_new_battery_data = False
                msg_cnt += 1

                if self.battery_data.data <= batt_valid["battery"]["voltage_min"]:
                    return False, "LOW VOLTAGE"
                if self.battery_data.data >= batt_valid["battery"]["voltage_max"]:
                    return False, "HIGH VOLTAGE"

        return True, None


def test_hw(
    hardware: TestMode = TestMode.ALL,
) -> None:
    write_flush("--> Initializing ROS node.. ")
    rclpy.init(args=None)
    node = Node("leo_hardware_tester")
    spin_for(node, NODE_DISCOVERY_TIME)
    print_ok("DONE")

    try:
        board_type = check_firmware_node(node)

        if board_type is None:
            return

        #####################################################

        write_flush("--> Initializing Hardware Tester.. ")
        tester = HardwareTester(node)
        print_ok("DONE")

        #####################################################

        if hardware in (TestMode.ALL, TestMode.BATTERY):
            write_flush("--> Battery validation.. ")
            print_test_result(tester.test_battery())

        if hardware in (TestMode.ALL, TestMode.IMU) and board_type == BoardType.LEOCORE:
            write_flush("--> IMU validation.. ")
            print_test_result(tester.test_imu())

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
