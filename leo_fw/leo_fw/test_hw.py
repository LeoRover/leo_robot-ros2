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
from argparse import Namespace
from typing import Optional

import rclpy
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from ros2cli.node.direct import DirectNode
from ament_index_python.packages import get_package_share_directory

from leo_msgs.msg import Imu, WheelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from .utils import write_flush
from .board import BoardType, determine_board, check_firmware_version
from .utils import parse_yaml, print_ok, print_warn, print_fail, print_test_result


class TestMode(Enum):
    ENCODER = "encoder"
    TORQUE = "torque"
    IMU = "imu"
    BATTERY = "battery"
    ALL = "all"

    def __str__(self):
        return self.value


class HardwareTester:
    WHEEL_NAMES = ["FL", "RL", "FR", "RR"]

    is_new_imu_data = False
    is_new_wheel_data = False
    is_new_battery_data = False

    imu_data = Imu()
    wheel_data = WheelStates()
    battery_data = Float32()

    def __init__(self, node: rclpy.node.Node):
        self.path = os.path.join(
            get_package_share_directory("leo_fw"), "data", "hw_tests"
        )

        self.node = node

        ### Publishers

        self.cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 1)
        self.cmd_pwmfl_pub = node.create_publisher(
            Float32, "firmware/wheel_FL/cmd_pwm_duty", 1
        )
        self.cmd_pwmrl_pub = node.create_publisher(
            Float32, "firmware/wheel_RL/cmd_pwm_duty", 1
        )
        self.cmd_pwmfr_pub = node.create_publisher(
            Float32, "firmware/wheel_FR/cmd_pwm_duty", 1
        )
        self.cmd_pwmrr_pub = node.create_publisher(
            Float32, "firmware/wheel_RR/cmd_pwm_duty", 1
        )
        self.cmd_velfl_pub = node.create_publisher(
            Float32, "firmware/wheel_FL/cmd_velocity", 1
        )
        self.cmd_velrl_pub = node.create_publisher(
            Float32, "firmware/wheel_RL/cmd_velocity", 1
        )
        self.cmd_velfr_pub = node.create_publisher(
            Float32, "firmware/wheel_FR/cmd_velocity", 1
        )
        self.cmd_velrr_pub = node.create_publisher(
            Float32, "firmware/wheel_RR/cmd_velocity", 1
        )

        ### Subscriptions

        self.battery_sub = node.create_subscription(
            Float32, "firmware/battery", self.battery_callback, qos_profile_sensor_data
        )
        self.wheel_sub = node.create_subscription(
            WheelStates,
            "firmware/wheel_states",
            self.wheel_callback,
            qos_profile_sensor_data,
        )
        self.imu_sub = node.create_subscription(
            Imu, "firmware/imu", self.imu_callback, qos_profile_sensor_data
        )

        # Spin for 2 secods
        rclpy.spin_until_future_complete(self.node, Future(), None, 2.0)

    def battery_callback(self, data: Float32) -> None:
        self.battery_data = data
        self.is_new_battery_data = True

    def imu_callback(self, data: Imu) -> None:
        self.imu_data = data
        self.is_new_imu_data = True

    def wheel_callback(self, data: WheelStates) -> None:
        self.wheel_data = data
        self.is_new_wheel_data = True

    def check_motor_load(self) -> bool:
        speed_limit = 1.0
        motors_loaded = True

        for pwm in range(30):
            pwm_value = float(pwm)

            self.cmd_pwmfl_pub.publish(Float32(data=pwm_value))
            self.cmd_pwmfr_pub.publish(Float32(data=pwm_value))
            self.cmd_pwmrl_pub.publish(Float32(data=-pwm_value))
            self.cmd_pwmrr_pub.publish(Float32(data=-pwm_value))

            rclpy.spin_until_future_complete(self.node, Future(), None, 0.2)

            if (
                self.wheel_data.velocity[0] > speed_limit
                and self.wheel_data.velocity[1] < -speed_limit
                and self.wheel_data.velocity[2] > speed_limit
                and self.wheel_data.velocity[3] < -speed_limit
            ):
                motors_loaded = False
                break

        self.cmd_pwmfl_pub.publish(Float32(data=0.0))
        self.cmd_pwmfr_pub.publish(Float32(data=0.0))
        self.cmd_pwmrl_pub.publish(Float32(data=0.0))
        self.cmd_pwmrr_pub.publish(Float32(data=0.0))

        return motors_loaded

    def test_encoder(self, motors_loaded=True) -> tuple[bool, Optional[str]]:
        is_error = [False] * 4

        if motors_loaded:
            wheel_valid = parse_yaml(os.path.join(self.path, "encoder_load.yaml"))
        else:
            wheel_valid = parse_yaml(os.path.join(self.path, "encoder.yaml"))

        for wheel_test in wheel_valid["tests"]:
            self.cmd_velfl_pub.publish(Float32(data=wheel_test["velocity"]))
            self.cmd_velfr_pub.publish(Float32(data=wheel_test["velocity"]))
            self.cmd_velrl_pub.publish(Float32(data=wheel_test["velocity"]))
            self.cmd_velrr_pub.publish(Float32(data=wheel_test["velocity"]))

            rclpy.spin_until_future_complete(
                self.node, Future(), None, wheel_test["time"]
            )

            speed_min = wheel_test["velocity"] - wheel_test["tolerance"]
            speed_max = wheel_test["velocity"] + wheel_test["tolerance"]

            for i in range(0, 4):
                if not speed_min < self.wheel_data.velocity[i] < speed_max:
                    is_error[i] = True

        self.cmd_velfl_pub.publish(Float32(data=0.0))
        self.cmd_velfr_pub.publish(Float32(data=0.0))
        self.cmd_velrl_pub.publish(Float32(data=0.0))
        self.cmd_velrr_pub.publish(Float32(data=0.0))

        if any(is_error):
            return False, ", ".join(
                [name for name, error in zip(self.WHEEL_NAMES, is_error) if error]
            )

        return True, None

    def test_torque(self, motors_loaded=True) -> tuple[bool, Optional[str]]:
        is_error = [False] * 4

        if motors_loaded:
            torque_valid = parse_yaml(os.path.join(self.path, "torque_load.yaml"))
        else:
            torque_valid = parse_yaml(os.path.join(self.path, "torque.yaml"))

        for torque_test in torque_valid["tests"]:
            self.cmd_pwmfl_pub.publish(Float32(data=torque_test["pwm"]))
            self.cmd_pwmfr_pub.publish(Float32(data=torque_test["pwm"]))
            self.cmd_pwmrl_pub.publish(Float32(data=-torque_test["pwm"]))
            self.cmd_pwmrr_pub.publish(Float32(data=-torque_test["pwm"]))

            rclpy.spin_until_future_complete(
                self.node, Future(), None, torque_test["time"]
            )

            for i in range(4):
                if (
                    not torque_test["torque_min"]
                    <= self.wheel_data.torque[i]
                    <= torque_test["torque_max"]
                ):
                    is_error[i] = True

        self.cmd_pwmfl_pub.publish(Float32(data=0.0))
        self.cmd_pwmfr_pub.publish(Float32(data=0.0))
        self.cmd_pwmrl_pub.publish(Float32(data=0.0))
        self.cmd_pwmrr_pub.publish(Float32(data=0.0))

        if any(is_error):
            return False, ", ".join(
                [name for name, error in zip(self.WHEEL_NAMES, is_error) if error]
            )

        return True, None

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
            rclpy.spin_once(self.node)

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
            rclpy.spin_once(self.node)

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


# pylint: disable=too-many-branches,too-many-statements
def test_hw(
    hardware: TestMode = TestMode.ALL,
) -> None:
    write_flush("--> Initializing ROS node.. ")
    node_wrapper = DirectNode(
        Namespace(node_name_suffix="firmware_tester", spin_time=3.0)
    )
    node = node_wrapper.node
    print_ok("DONE")

    write_flush("--> Initializing Hardware Tester.. ")
    tester = HardwareTester(node)
    print_ok("DONE")

    try:
        write_flush("--> Checking if firmware node is active.. ")

        if ("firmware", node.get_namespace()) in node.get_node_names_and_namespaces():
            print_ok("YES")
        else:
            print_fail("NO")
            print_warn(
                "Firmware node is not active. "
                "Will not be able to validate hardware. "
                "Try to flash the firmware or restart the Micro-ROS Agent."
            )
            return

        #####################################################

        write_flush("--> Trying to determine board type.. ")

        board_type = determine_board(node)

        if board_type is not None:
            print_ok("SUCCESS")
        else:
            print_fail("FAIL")
            print_warn(
                "Can not determine board type. "
                "Update the firmware and try to rerun the script."
            )
            return

        #####################################################

        write_flush("--> Trying to check the current firmware version.. ")

        current_firmware_version = "<unknown>"
        current_firmware_version = check_firmware_version(node)

        if current_firmware_version != "<unknown>":
            print_ok("SUCCESS")
        else:
            print_fail("FAIL")

        #####################################################

        if board_type == BoardType.CORE2:
            print("Board type: Husarion CORE2")
        elif board_type == BoardType.LEOCORE:
            print("Board type: LeoCore")
        print(f"Firmware version: {current_firmware_version}")

        #####################################################

        if hardware in (TestMode.ALL, TestMode.BATTERY):
            write_flush("--> Battery validation.. ")
            print_test_result(tester.test_battery())

        if hardware in (TestMode.ALL, TestMode.IMU) and board_type == BoardType.LEOCORE:
            write_flush("--> IMU validation.. ")
            print_test_result(tester.test_imu())

        if hardware in (TestMode.ALL, TestMode.TORQUE, TestMode.ENCODER):
            write_flush("--> Checking if motors are loaded.. ")
            motors_loaded = tester.check_motor_load()
            if motors_loaded:
                print("YES")
            else:
                print("NO")

        if hardware in (TestMode.ALL, TestMode.ENCODER):
            write_flush("--> Encoders validation.. ")
            print_test_result(tester.test_encoder(motors_loaded))

        if (
            hardware in (TestMode.ALL, TestMode.TORQUE)
            and board_type == BoardType.LEOCORE
        ):
            write_flush("--> Torque sensors validation.. ")
            print_test_result(tester.test_torque(motors_loaded))

    finally:
        node.destroy_node()
        rclpy.shutdown()
