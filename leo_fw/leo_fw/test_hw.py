# Copyright 2022 Kell Ideas sp. z o.o.
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

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from ament_index_python.packages import get_package_share_directory

from leo_msgs.msg import Imu, WheelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from .utils import write_flush
from .board import BoardType, determine_board, check_firmware_version
from .utils import CSIColor, parse_yaml


class TestMode(Enum):
    ENCODER = "encoder"
    TORQUE = "torque"
    IMU = "imu"
    BATTERY = "battery"
    ALL = "all"

    def __str__(self):
        return self.value


class HardwareTester(Node):

    is_new_imu_data = False
    is_new_wheel_data = False
    is_new_battery_data = False
    is_wheel_loaded = True

    imu_data = Imu()
    wheel_data = WheelStates()
    battery_data = Float32()

    def __init__(self):
        super().__init__("hardware_tester")
        self.path = os.path.join(
            get_package_share_directory("leo_fw"), "data", "hw_tests"
        )

        ### Publishers

        self.cmd_vel_pub = self.create_publisher(
            Twist, "firmware/wheel_FL/cmd_pwm_duty", 1
        )
        self.cmd_pwmfl_pub = self.create_publisher(
            Float32, "firmware/wheel_FL/cmd_pwm_duty", 1
        )
        self.cmd_pwmrl_pub = self.create_publisher(
            Float32, "firmware/wheel_RL/cmd_pwm_duty", 1
        )
        self.cmd_pwmfr_pub = self.create_publisher(
            Float32, "firmware/wheel_FR/cmd_pwm_duty", 1
        )
        self.cmd_pwmrr_pub = self.create_publisher(
            Float32, "firmware/wheel_RR/cmd_pwm_duty", 1
        )
        self.cmd_velfl_pub = self.create_publisher(
            Float32, "firmware/wheel_FL/cmd_velocity", 1
        )
        self.cmd_velrl_pub = self.create_publisher(
            Float32, "firmware/wheel_RL/cmd_velocity", 1
        )
        self.cmd_velfr_pub = self.create_publisher(
            Float32, "firmware/wheel_FR/cmd_velocity", 1
        )
        self.cmd_velrr_pub = self.create_publisher(
            Float32, "firmware/wheel_RR/cmd_velocity", 1
        )

        ### Subscriptions

        self.battery_sub = self.create_subscription(
            Float32, "firmware/battery", self.battery_callback, 1
        )
        self.wheel_sub = self.create_subscription(
            WheelStates, "firmware/wheel_states", self.wheel_callback, 1
        )
        self.imu_sub = self.create_subscription(
            Imu, "firmware/imu", self.imu_callback, 1
        )

    def battery_callback(self, data: Float32) -> None:
        self.battery_data = data
        self.is_new_battery_data = True

    def imu_callback(self, data: Imu) -> None:
        self.imu_data = data
        self.is_new_imu_data = True

    def wheel_callback(self, data: WheelStates) -> None:
        self.wheel_data = data
        self.is_new_wheel_data = True

    def check_motor_load(self) -> None:
        speed_limit = 1.0

        for pwm in range(30):
            pwm_value = float(pwm)

            self.cmd_pwmfl_pub.publish(Float32(data=pwm_value))
            self.cmd_pwmfr_pub.publish(Float32(data=pwm_value))
            self.cmd_pwmrl_pub.publish(Float32(data=-pwm_value))
            self.cmd_pwmrr_pub.publish(Float32(data=-pwm_value))

            rclpy.spin_until_future_complete(self, Future(), None, 0.2)

            if (
                self.wheel_data.velocity[0] > speed_limit
                and self.wheel_data.velocity[1] < -speed_limit
                and self.wheel_data.velocity[2] > speed_limit
                and self.wheel_data.velocity[3] < -speed_limit
            ):
                self.is_wheel_loaded = False
                break

        if self.is_wheel_loaded:
            print(CSIColor.OKGREEN + "LOAD" + CSIColor.ENDC)
        else:
            print(CSIColor.OKGREEN + "UNLOAD" + CSIColor.ENDC)

        self.cmd_pwmfl_pub.publish(Float32(data=0.0))
        self.cmd_pwmfr_pub.publish(Float32(data=0.0))
        self.cmd_pwmrl_pub.publish(Float32(data=0.0))
        self.cmd_pwmrr_pub.publish(Float32(data=0.0))

    def check_encoder(self) -> bool:
        is_error = [False] * 4

        if self.is_wheel_loaded:
            wheel_valid = parse_yaml(os.path.join(self.path, "encoder_load.yaml"))
        else:
            wheel_valid = parse_yaml(os.path.join(self.path, "encoder.yaml"))

        for wheel_test in wheel_valid["tests"]:

            self.cmd_velfl_pub.publish(Float32(data=wheel_test["velocity"]))
            self.cmd_velfr_pub.publish(Float32(data=wheel_test["velocity"]))
            self.cmd_velrl_pub.publish(Float32(data=wheel_test["velocity"]))
            self.cmd_velrr_pub.publish(Float32(data=wheel_test["velocity"]))

            rclpy.spin_until_future_complete(self, Future(), None, wheel_test["time"])

            speed_min = wheel_test["velocity"] - wheel_test["offset"]
            speed_max = wheel_test["velocity"] + wheel_test["offset"]

            for i in range(0, 4):
                if not speed_min < self.wheel_data.velocity[i] < speed_max:
                    is_error[i] = True

        self.cmd_velfl_pub.publish(Float32(data=0.0))
        self.cmd_velfr_pub.publish(Float32(data=0.0))
        self.cmd_velrl_pub.publish(Float32(data=0.0))
        self.cmd_velrr_pub.publish(Float32(data=0.0))

        if any(is_error):
            error_msg = "ERROR WHEEL ENCODER "
            error_msg += str(is_error)
            print(CSIColor.FAIL + error_msg + CSIColor.ENDC)
            return False

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return True

    def check_torque(self) -> bool:
        is_error = [False] * 4

        if self.is_wheel_loaded:
            torque_valid = parse_yaml(os.path.join(self.path, "torque_load.yaml"))
        elif self.is_wheel_loaded:
            torque_valid = parse_yaml(os.path.join(self.path, "torque.yaml"))

        for torque_test in torque_valid["tests"]:
            self.cmd_pwmfl_pub.publish(Float32(data=torque_test["pwm"]))
            self.cmd_pwmfr_pub.publish(Float32(data=torque_test["pwm"]))
            self.cmd_pwmrl_pub.publish(Float32(data=-torque_test["pwm"]))
            self.cmd_pwmrr_pub.publish(Float32(data=-torque_test["pwm"]))

            rclpy.spin_until_future_complete(self, Future(), None, torque_test["time"])

            torque_min = torque_test["torque"]
            torque_max = torque_test["torque"] + 1.0

            for i in range(4):
                if not torque_min < self.wheel_data.torque[i] < torque_max:
                    is_error[i] = True

        self.cmd_pwmfl_pub.publish(Float32(data=0.0))
        self.cmd_pwmfr_pub.publish(Float32(data=0.0))
        self.cmd_pwmrl_pub.publish(Float32(data=0.0))
        self.cmd_pwmrr_pub.publish(Float32(data=0.0))

        if any(is_error):
            error_msg = "ERROR WHEEL TORQUE "
            error_msg += str(is_error)
            print(CSIColor.FAIL + error_msg + CSIColor.ENDC)
            return False

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return True

    def check_imu(self) -> bool:
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
            rclpy.spin_once(self)

            time_now = time.monotonic()
            if time_last_msg + imu_valid["imu"]["timeout"] < time_now:
                print(CSIColor.WARNING + "TIMEOUT" + CSIColor.ENDC)
                return False

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
                    print(CSIColor.FAIL + "INVALID DATA" + CSIColor.ENDC)
                    return False

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return True

    def check_battery(self) -> bool:
        msg_cnt = 0
        time_last_msg = time.monotonic()
        batt_valid = parse_yaml(os.path.join(self.path, "battery.yaml"))

        while msg_cnt < 50:
            rclpy.spin_once(self)

            time_now = time.monotonic()
            if time_last_msg + batt_valid["battery"]["timeout"] < time_now:
                print(CSIColor.WARNING + "TIMEOUT" + CSIColor.ENDC)
                return False

            if self.is_new_battery_data:
                time_last_msg = time_now
                self.is_new_battery_data = False
                msg_cnt += 1

                if self.battery_data.data <= batt_valid["battery"]["voltage_min"]:
                    print(CSIColor.FAIL + "LOW VOLTAGE" + CSIColor.ENDC)
                    return False
                if self.battery_data.data >= batt_valid["battery"]["voltage_max"]:
                    print(CSIColor.FAIL + "HIGH VOLTAGE" + CSIColor.ENDC)
                    return False

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return True


# pylint: disable=too-many-branches,too-many-statements
def test_hw(
    hardware: TestMode = TestMode.ALL,
) -> None:

    write_flush("--> Initializing ROS node.. ")
    rclpy.init()
    tester = HardwareTester()
    print("DONE")

    try:
        write_flush("--> Checking if firmware node is active.. ")

        if tester.get_namespace() + "firmware" in tester.get_node_names():
            print("YES")
        else:
            print("NO")
            print(
                "Firmware node is not active. "
                "Will not be able to validate hardware. "
                "Try to flash the firmware or restart the Micro-ROS Agent."
            )
            return

        #####################################################

        write_flush("--> Trying to determine board type.. ")

        board_type = None
        board_type = determine_board(tester)

        if board_type is not None:
            print("SUCCESS")
        else:
            print("FAIL")
            print(
                "Can not determine board type. "
                "Update the firmware and try to rerun the script."
            )
            return

        #####################################################

        write_flush("--> Trying to check the current firmware version.. ")

        current_firmware_version = "<unknown>"
        current_firmware_version = check_firmware_version(tester)

        if current_firmware_version != "<unknown>":
            print("SUCCESS")
        else:
            print("FAIL")

        #####################################################

        if board_type == BoardType.CORE2:
            print("Board type: Husarion CORE2")
        elif board_type == BoardType.LEOCORE:
            print("Board type: LeoCore")
        print(f"Firmware version: {current_firmware_version}")

        #####################################################

        if hardware in (TestMode.ALL, TestMode.BATTERY):
            write_flush("--> Battery validation.. ")
            tester.check_battery()

        if hardware in (TestMode.ALL, TestMode.IMU) and board_type == BoardType.LEOCORE:
            write_flush("--> IMU validation.. ")
            tester.check_imu()

        if hardware in (TestMode.ALL, TestMode.TORQUE, TestMode.ENCODER):
            write_flush("--> Motors load test.. ")
            tester.check_motor_load()

        if hardware in (TestMode.ALL, TestMode.ENCODER):
            write_flush("--> Encoders validation.. ")
            tester.check_encoder()

        if (
            hardware in (TestMode.ALL, TestMode.TORQUE)
            and board_type == BoardType.LEOCORE
        ):
            write_flush("--> Torque sensors validation.. ")
            tester.check_torque()

    finally:
        tester.destroy_node()
        rclpy.shutdown()
