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
from .console import get_logger, log_step, report_results
from .utils import spin_for, parse_yaml

# Time given to the ROS graph to be discovered before it gets inspected
NODE_DISCOVERY_TIME = 3.0

# Time given to the subscriptions to match with the firmware publishers
TOPIC_DISCOVERY_TIME = 2.0

# Number of messages each sensor test has to validate
IMU_SAMPLES = 20
BATTERY_SAMPLES = 20

_log = get_logger("test_hw")


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

        self.logger = get_logger("HardwareTester")
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

    def test_imu(self) -> bool:
        """
        Validate the IMU readings while the robot is stationary.

        :return: True if all of the IMU checks pass, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Validating IMU data"):
                imu_valid = parse_yaml(os.path.join(self.path, "imu.yaml"))["imu"]
                timeout = imu_valid["timeout"]

                msg_cnt = 0
                time_last_msg = time.monotonic()

                while msg_cnt < IMU_SAMPLES:
                    rclpy.spin_once(self.node, timeout_sec=timeout)

                    time_now = time.monotonic()
                    if time_last_msg + timeout < time_now:
                        msg = (
                            f"No IMU message received within {timeout:.2f} s "
                            f"({msg_cnt}/{IMU_SAMPLES} samples collected)"
                        )
                        raise TimeoutError(msg)

                    if self.is_new_imu_data:
                        time_last_msg = time_now
                        self.is_new_imu_data = False
                        msg_cnt += 1

                        self._verify_imu_sample(self.imu_data, imu_valid)
        except (TimeoutError, ValueError) as exc:
            self.logger.error(
                "IMU test failed. Make sure the robot is stationary "
                "and the IMU data is being published: %s",
                exc,
            )
            return False
        return True

    def _verify_imu_sample(self, sample: Imu, limits: dict) -> None:
        """
        Verify a single IMU message against the configured limits.

        :param sample: The IMU message to validate
        :type sample: Imu
        :param limits: The "imu" section of the imu.yaml file
        :type limits: dict
        :raises ValueError: If any of the axes is outside of its tolerance
        """
        accel_del = limits["accel_del"]
        gyro_del = limits["gyro_del"]

        for name, value, expected, tolerance, unit in (
            ("accel_x", sample.accel_x, limits["accel_x"], accel_del, "m/s^2"),
            ("accel_y", sample.accel_y, limits["accel_y"], accel_del, "m/s^2"),
            ("accel_z", abs(sample.accel_z), limits["accel_z"], accel_del, "m/s^2"),
            ("gyro_x", sample.gyro_x, limits["gyro_x"], gyro_del, "rad/s"),
            ("gyro_y", sample.gyro_y, limits["gyro_y"], gyro_del, "rad/s"),
            ("gyro_z", sample.gyro_z, limits["gyro_z"], gyro_del, "rad/s"),
        ):
            if not expected - tolerance < value < expected + tolerance:
                msg = (
                    f"IMU {name}={value:.3f} {unit} deviates from the expected "
                    f"{expected:.3f} {unit} by more than {tolerance:.3f} {unit}"
                )
                raise ValueError(msg)

    def test_battery(self) -> bool:
        """
        Validate the battery voltage readings.

        :return: True if all of the battery checks pass, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Checking the battery voltage"):
                batt_valid = parse_yaml(os.path.join(self.path, "battery.yaml"))[
                    "battery"
                ]
                timeout = batt_valid["timeout"]

                msg_cnt = 0
                time_last_msg = time.monotonic()

                while msg_cnt < BATTERY_SAMPLES:
                    rclpy.spin_once(self.node, timeout_sec=timeout)

                    time_now = time.monotonic()
                    if time_last_msg + timeout < time_now:
                        msg = (
                            f"No battery message received within {timeout:.2f} s "
                            f"({msg_cnt}/{BATTERY_SAMPLES} samples collected)"
                        )
                        raise TimeoutError(msg)

                    if self.is_new_battery_data:
                        time_last_msg = time_now
                        self.is_new_battery_data = False
                        msg_cnt += 1

                        self._verify_battery_voltage(self.battery_data.data, batt_valid)
        except (TimeoutError, ValueError) as exc:
            self.logger.error("Battery test failed: %s", exc)
            return False
        return True

    def _verify_battery_voltage(self, voltage: float, limits: dict) -> None:
        """
        Verify a single battery voltage reading against the configured limits.

        :param voltage: The measured battery voltage
        :type voltage: float
        :param limits: The "battery" section of the battery.yaml file
        :type limits: dict
        :raises ValueError: If the voltage is outside of the valid range
        """
        if voltage <= limits["voltage_min"]:
            msg = (
                f"Battery voltage {voltage:.2f} V is at or below "
                f"the minimum of {limits['voltage_min']:.2f} V"
            )
            raise ValueError(msg)
        if voltage >= limits["voltage_max"]:
            msg = (
                f"Battery voltage {voltage:.2f} V is at or above "
                f"the maximum of {limits['voltage_max']:.2f} V"
            )
            raise ValueError(msg)


def test_hw(
    hardware: TestMode = TestMode.ALL,
    ros_args: Optional[list[str]] = None,
) -> int:
    """
    Run the hardware tests that do not require the robot to move.

    :param hardware: Which of the tests to run
    :type hardware: TestMode
    :param ros_args: Arguments forwarded to rclpy, or None to use sys.argv
    :type ros_args: Optional[list[str]]
    :return: 0 if every check passed, 1 otherwise
    :rtype: int
    """
    _log.info("Starting hardware tests.")

    with log_step("Initializing ROS node"):
        rclpy.init(args=ros_args)
        node = Node("leo_hardware_tester")
        spin_for(node, NODE_DISCOVERY_TIME)

    try:
        board_type = check_firmware_node(node)

        if board_type is None:
            return 1

        with log_step("Initializing the hardware tester"):
            tester = HardwareTester(node)

        results: list[tuple[str, bool]] = []

        if hardware in (TestMode.ALL, TestMode.BATTERY):
            results.append(("Battery voltage", tester.test_battery()))

        if hardware in (TestMode.ALL, TestMode.IMU) and board_type == BoardType.LEOCORE:
            results.append(("IMU", tester.test_imu()))

        if not results:
            _log.warning("No test was selected to run.")

        return report_results(_log, results)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
