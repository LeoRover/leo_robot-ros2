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
from sensor_msgs.msg import Image

from .board import BoardType, check_firmware_node
from .console import get_logger, log_step, report_results
from .utils import spin_for, parse_yaml
from .versions import get_firmware_binary_path, get_firmware_version

# Time given to the ROS graph to be discovered before it gets inspected
NODE_DISCOVERY_TIME = 3.0

# Time given to the subscriptions to match with the firmware publishers
TOPIC_DISCOVERY_TIME = 2.0

# Number of messages each sensor test has to validate
IMU_SAMPLES = 20
BATTERY_SAMPLES = 20

# The camera test only checks that the stream is alive, so a single frame is
# enough
CAMERA_SAMPLES = 1

_log = get_logger("test_hw")


class TestMode(Enum):
    FIRMWARE = "firmware"
    IMU = "imu"
    BATTERY = "battery"
    CAMERA = "camera"
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
        self.is_new_camera_data = False

        self.imu_data = Imu()
        self.battery_data = Float32()
        self.camera_data = Image()

        ### Subscriptions

        self.battery_sub = node.create_subscription(
            Float32, "firmware/battery", self.battery_callback, qos_profile_sensor_data
        )
        self.imu_sub = node.create_subscription(
            Imu, "firmware/imu", self.imu_callback, qos_profile_sensor_data
        )
        self.camera_sub = node.create_subscription(
            Image, "camera/image_color", self.camera_callback, qos_profile_sensor_data
        )

        spin_for(self.node, TOPIC_DISCOVERY_TIME)

    def battery_callback(self, data: Float32) -> None:
        self.battery_data = data
        self.is_new_battery_data = True

    def imu_callback(self, data: Imu) -> None:
        self.imu_data = data
        self.is_new_imu_data = True

    def camera_callback(self, data: Image) -> None:
        self.camera_data = data
        self.is_new_camera_data = True

    def test_firmware_version(
        self, board_type: BoardType, current_version: str
    ) -> bool:
        """
        Check that the board runs the firmware version shipped in this package.

        :param board_type: The board the firmware node reported
        :type board_type: BoardType
        :param current_version: The version the firmware node reported
        :type current_version: str
        :return: True if the versions match, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Checking the firmware version"):
                if current_version == "<unknown>":
                    msg = "The firmware node did not report its version"
                    raise ValueError(msg)

                binary_path = get_firmware_binary_path(board_type)
                expected_version = get_firmware_version(binary_path, board_type)

                if current_version != expected_version:
                    msg = (
                        f"The board runs firmware {current_version}, but this "
                        f"package ships {expected_version}. "
                        "Run the flash script to update it."
                    )
                    raise ValueError(msg)
        except (OSError, ValueError) as exc:
            self.logger.error("Firmware version test failed: %s", exc)
            return False
        return True

    def test_imu(self) -> bool:
        """
        Validate the IMU readings while the robot is stationary.

        :return: True if all of the IMU checks pass, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Validating IMU data"):
                imu_valid = parse_yaml(os.path.join(self.path, "imu.yaml"))["imu"]

                samples = self._collect_imu_samples(IMU_SAMPLES, imu_valid["timeout"])
                self._validate_imu_samples(samples, imu_valid)
        except (TimeoutError, ValueError) as exc:
            self.logger.error(
                "IMU test failed. Make sure the robot is stationary "
                "and the IMU data is being published: %s",
                exc,
            )
            return False
        return True

    def _collect_imu_samples(self, sample_count: int, timeout: float) -> list[Imu]:
        """
        Collect a fixed number of fresh IMU messages.

        :param sample_count: Number of messages to collect
        :type sample_count: int
        :param timeout: Longest accepted gap between two messages, in seconds
        :type timeout: float
        :return: The collected messages
        :rtype: list[Imu]
        :raises TimeoutError: If the messages stop arriving
        """
        samples: list[Imu] = []
        time_last_msg = time.monotonic()

        while len(samples) < sample_count:
            rclpy.spin_once(self.node, timeout_sec=timeout)

            time_now = time.monotonic()
            if time_last_msg + timeout < time_now:
                msg = (
                    f"No IMU message received within {timeout:.2f} s "
                    f"({len(samples)}/{sample_count} samples collected)"
                )
                raise TimeoutError(msg)

            if self.is_new_imu_data:
                time_last_msg = time_now
                self.is_new_imu_data = False
                samples.append(self.imu_data)

        return samples

    def _validate_imu_samples(self, samples: list[Imu], limits: dict) -> None:
        """
        Validate every collected IMU sample against the configured limits.

        Reports each axis once, with the count of offending samples and the
        reading that deviated the most.

        :param samples: The IMU messages to validate
        :type samples: list[Imu]
        :param limits: The "imu" section of the imu.yaml file
        :type limits: dict
        :raises ValueError: If any of the axes is outside of its tolerance
        """
        accel_del = limits["accel_del"]
        gyro_del = limits["gyro_del"]

        axes = (
            ("accel_x", lambda s: s.accel_x, limits["accel_x"], accel_del, "m/s^2"),
            ("accel_y", lambda s: s.accel_y, limits["accel_y"], accel_del, "m/s^2"),
            (
                "accel_z",
                lambda s: abs(s.accel_z),
                limits["accel_z"],
                accel_del,
                "m/s^2",
            ),
            ("gyro_x", lambda s: s.gyro_x, limits["gyro_x"], gyro_del, "rad/s"),
            ("gyro_y", lambda s: s.gyro_y, limits["gyro_y"], gyro_del, "rad/s"),
            ("gyro_z", lambda s: s.gyro_z, limits["gyro_z"], gyro_del, "rad/s"),
        )

        failures: list[str] = []

        for name, get_value, expected, tolerance, unit in axes:
            values = [get_value(sample) for sample in samples]
            invalid = [
                (abs(value - expected), value)
                for value in values
                if not expected - tolerance < value < expected + tolerance
            ]

            if invalid:
                worst = max(invalid)[1]
                failures.append(
                    f"{name} was out of range in {len(invalid)}/{len(values)} "
                    f"samples, worst {worst:.3f} {unit} against the expected "
                    f"{expected:.3f} +/- {tolerance:.3f} {unit}"
                )

        if failures:
            raise ValueError("; ".join(failures))

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

                samples = self._collect_battery_samples(
                    BATTERY_SAMPLES, batt_valid["timeout"]
                )
                self._validate_battery_samples(samples, batt_valid)
        except (TimeoutError, ValueError) as exc:
            self.logger.error("Battery test failed: %s", exc)
            return False
        return True

    def _collect_battery_samples(
        self, sample_count: int, timeout: float
    ) -> list[float]:
        """
        Collect a fixed number of fresh battery voltage readings.

        :param sample_count: Number of readings to collect
        :type sample_count: int
        :param timeout: Longest accepted gap between two messages, in seconds
        :type timeout: float
        :return: The collected voltages
        :rtype: list[float]
        :raises TimeoutError: If the messages stop arriving
        """
        samples: list[float] = []
        time_last_msg = time.monotonic()

        while len(samples) < sample_count:
            rclpy.spin_once(self.node, timeout_sec=timeout)

            time_now = time.monotonic()
            if time_last_msg + timeout < time_now:
                msg = (
                    f"No battery message received within {timeout:.2f} s "
                    f"({len(samples)}/{sample_count} samples collected)"
                )
                raise TimeoutError(msg)

            if self.is_new_battery_data:
                time_last_msg = time_now
                self.is_new_battery_data = False
                samples.append(self.battery_data.data)

        return samples

    def _validate_battery_samples(self, samples: list[float], limits: dict) -> None:
        """
        Validate every collected voltage reading against the configured limits.

        :param samples: The collected voltages
        :type samples: list[float]
        :param limits: The "battery" section of the battery.yaml file
        :type limits: dict
        :raises ValueError: If any reading is outside of the valid range
        """
        voltage_min = limits["voltage_min"]
        voltage_max = limits["voltage_max"]

        too_low = [voltage for voltage in samples if voltage <= voltage_min]
        too_high = [voltage for voltage in samples if voltage >= voltage_max]

        failures: list[str] = []

        if too_low:
            failures.append(
                f"{len(too_low)}/{len(samples)} samples at or below the minimum "
                f"of {voltage_min:.2f} V, lowest {min(too_low):.2f} V"
            )
        if too_high:
            failures.append(
                f"{len(too_high)}/{len(samples)} samples at or above the maximum "
                f"of {voltage_max:.2f} V, highest {max(too_high):.2f} V"
            )

        if failures:
            raise ValueError("; ".join(failures))

    def test_camera(self) -> bool:
        """
        Check that the camera is publishing images.

        Only the arrival of a frame is checked, not its content.

        :return: True if a frame was received, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Checking the camera stream"):
                camera_valid = parse_yaml(os.path.join(self.path, "camera.yaml"))[
                    "camera"
                ]

                self._collect_camera_samples(CAMERA_SAMPLES, camera_valid["timeout"])
        except TimeoutError as exc:
            self.logger.error("Camera test failed: %s", exc)
            return False
        return True

    def _collect_camera_samples(self, sample_count: int, timeout: float) -> list[Image]:
        """
        Collect a fixed number of fresh camera frames.

        :param sample_count: Number of frames to collect
        :type sample_count: int
        :param timeout: Longest accepted gap between two frames, in seconds
        :type timeout: float
        :return: The collected frames
        :rtype: list[Image]
        :raises TimeoutError: If the frames stop arriving
        """
        samples: list[Image] = []
        time_last_msg = time.monotonic()

        while len(samples) < sample_count:
            rclpy.spin_once(self.node, timeout_sec=timeout)

            time_now = time.monotonic()
            if time_last_msg + timeout < time_now:
                msg = (
                    f"No camera message received within {timeout:.2f} s "
                    f"({len(samples)}/{sample_count} samples collected)"
                )
                raise TimeoutError(msg)

            if self.is_new_camera_data:
                time_last_msg = time_now
                self.is_new_camera_data = False
                samples.append(self.camera_data)

        return samples


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
        board_type: Optional[BoardType] = None
        firmware_version: Optional[str] = None

        if hardware in (TestMode.ALL, TestMode.FIRMWARE, TestMode.IMU):
            firmware_info = check_firmware_node(node)

            if firmware_info is None:
                return 1

            board_type, firmware_version = firmware_info

        with log_step("Initializing the hardware tester"):
            tester = HardwareTester(node)

        results: list[tuple[str, bool]] = []

        if hardware in (TestMode.ALL, TestMode.FIRMWARE):
            if board_type == BoardType.LEOCORE:
                results.append(
                    (
                        "Firmware version",
                        tester.test_firmware_version(board_type, firmware_version),
                    )
                )
            else:
                _log.warning("CORE2 detected, the firmware version is not checked.")

        if hardware in (TestMode.ALL, TestMode.BATTERY):
            results.append(("Battery voltage", tester.test_battery()))

        if hardware in (TestMode.ALL, TestMode.IMU) and board_type == BoardType.LEOCORE:
            results.append(("IMU", tester.test_imu()))

        if hardware in (TestMode.ALL, TestMode.CAMERA):
            results.append(("Camera", tester.test_camera()))

        if not results:
            _log.warning("No test was selected to run.")

        return report_results(_log, results)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
