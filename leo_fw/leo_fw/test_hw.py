# Copyright 2022-2026 Fictionlab sp. z o.o.
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
from collections.abc import Collection, Generator
from contextlib import contextmanager
from enum import Enum
from typing import Optional, TypeVar

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from leo_msgs.msg import Imu, WheelStates
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from .board import BoardType, check_firmware_node
from .console import (
    get_confirmation_prompt,
    get_logger,
    log_step,
    report_results,
)
from .utils import spin_for, parse_yaml
from .versions import get_firmware_binary_path, get_firmware_version

NODE_DISCOVERY_TIME = 3.0
TOPIC_DISCOVERY_TIME = 2.0
MOTOR_STOP_TIME = 0.2
PWM_RAMP_STEP_TIME = 0.2

# Number of messages each test has to validate
IMU_SAMPLES = 20
BATTERY_SAMPLES = 20
CAMERA_SAMPLES = 1
WHEEL_SAMPLES = 10

# Longest accepted gap between two wheel state messages, in seconds
WHEEL_SAMPLE_TIMEOUT = 0.5

# Subscribed to by three of the tests, so it is worth naming once
WHEEL_STATES_TOPIC = "firmware/wheel_states"

_log = get_logger("test_hw")

MsgT = TypeVar("MsgT")


class TestMode(Enum):
    FIRMWARE = "firmware"
    IMU = "imu"
    BATTERY = "battery"
    CAMERA = "camera"
    ENCODER = "encoder"
    TORQUE = "torque"
    ALL = "all"

    def __str__(self):
        return self.value


# What TestMode.ALL stands for: every mode that names an actual test
ALL_TESTS = frozenset(TestMode) - {TestMode.ALL}

# The tests that need the firmware node to report its board and version
FIRMWARE_INFO_TESTS = frozenset({TestMode.FIRMWARE, TestMode.IMU, TestMode.TORQUE})

# The tests that spin the wheels, and therefore have to be confirmed first
MOTOR_TESTS = frozenset({TestMode.ENCODER, TestMode.TORQUE})


def _resolve_tests(hardware: Collection[TestMode]) -> frozenset[TestMode]:
    """
    Expand the selected modes into the set of tests to run.

    :param hardware: The modes that were selected, in any order
    :type hardware: Collection[TestMode]
    :return: Every test the selection stands for, with ALL expanded
    :rtype: frozenset[TestMode]
    """
    if TestMode.ALL in hardware:
        return ALL_TESTS

    return frozenset(hardware)


class HardwareTester:
    """Validates the sensors, and the motors when they are allowed to spin."""

    WHEEL_NAMES = ["FL", "RL", "FR", "RR"]

    # PWM duty signs, in WHEEL_NAMES order, that make the robot spin in place
    PWM_SPIN_SIGNS = [1.0, -1.0, 1.0, -1.0]

    def __init__(self, node: Node):
        self.path = os.path.join(
            get_package_share_directory("leo_fw"), "data", "hw_tests"
        )

        self.logger = get_logger("HardwareTester")
        self.node = node

        self.cmd_pwm_pubs: dict[str, Publisher] = {}
        self.cmd_vel_pubs: dict[str, Publisher] = {}

    @contextmanager
    def _subscription(
        self, msg_type: type[MsgT], topic: str
    ) -> Generator[list[MsgT], None, None]:
        """
        Subscribe to a topic for the duration of the block, then unsubscribe.

        The subscription never outlives the test that needs it, so a test is
        never handed data from a topic it does not validate.

        The yielded list is the one the callback appends to, so it keeps growing
        while the block spins the node. Use :meth:`_wait_for_samples` to take a
        batch of fresh messages out of it.

        :param msg_type: Type of the messages to receive
        :type msg_type: type[MsgT]
        :param topic: Topic to subscribe to
        :type topic: str
        :return: The list the received messages get appended to
        :rtype: Generator[list[MsgT], None, None]
        """
        samples: list[MsgT] = []

        subscription = self.node.create_subscription(
            msg_type, topic, samples.append, qos_profile_sensor_data
        )

        try:
            # Let the subscription match with the publisher, then drop whatever
            # arrived while it did, so that every sample is collected fresh
            spin_for(self.node, TOPIC_DISCOVERY_TIME)
            samples.clear()

            yield samples
        finally:
            self.node.destroy_subscription(subscription)

    def _wait_for_samples(
        self,
        samples: list[MsgT],
        topic: str,
        sample_count: int,
        timeout: float,
    ) -> list[MsgT]:
        """
        Wait for a fixed number of fresh messages on an open subscription.

        Whatever arrived before the call is discarded, so that the returned
        batch only describes the state the robot is in now.

        :param samples: The list yielded by :meth:`_subscription`
        :type samples: list[MsgT]
        :param topic: Topic the messages arrive on, used in the error message
        :type topic: str
        :param sample_count: Number of messages to collect
        :type sample_count: int
        :param timeout: Longest accepted gap between two messages, in seconds
        :type timeout: float
        :return: The collected messages
        :rtype: list[MsgT]
        :raises TimeoutError: If the messages stop arriving
        """
        samples.clear()

        collected = 0
        time_last_msg = time.monotonic()

        while len(samples) < sample_count:
            rclpy.spin_once(self.node, timeout_sec=timeout)

            time_now = time.monotonic()
            if len(samples) > collected:
                collected = len(samples)
                time_last_msg = time_now
            elif time_last_msg + timeout < time_now:
                msg = (
                    f"No message received on {topic} within {timeout:.2f} s "
                    f"({collected}/{sample_count} samples collected)"
                )
                raise TimeoutError(msg)

        return samples[:sample_count]

    def _collect_samples(
        self,
        msg_type: type[MsgT],
        topic: str,
        sample_count: int,
        timeout: float,
    ) -> list[MsgT]:
        """
        Subscribe, collect a fixed number of fresh messages, then unsubscribe.

        :param msg_type: Type of the messages to collect
        :type msg_type: type[MsgT]
        :param topic: Topic to subscribe to
        :type topic: str
        :param sample_count: Number of messages to collect
        :type sample_count: int
        :param timeout: Longest accepted gap between two messages, in seconds
        :type timeout: float
        :return: The collected messages
        :rtype: list[MsgT]
        :raises TimeoutError: If the messages stop arriving
        """
        with self._subscription(msg_type, topic) as samples:
            return self._wait_for_samples(samples, topic, sample_count, timeout)

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

                samples = self._collect_samples(
                    Imu, "firmware/imu", IMU_SAMPLES, imu_valid["timeout"]
                )
                self._validate_imu_samples(samples, imu_valid)
        except (TimeoutError, ValueError) as exc:
            self.logger.error(
                "IMU test failed. Make sure the robot is stationary "
                "and the IMU data is being published: %s",
                exc,
            )
            return False
        return True

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

                samples = self._collect_samples(
                    Float32,
                    "firmware/battery",
                    BATTERY_SAMPLES,
                    batt_valid["timeout"],
                )
                self._validate_battery_samples(samples, batt_valid)
        except (TimeoutError, ValueError) as exc:
            self.logger.error("Battery test failed: %s", exc)
            return False
        return True

    def _validate_battery_samples(self, samples: list[Float32], limits: dict) -> None:
        """
        Validate every collected voltage reading against the configured limits.

        :param samples: The collected battery messages
        :type samples: list[Float32]
        :param limits: The "battery" section of the battery.yaml file
        :type limits: dict
        :raises ValueError: If any reading is outside of the valid range
        """
        voltage_min = limits["voltage_min"]
        voltage_max = limits["voltage_max"]

        voltages = [sample.data for sample in samples]

        too_low = [voltage for voltage in voltages if voltage <= voltage_min]
        too_high = [voltage for voltage in voltages if voltage >= voltage_max]

        failures: list[str] = []

        if too_low:
            failures.append(
                f"{len(too_low)}/{len(voltages)} samples at or below the minimum "
                f"of {voltage_min:.2f} V, lowest {min(too_low):.2f} V"
            )
        if too_high:
            failures.append(
                f"{len(too_high)}/{len(voltages)} samples at or above the maximum "
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

                self._collect_samples(
                    Image,
                    "camera/image_color",
                    CAMERA_SAMPLES,
                    camera_valid["timeout"],
                )
        except TimeoutError as exc:
            self.logger.error("Camera test failed: %s", exc)
            return False
        return True

    def create_motor_publishers(self) -> None:
        """Create the wheel command publishers and let the firmware match them."""
        for name in self.WHEEL_NAMES:
            self.cmd_pwm_pubs[name] = self.node.create_publisher(
                Float32, f"firmware/wheel_{name}/cmd_pwm_duty", 1
            )
            self.cmd_vel_pubs[name] = self.node.create_publisher(
                Float32, f"firmware/wheel_{name}/cmd_velocity", 1
            )

        spin_for(self.node, TOPIC_DISCOVERY_TIME)

    def _publish_velocity(self, velocity: float) -> None:
        """Command the same velocity on every wheel, driving the robot forward."""
        for publisher in self.cmd_vel_pubs.values():
            publisher.publish(Float32(data=velocity))

    def _publish_pwm(self, pwm: float) -> None:
        """Command the given PWM duty, with the sides opposed so the robot spins."""
        for name, sign in zip(self.WHEEL_NAMES, self.PWM_SPIN_SIGNS):
            self.cmd_pwm_pubs[name].publish(Float32(data=sign * pwm))

    def stop_motors(self) -> None:
        """Command every wheel to stop."""
        if not self.cmd_vel_pubs:
            return

        self._publish_velocity(0.0)
        self._publish_pwm(0.0)

        spin_for(self.node, MOTOR_STOP_TIME)

    def check_motor_load(self) -> bool:
        """
        Ramp up the PWM duty until the wheels either spin freely or stall.

        :return: True if the wheels are loaded, False if they spin freely
        :rtype: bool
        """
        speed_limit = 1.0
        motors_loaded = True

        with self._subscription(WheelStates, WHEEL_STATES_TOPIC) as samples:
            try:
                for pwm in range(30):
                    self._publish_pwm(float(pwm))

                    spin_for(self.node, PWM_RAMP_STEP_TIME)

                    if not samples:
                        continue

                    velocity = samples[-1].velocity
                    samples.clear()

                    # Every wheel turns in the direction it was commanded, and
                    # faster than it could under load
                    if all(
                        sign * velocity[i] > speed_limit
                        for i, sign in enumerate(self.PWM_SPIN_SIGNS)
                    ):
                        motors_loaded = False
                        break
            finally:
                self._publish_pwm(0.0)

        return motors_loaded

    def test_encoder(self, motors_loaded: bool = True) -> bool:
        """
        Validate the wheel encoders by driving the wheels at set velocities.

        :param motors_loaded: Whether the wheels are loaded, selecting the limits
        :type motors_loaded: bool
        :return: True if every wheel reported a valid velocity, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Validating the wheel encoders"):
                if motors_loaded:
                    wheel_valid = parse_yaml(
                        os.path.join(self.path, "encoder_load.yaml")
                    )
                else:
                    wheel_valid = parse_yaml(os.path.join(self.path, "encoder.yaml"))

                errors: dict[str, str] = {}

                with self._subscription(WheelStates, WHEEL_STATES_TOPIC) as samples:
                    try:
                        for wheel_test in wheel_valid["tests"]:
                            self._publish_velocity(wheel_test["velocity"])
                            spin_for(self.node, wheel_test["time"])
                            self._collect_velocity_errors(
                                wheel_test,
                                self._wait_for_samples(
                                    samples,
                                    WHEEL_STATES_TOPIC,
                                    WHEEL_SAMPLES,
                                    WHEEL_SAMPLE_TIMEOUT,
                                ),
                                errors,
                            )
                    finally:
                        self._publish_velocity(0.0)

                self._check_wheel_errors(errors)
        except (TimeoutError, ValueError) as exc:
            self.logger.error("Encoder test failed: %s", exc)
            return False
        return True

    def _collect_velocity_errors(
        self, wheel_test: dict, samples: list[WheelStates], errors: dict[str, str]
    ) -> None:
        """
        Record the wheels whose velocity is outside of the tolerance.

        :param wheel_test: A single entry of the encoder yaml "tests" list
        :type wheel_test: dict
        :param samples: Wheel states collected at the end of the step
        :type samples: list[WheelStates]
        :param errors: Mapping of wheel name to its failure description
        :type errors: dict[str, str]
        """
        setpoint = wheel_test["velocity"]
        speed_min = setpoint - wheel_test["tolerance"]
        speed_max = setpoint + wheel_test["tolerance"]

        for i, name in enumerate(self.WHEEL_NAMES):
            if name in errors:
                continue

            values = [sample.velocity[i] for sample in samples]
            invalid = [
                (index, value)
                for index, value in enumerate(values)
                if not speed_min <= value <= speed_max
            ]

            if invalid:
                _, worst = max(invalid, key=lambda item: abs(item[1] - setpoint))
                indices = ", ".join(str(index) for index, _ in invalid)
                errors[name] = (
                    f"{name} was out of range in {len(invalid)}/{len(values)} "
                    f"samples at a {setpoint:.2f} rad/s setpoint "
                    f"(samples {indices}), worst {worst:.2f} rad/s against "
                    f"{speed_min:.2f}..{speed_max:.2f} rad/s"
                )

    def test_torque(self, motors_loaded: bool = True) -> bool:
        """
        Validate the torque sensors by driving the wheels at set PWM duties.

        :param motors_loaded: Whether the wheels are loaded, selecting the limits
        :type motors_loaded: bool
        :return: True if every wheel reported a valid torque, False otherwise
        :rtype: bool
        """
        try:
            with log_step("Validating the torque sensors"):
                if motors_loaded:
                    torque_valid = parse_yaml(
                        os.path.join(self.path, "torque_load.yaml")
                    )
                else:
                    torque_valid = parse_yaml(os.path.join(self.path, "torque.yaml"))

                errors: dict[str, str] = {}

                with self._subscription(WheelStates, WHEEL_STATES_TOPIC) as samples:
                    try:
                        for torque_test in torque_valid["tests"]:
                            self._publish_pwm(torque_test["pwm"])
                            spin_for(self.node, torque_test["time"])
                            self._collect_torque_errors(
                                torque_test,
                                self._wait_for_samples(
                                    samples,
                                    WHEEL_STATES_TOPIC,
                                    WHEEL_SAMPLES,
                                    WHEEL_SAMPLE_TIMEOUT,
                                ),
                                errors,
                            )
                    finally:
                        self._publish_pwm(0.0)

                self._check_wheel_errors(errors)
        except (TimeoutError, ValueError) as exc:
            self.logger.error("Torque sensor test failed: %s", exc)
            return False
        return True

    def _collect_torque_errors(
        self, torque_test: dict, samples: list[WheelStates], errors: dict[str, str]
    ) -> None:
        """
        Record the wheels whose torque is outside of the valid range.

        :param torque_test: A single entry of the torque yaml "tests" list
        :type torque_test: dict
        :param samples: Wheel states collected at the end of the step
        :type samples: list[WheelStates]
        :param errors: Mapping of wheel name to its failure description
        :type errors: dict[str, str]
        """
        torque_min = torque_test["torque_min"]
        torque_max = torque_test["torque_max"]
        midpoint = (torque_min + torque_max) / 2.0

        for i, name in enumerate(self.WHEEL_NAMES):
            if name in errors:
                continue

            values = [sample.torque[i] for sample in samples]
            invalid = [
                (index, value)
                for index, value in enumerate(values)
                if not torque_min <= value <= torque_max
            ]

            if invalid:
                _, worst = max(invalid, key=lambda item: abs(item[1] - midpoint))
                indices = ", ".join(str(index) for index, _ in invalid)
                errors[name] = (
                    f"{name} was out of range in {len(invalid)}/{len(values)} "
                    f"samples at a {torque_test['pwm']:.0f}% PWM duty "
                    f"(samples {indices}), worst {worst:.3f} Nm against "
                    f"{torque_min:.3f}..{torque_max:.3f} Nm"
                )

    def _check_wheel_errors(self, errors: dict[str, str]) -> None:
        """
        Raise a single error describing every faulty wheel.

        :param errors: Mapping of wheel name to its failure description
        :type errors: dict[str, str]
        :raises ValueError: If any wheel was recorded as faulty
        """
        if errors:
            msg = "; ".join(errors[name] for name in self.WHEEL_NAMES if name in errors)
            raise ValueError(msg)


def _run_sensor_tests(
    tester: HardwareTester,
    tests: frozenset[TestMode],
    board_type: Optional[BoardType],
    firmware_version: str,
) -> list[tuple[str, bool]]:
    """
    Run the selected tests that leave the robot stationary.

    :param tester: The tester to run the tests on
    :type tester: HardwareTester
    :param tests: The tests to run, as resolved by _resolve_tests
    :type tests: frozenset[TestMode]
    :param board_type: The board the firmware node reported, if it was checked
    :type board_type: Optional[BoardType]
    :param firmware_version: The version the firmware node reported
    :type firmware_version: str
    :return: The name and outcome of every test that ran
    :rtype: list[tuple[str, bool]]
    """
    results: list[tuple[str, bool]] = []

    if TestMode.FIRMWARE in tests and board_type is not None:
        results.append(
            (
                "Firmware version",
                tester.test_firmware_version(board_type, firmware_version),
            )
        )

    if TestMode.BATTERY in tests:
        results.append(("Battery voltage", tester.test_battery()))

    if TestMode.IMU in tests and board_type == BoardType.LEOCORE:
        results.append(("IMU", tester.test_imu()))

    if TestMode.CAMERA in tests:
        results.append(("Camera", tester.test_camera()))

    return results


def _run_motor_tests(
    tester: HardwareTester,
    tests: frozenset[TestMode],
    board_type: Optional[BoardType],
) -> list[tuple[str, bool]]:
    """
    Prepare the motors and run the selected tests that spin the wheels.

    :param tester: The tester to run the tests on
    :type tester: HardwareTester
    :param tests: The tests to run, as resolved by _resolve_tests
    :type tests: frozenset[TestMode]
    :param board_type: The board the firmware node reported, if it was checked
    :type board_type: Optional[BoardType]
    :return: The name and outcome of every test that ran
    :rtype: list[tuple[str, bool]]
    """
    results: list[tuple[str, bool]] = []

    with log_step("Preparing the motors"):
        tester.create_motor_publishers()

    with log_step("Checking if the motors are loaded"):
        motors_loaded = tester.check_motor_load()

    _log.info("Motors are %s.", "loaded" if motors_loaded else "not loaded")

    if TestMode.ENCODER in tests:
        results.append(("Wheel encoders", tester.test_encoder(motors_loaded)))

    if TestMode.TORQUE in tests and board_type == BoardType.LEOCORE:
        results.append(("Torque sensors", tester.test_torque(motors_loaded)))

    return results


def test_hw(
    hardware: Collection[TestMode] = (TestMode.ALL,),
    ros_args: Optional[list[str]] = None,
) -> int:
    """
    Run the hardware tests.

    The tests always run in a fixed order regardless of how they were selected,
    with the ones that spin the wheels left for last.

    :param hardware: Which of the tests to run, in any order
    :type hardware: Collection[TestMode]
    :param ros_args: Arguments forwarded to rclpy, or None to use sys.argv
    :type ros_args: Optional[list[str]]
    :return: 0 if every check passed, 1 otherwise
    :rtype: int
    """
    _log.info("Starting hardware tests.")

    tests = _resolve_tests(hardware)

    with log_step("Initializing ROS node"):
        rclpy.init(args=ros_args)
        node = Node("leo_hardware_tester")
        spin_for(node, NODE_DISCOVERY_TIME)

    tester = HardwareTester(node)

    try:
        board_type: Optional[BoardType] = None

        firmware_version = "<unknown>"

        if tests & FIRMWARE_INFO_TESTS:
            firmware_info = check_firmware_node(node)

            if firmware_info is None:
                return 1

            board_type, firmware_version = firmware_info

        results = _run_sensor_tests(tester, tests, board_type, firmware_version)

        if tests & MOTOR_TESTS:
            _log.warning(
                "The motors will spin during this procedure. "
                "Make sure the robot is placed on a stand or has enough free space "
                "around it, and keep clear of the wheels."
            )

            if get_confirmation_prompt("Do you want to start the motor tests?"):
                results += _run_motor_tests(tester, tests, board_type)
            else:
                _log.info("Motor tests cancelled by the user.")

        if not results:
            _log.warning("No test was selected to run.")

        return report_results(_log, results)

    finally:
        tester.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
