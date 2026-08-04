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

from leo_msgs.msg import WheelStates
from std_msgs.msg import Float32

from .board import BoardType, check_firmware_node
from .console import (
    get_confirmation_prompt,
    get_logger,
    log_step,
    report_results,
)
from .utils import spin_for, parse_yaml

NODE_DISCOVERY_TIME = 3.0
TOPIC_DISCOVERY_TIME = 2.0
MOTOR_STOP_TIME = 0.2
PWM_RAMP_STEP_TIME = 0.2

# Number of wheel states validated at the end of every test step, and the
# longest accepted gap between two of them
WHEEL_SAMPLES = 10
WHEEL_SAMPLE_TIMEOUT = 0.5

_log = get_logger("test_motors")


class MotorTestMode(Enum):
    ENCODER = "encoder"
    TORQUE = "torque"
    ALL = "all"

    def __str__(self):
        return self.value


class MotorTester:
    """Validates the wheel encoders and torque sensors by driving the motors."""

    WHEEL_NAMES = ["FL", "RL", "FR", "RR"]

    def __init__(self, node: Node):
        self.path = os.path.join(
            get_package_share_directory("leo_fw"), "data", "hw_tests"
        )

        self.logger = get_logger("MotorTester")
        self.node = node

        self.is_new_wheel_data = False
        self.wheel_data = WheelStates()

        ### Publishers

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

        self.wheel_sub = node.create_subscription(
            WheelStates,
            "firmware/wheel_states",
            self.wheel_callback,
            qos_profile_sensor_data,
        )

        spin_for(self.node, TOPIC_DISCOVERY_TIME)

    def wheel_callback(self, data: WheelStates) -> None:
        self.wheel_data = data
        self.is_new_wheel_data = True

    def _collect_wheel_samples(
        self, sample_count: int, timeout: float
    ) -> list[WheelStates]:
        """
        Collect a fixed number of fresh wheel state messages.

        :param sample_count: Number of messages to collect
        :type sample_count: int
        :param timeout: Longest accepted gap between two messages, in seconds
        :type timeout: float
        :return: The collected messages
        :rtype: list[WheelStates]
        :raises TimeoutError: If the messages stop arriving
        """
        samples: list[WheelStates] = []
        time_last_msg = time.monotonic()

        while len(samples) < sample_count:
            rclpy.spin_once(self.node, timeout_sec=timeout)

            time_now = time.monotonic()
            if time_last_msg + timeout < time_now:
                msg = (
                    f"No wheel states message received within {timeout:.2f} s "
                    f"({len(samples)}/{sample_count} samples collected)"
                )
                raise TimeoutError(msg)

            if self.is_new_wheel_data:
                time_last_msg = time_now
                self.is_new_wheel_data = False
                samples.append(self.wheel_data)

        return samples

    def _publish_velocity(self, velocity: float) -> None:
        """Command the same velocity on every wheel, driving the robot forward."""
        self.cmd_velfl_pub.publish(Float32(data=velocity))
        self.cmd_velfr_pub.publish(Float32(data=velocity))
        self.cmd_velrl_pub.publish(Float32(data=velocity))
        self.cmd_velrr_pub.publish(Float32(data=velocity))

    def _publish_pwm(self, pwm: float) -> None:
        """Command the given PWM duty, with the sides opposed so the robot spins."""
        self.cmd_pwmfl_pub.publish(Float32(data=pwm))
        self.cmd_pwmfr_pub.publish(Float32(data=pwm))
        self.cmd_pwmrl_pub.publish(Float32(data=-pwm))
        self.cmd_pwmrr_pub.publish(Float32(data=-pwm))

    def stop_motors(self) -> None:
        self._publish_velocity(0.0)
        self._publish_pwm(0.0)

        spin_for(self.node, MOTOR_STOP_TIME)

    def check_motor_load(self) -> bool:
        speed_limit = 1.0
        motors_loaded = True

        try:
            for pwm in range(30):
                self._publish_pwm(float(pwm))

                spin_for(self.node, PWM_RAMP_STEP_TIME)

                if (
                    self.wheel_data.velocity[0] > speed_limit
                    and self.wheel_data.velocity[1] < -speed_limit
                    and self.wheel_data.velocity[2] > speed_limit
                    and self.wheel_data.velocity[3] < -speed_limit
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

                try:
                    for wheel_test in wheel_valid["tests"]:
                        self._publish_velocity(wheel_test["velocity"])
                        spin_for(self.node, wheel_test["time"])
                        samples = self._collect_wheel_samples(
                            WHEEL_SAMPLES, WHEEL_SAMPLE_TIMEOUT
                        )
                        self._collect_velocity_errors(wheel_test, samples, errors)
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

        Only the first failing step of a given wheel is kept, so that every
        faulty wheel gets reported exactly once.

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
                value for value in values if not speed_min < value < speed_max
            ]

            if invalid:
                worst = max(invalid, key=lambda value: abs(value - setpoint))
                errors[name] = (
                    f"{name} was out of range in {len(invalid)}/{len(values)} "
                    f"samples at a {setpoint:.2f} rad/s setpoint, worst "
                    f"{worst:.2f} rad/s against {speed_min:.2f}..{speed_max:.2f} rad/s"
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

                try:
                    for torque_test in torque_valid["tests"]:
                        self._publish_pwm(torque_test["pwm"])
                        spin_for(self.node, torque_test["time"])
                        samples = self._collect_wheel_samples(
                            WHEEL_SAMPLES, WHEEL_SAMPLE_TIMEOUT
                        )
                        self._collect_torque_errors(torque_test, samples, errors)
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

        Only the first failing step of a given wheel is kept, so that every
        faulty wheel gets reported exactly once.

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
                value for value in values if not torque_min <= value <= torque_max
            ]

            if invalid:
                worst = max(invalid, key=lambda value: abs(value - midpoint))
                errors[name] = (
                    f"{name} was out of range in {len(invalid)}/{len(values)} "
                    f"samples at a {torque_test['pwm']:.0f}% PWM duty, worst "
                    f"{worst:.3f} Nm against {torque_min:.3f}..{torque_max:.3f} Nm"
                )

    def _check_wheel_errors(self, errors: dict[str, str]) -> None:
        """
        Raise a single error describing every faulty wheel.

        Each description already starts with the name of the wheel it concerns.

        :param errors: Mapping of wheel name to its failure description
        :type errors: dict[str, str]
        :raises ValueError: If any wheel was recorded as faulty
        """
        if errors:
            msg = "; ".join(errors[name] for name in self.WHEEL_NAMES if name in errors)
            raise ValueError(msg)


def test_motors(
    mode: MotorTestMode = MotorTestMode.ALL,
    ros_args: Optional[list[str]] = None,
) -> int:
    """
    Run the motor tests, after asking the user to confirm.

    :param mode: Which of the tests to run
    :type mode: MotorTestMode
    :param ros_args: Arguments forwarded to rclpy, or None to use sys.argv
    :type ros_args: Optional[list[str]]
    :return: 0 if every check passed, 1 otherwise
    :rtype: int
    """
    _log.info("Starting motor tests.")

    with log_step("Initializing ROS node"):
        rclpy.init(args=ros_args)
        node = Node("leo_motor_tester")
        spin_for(node, NODE_DISCOVERY_TIME)

    tester: Optional[MotorTester] = None

    try:
        firmware_info = check_firmware_node(node)

        if firmware_info is None:
            return 1

        board_type, _ = firmware_info

        _log.warning(
            "The motors will spin during this procedure. "
            "Make sure the robot is placed on a stand or has enough free space "
            "around it, and keep clear of the wheels."
        )

        if not get_confirmation_prompt("Do you want to start the motor tests?"):
            _log.info("Motor tests cancelled by the user.")
            return 0

        with log_step("Initializing the motor tester"):
            tester = MotorTester(node)

        with log_step("Checking if the motors are loaded"):
            motors_loaded = tester.check_motor_load()

        _log.info(f"Motors are {'loaded' if motors_loaded else 'not loaded'}.")

        results: list[tuple[str, bool]] = []

        if mode in (MotorTestMode.ALL, MotorTestMode.ENCODER):
            results.append(("Wheel encoders", tester.test_encoder(motors_loaded)))

        if (
            mode in (MotorTestMode.ALL, MotorTestMode.TORQUE)
            and board_type == BoardType.LEOCORE
        ):
            results.append(("Torque sensors", tester.test_torque(motors_loaded)))

        if not results:
            _log.warning("No test was selected to run.")

        return report_results(_log, results)

    finally:
        if tester is not None:
            tester.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
