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

from os import path
from typing import Optional

import yaml  # type: ignore

from ament_index_python import get_package_share_directory

from rcl_interfaces.msg import Parameter as ParameterMsg, SetParametersResult
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class ParameterBridge(Node):
    firmware_parameters: list[ParameterMsg] = []
    default_config: dict = {}
    override_config: dict = {}
    type_dict: dict = {
        str: Parameter.Type.STRING,
        int: Parameter.Type.INTEGER,
        bool: Parameter.Type.BOOL,
        float: Parameter.Type.DOUBLE,
    }

    def __init__(self, executor: MultiThreadedExecutor) -> None:
        super().__init__("firmware_parameter_bridge")
        self.executor = executor

        leo_fw_share = get_package_share_directory("leo_fw")

        self.declare_parameter(
            "default_params_file_path",
            path.join(leo_fw_share, "data", "default_firmware_params.yaml"),
        )
        self.declare_parameter("override_params_file_path", "")

        self.default_param_file: str = (
            self.get_parameter("default_params_file_path")
            .get_parameter_value()
            .string_value
        )
        self.override_param_file: str = (
            self.get_parameter("override_params_file_path")
            .get_parameter_value()
            .string_value
        )

        self.load_yaml_configs()
        self.parse_yaml_configs()

        cb_group = MutuallyExclusiveCallbackGroup()
        self.firmware_parameter_service_client = self.create_client(
            SetParameters,
            "firmware/set_parameters",
            callback_group=cb_group,
        )

        self.frimware_boot_service_client = self.create_client(
            Trigger,
            "firmware/boot",
            callback_group=cb_group,
        )

        self.param_bridge_srv = self.create_service(
            Trigger,
            "upload_params",
            self.param_bridge_srv_callback,
        )

        self.firmware_subscriber = self.create_subscription(
            Empty,
            "firmware/param_trigger",
            self.param_bridge_sub_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )

        self.get_logger().info("Starting node.")
        self.send_params(boot_firmware=False)

    def load_yaml_configs(self) -> None:
        if self.default_param_file == "":
            self.get_logger().error("Path to file with default parameters is empty!")
            raise UserWarning("Path to file with default parameters is empty!")

        with open(self.default_param_file, "r", encoding="utf-8") as file:
            try:
                self.default_config: dict = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)
                raise

        if self.override_param_file != "":
            with open(self.override_param_file, "r", encoding="utf-8") as file:
                try:
                    self.override_config: dict = yaml.safe_load(file)
                except yaml.YAMLError as exc:
                    self.get_logger().error(exc)
                    raise
        else:
            self.get_logger().warning("Path to file with override parameters is empty.")

    def parse_yaml_configs(
        self,
        param_name_prefix: str = "",
        default_dict: dict = None,
        override_dict: dict = None,
    ) -> None:
        if default_dict is None:
            default_dict = self.default_config
        if override_dict is None:
            override_dict = self.override_config

        default_keys = set(default_dict.keys())
        override_keys = set(override_dict.keys())

        for key in default_keys:
            if isinstance(default_dict[key], dict):
                new_name_prefix = param_name_prefix + key + "/"
                self.parse_yaml_configs(
                    new_name_prefix,
                    default_dict[key],
                    override_dict.get(key, {}),
                )
                continue

            if key in override_keys:
                value = override_dict[key]
            else:
                value = default_dict[key]

            new_param = rclpy.Parameter(
                param_name_prefix + key, self.type_dict[type(value)], value
            )
            self.firmware_parameters.append(new_param.to_parameter_msg())

    def param_bridge_sub_callback(self, msg: Empty) -> None:
        self.get_logger().info("Request for firmware parameters.")
        self.send_params()

    def param_bridge_srv_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("User request for loading parameters.")
        self.firmware_parameters = []

        try:
            self.load_yaml_configs()
            self.parse_yaml_configs()
        except (yaml.YAMLError, UserWarning) as e:
            response.success = False
            response.message = e
            return response

        result, num = self.send_params(boot_firmware=False)
        if result:
            response.message = "Successfully set firmware parameters."
            if num > 0:
                response.message += (
                    f" {num} parameter(s) was(were) not set. Check logs: `ros-logs`."
                )
            response.success = True
        else:
            response.message = "Failed to set firmware parameters."
            response.success = False

        return response

    def send_params(self, boot_firmware=True) -> tuple[bool, int]:
        not_set_params_num = 0
        if not self.firmware_parameter_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Firmware parameter service not active!")
            return (False, not_set_params_num)

        param_request = SetParameters.Request()
        param_request.parameters = self.firmware_parameters
        future = self.firmware_parameter_service_client.call_async(param_request)

        self.get_logger().info("Sending new parameters to firmware node.")

        self.executor.spin_until_future_complete(future, 5.0)

        if future.result():
            result: SetParametersResult
            param: ParameterMsg
            for result, param in zip(future.result().results, param_request.parameters):
                if not result.successful:
                    self.get_logger().warning(
                        f"Parameter '{param.name}' not set. Reason: '{result.reason}'"
                    )
                    not_set_params_num += 1

            self.get_logger().info("Successfully set parameters for firmware node.")
            if boot_firmware:
                return (self.trigger_boot(), not_set_params_num)

            return (True, not_set_params_num)

        self.get_logger().error("Unable to set parameters for firmware node.")
        return (False, not_set_params_num)

    def trigger_boot(self) -> bool:
        if not self.frimware_boot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Firmware boot service not active!")

        self.get_logger().info("Sending trigger for booting the firmware.")

        boot_request = Trigger.Request()
        boot_future = self.frimware_boot_service_client.call_async(boot_request)

        self.executor.spin_until_future_complete(boot_future, 5.0)

        if boot_future.result():
            self.get_logger().info("Triggering firmware boot successful.")
            return True

        self.get_logger().error("Didn't get response from firmware boot service!")
        return False
