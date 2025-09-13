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
from rclpy.client import Client

from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class ParameterBridge(Node):
    firmware_parameters: list[ParameterMsg] = []
    default_params: dict = {}
    override_params: dict = {}
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

        self.declare_parameter("leo_hardware_version", 109)

        self.load_default_params()
        self.load_override_params()

        cb_group = MutuallyExclusiveCallbackGroup()
        self.firmware_parameter_service_client: Client = self.create_client(
            SetParameters,
            "firmware/set_parameters",
            callback_group=cb_group,
        )

        self.firmware_boot_service_client: Client = self.create_client(
            Trigger,
            "firmware/boot",
            callback_group=cb_group,
        )

        self.param_bridge_srv = self.create_service(
            Trigger,
            "upload_params",
            self.upload_params_callback,
        )

        self.firmware_subscriber = self.create_subscription(
            Empty,
            "firmware/param_trigger",
            self.param_trigger_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )

        self.send_params()

    def load_default_params(self) -> None:
        default_params_file: str = (
            self.get_parameter("default_params_file_path")
            .get_parameter_value()
            .string_value
        )

        with open(default_params_file, "r", encoding="utf-8") as file:
            self.default_params: dict = yaml.safe_load(file)

    def load_override_params(self) -> None:
        override_params_file: str = (
            self.get_parameter("override_params_file_path")
            .get_parameter_value()
            .string_value
        )

        self.override_params = {}

        if override_params_file != "":
            try:
                with open(override_params_file, "r", encoding="utf-8") as file:
                    override_yaml = yaml.safe_load(file)
                    if isinstance(override_yaml, dict):
                        self.override_params = override_yaml
            except (FileNotFoundError, PermissionError, yaml.YAMLError) as exc:
                self.get_logger().error("Failed to load parameter overrides!")
                self.get_logger().error(str(exc))
        else:
            self.get_logger().warning("Path to file with override parameters is empty.")

    def parse_firmware_parameters(self) -> list[ParameterMsg]:
        def parse_parameters_recursive(
            parameters: list[ParameterMsg],
            param_name_prefix: str,
            default_dict: dict,
            override_dict: dict,
        ) -> None:
            for key, value in default_dict.items():
                if isinstance(value, dict):
                    new_name_prefix = param_name_prefix + key + "/"
                    parse_parameters_recursive(
                        parameters,
                        new_name_prefix,
                        value,
                        override_dict.get(key, {}),
                    )
                    continue

                if key in override_dict:
                    value = override_dict[key]

                new_param = rclpy.Parameter(
                    param_name_prefix + key, self.type_dict[type(value)], value
                )
                parameters.append(new_param.to_parameter_msg())

        parameters: list[ParameterMsg] = []
        parse_parameters_recursive(
            parameters, "", self.default_params, self.override_params
        )
        return parameters

    def param_trigger_callback(self, _msg: Empty) -> None:
        self.get_logger().info("Request for firmware parameters.")
        success, _ = self.send_params()
        if success:
            self.trigger_boot()

    def upload_params_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info(
            "Serving user request for setting firmware parameters..."
        )

        self.load_override_params()

        result, num = self.send_params()
        if result:
            response.message = "Successfully set firmware parameters."
            if num > 0:
                response.message += (
                    f" {num} parameter(s) was(were) not set. Check node logs."
                )
            response.success = True
        else:
            response.message = "Failed to set firmware parameters."
            response.success = False

        return response

    def send_params(self) -> tuple[bool, int]:
        self.get_logger().info("Trying to set parameters for firmware node...")

        not_set_params_num = 0
        if not self.firmware_parameter_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Firmware parameter service not active!")
            return (False, not_set_params_num)

        param_request = SetParameters.Request()
        param_request.parameters = self.parse_firmware_parameters()

        param_request.parameters.append(
            self.get_parameter("leo_hardware_version").to_parameter_msg()
        )

        future = self.firmware_parameter_service_client.call_async(param_request)

        assert self.executor is not None
        self.executor.spin_until_future_complete(future, 5.0)

        set_params_response: SetParameters.Response | None = future.result()
        if set_params_response is not None:
            result: SetParametersResult
            param: ParameterMsg
            for result, param in zip(
                set_params_response.results, param_request.parameters
            ):
                if not result.successful:
                    self.get_logger().warning(
                        f"Parameter '{param.name}' not set. Reason: '{result.reason}'"
                    )
                    not_set_params_num += 1

            self.get_logger().info("Successfully set parameters for firmware node.")

            return (True, not_set_params_num)

        self.get_logger().error("Unable to set parameters for firmware node.")
        return (False, not_set_params_num)

    def trigger_boot(self) -> bool:
        self.get_logger().info("Trying to trigger firmware boot.")

        if not self.firmware_boot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Firmware boot service not active!")

        boot_request = Trigger.Request()
        boot_future = self.firmware_boot_service_client.call_async(boot_request)

        assert self.executor is not None
        self.executor.spin_until_future_complete(boot_future, 5.0)

        if boot_future.result():
            self.get_logger().info("Firmware boot triggered successfully.")
            return True

        self.get_logger().error("Didn't get response from firmware boot service!")
        return False
