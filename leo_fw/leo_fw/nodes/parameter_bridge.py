# Copyright 2023-2026 Fictionlab sp. z o.o.
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

from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    SetParametersResult,
    ParameterDescriptor,
)
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.timer import Timer
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

    def __init__(self) -> None:
        super().__init__("firmware_parameter_bridge")

        leo_fw_share = get_package_share_directory("leo_fw")

        self.declare_parameter(
            "default_params_file_path",
            path.join(leo_fw_share, "data", "default_firmware_params.yaml"),
            ParameterDescriptor(read_only=True),
        )

        self.declare_parameter(
            "leo_hardware_version", 2, ParameterDescriptor(read_only=True)
        )

        self.load_default_params()

        self.params_dict = self.parse_default_params()
        self.declare_firmware_parameters()

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
            "~/upload_params",
            self.upload_params_callback,  # type: ignore[arg-type]
        )

        self.firmware_subscriber = self.create_subscription(
            Empty,
            "firmware/param_trigger",
            self.param_trigger_callback,  # type: ignore[arg-type]
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )

        self.params_retry_timer: Timer | None = self.create_timer(
            2.0,
            self.try_send_params,  # type: ignore[arg-type]
        )

        self.add_post_set_parameters_callback(self.post_set_parameters_callback)

    async def try_send_params(self) -> None:
        success, _ = await self.send_all_params()
        if success and self.params_retry_timer is not None:
            self.params_retry_timer.destroy()
            self.params_retry_timer = None
            self.get_logger().info(
                "Firmware parameters uploaded successfully. Retry timer stopped."
            )

    def load_default_params(self) -> None:
        default_params_file: str = (
            self.get_parameter("default_params_file_path")
            .get_parameter_value()
            .string_value
        )

        with open(default_params_file, "r", encoding="utf-8") as file:
            self.default_params: dict = yaml.safe_load(file)

    def parse_default_params(self) -> dict[str, Parameter]:
        def parse_parameters_recursive(
            parameters: dict[str, Parameter],
            param_name_prefix: str,
            default_dict: dict,
        ) -> None:
            for key, value in default_dict.items():
                if isinstance(value, dict):
                    new_name_prefix = param_name_prefix + key + "."
                    parse_parameters_recursive(
                        parameters,
                        new_name_prefix,
                        value,
                    )
                    continue

                new_param = rclpy.Parameter(
                    param_name_prefix + key, self.type_dict[type(value)], value
                )
                parameters[new_param.name] = new_param

        parameters: dict[str, Parameter] = {}
        parse_parameters_recursive(
            parameters,
            "",
            self.default_params,
        )
        return parameters

    def declare_firmware_parameters(self) -> None:
        for param in self.params_dict.values():
            self.declare_parameter(param.name, param.value)

        for param_name in self.params_dict:
            param = self.get_parameter(param_name)
            self.params_dict.update({param_name: param})

    def post_set_parameters_callback(self, params: list[Parameter]) -> None:
        new_firmware_params: list[Parameter] = []

        for param in params:
            if param.name in self.params_dict:
                self.params_dict[param.name] = param
                new_firmware_params.append(param)
                self.get_logger().info(
                    f"Parameter '{param.name}' updated to: {param.value}"
                )

        if new_firmware_params:
            assert self.executor is not None
            self.executor.create_task(self.send_new_params, new_firmware_params)

    async def param_trigger_callback(self, _msg: Empty) -> None:
        self.get_logger().info("Request for firmware parameters.")
        success, _ = await self.send_all_params()
        if success:
            await self.trigger_boot()

    async def upload_params_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info(
            "Serving user request for setting firmware parameters..."
        )

        result, num = await self.send_all_params()
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
            self.get_logger().error(response.message)

        return response

    async def send_all_params(self) -> tuple[bool, int]:
        if not self.firmware_parameter_service_client.service_is_ready():
            self.get_logger().info("Firmware parameter service not ready.")
            return (False, 0)

        self.get_logger().info("Trying to set parameters for firmware node...")

        all_params = [param.to_parameter_msg() for param in self.params_dict.values()]
        all_params.append(self.get_parameter("leo_hardware_version").to_parameter_msg())

        not_set_params_num = 0
        for param in all_params:
            try:
                if not await self.send_param(Parameter.from_parameter_msg(param)):
                    not_set_params_num += 1
            except RuntimeError as e:
                self.get_logger().error(str(e))
                return (False, not_set_params_num)

        self.get_logger().info("Successfully set parameters for firmware node.")
        return (True, not_set_params_num)

    async def send_new_params(self, params: list[Parameter]) -> None:
        for param in params:
            try:
                await self.send_param(param)
            except RuntimeError as e:
                self.get_logger().error(str(e))
                return

    async def send_param(self, param: Parameter) -> bool:
        if not self.firmware_parameter_service_client.service_is_ready():
            self.get_logger().info("Firmware parameter service not ready.")
            return False

        param_request = SetParameters.Request()
        param_request.parameters = [param.to_parameter_msg()]

        future = self.firmware_parameter_service_client.call_async(param_request)

        cancel_timer = self.create_timer(
            5.0,
            lambda: None if future.done() else future.set_result(None),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        await future

        cancel_timer.destroy()

        set_params_response: SetParameters.Response | None = future.result()
        if set_params_response is not None:
            result: SetParametersResult = set_params_response.results[0]
            if result.successful:
                return True
            self.get_logger().warning(
                f"Parameter '{param.name}' not set. Reason: '{result.reason}'"
            )
            return False

        raise RuntimeError("Didn't get response from firmware parameter service!")

    async def trigger_boot(self) -> bool:
        self.get_logger().info("Trying to trigger firmware boot.")

        if not self.firmware_boot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Firmware boot service not active!")
            return False

        boot_request = Trigger.Request()
        boot_future = self.firmware_boot_service_client.call_async(boot_request)

        cancel_timer = self.create_timer(
            5.0,
            lambda: None if boot_future.done() else boot_future.set_result(None),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        await boot_future

        cancel_timer.destroy()

        if boot_future.result():
            self.get_logger().info("Firmware boot triggered successfully.")
            if self.params_retry_timer is not None:
                self.params_retry_timer.destroy()
                self.params_retry_timer = None
            return True

        self.get_logger().error("Didn't get response from firmware boot service!")
        return False
