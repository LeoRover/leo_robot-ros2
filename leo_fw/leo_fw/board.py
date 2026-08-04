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

from __future__ import annotations

from enum import Enum
from typing import Optional

import rclpy
from rclpy.client import Client

from std_srvs.srv import Trigger

from .console import get_logger, log_step

_log = get_logger("board")


class BoardType(Enum):
    LEOCORE = "leocore"
    CORE2 = "core2"

    def __str__(self):
        return self.value


def determine_board(node: rclpy.Node) -> Optional[BoardType]:
    services = node.get_service_names_and_types_by_node(
        "firmware", node.get_namespace()
    )

    board_type = None

    if node.resolve_service_name("firmware/get_board_type") in [
        service[0] for service in services
    ]:
        get_board_type: Client = node.create_client(Trigger, "firmware/get_board_type")
        future = get_board_type.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.done() and not future.exception():
            result: Trigger.Response | None = future.result()
            assert result is not None
            type_str = result.message
            if type_str == str(BoardType.CORE2):
                board_type = BoardType.CORE2
            elif type_str == str(BoardType.LEOCORE):
                board_type = BoardType.LEOCORE
        get_board_type.destroy()
    return board_type


def check_firmware_version(node: rclpy.Node) -> str:
    services = node.get_service_names_and_types_by_node(
        "firmware", node.get_namespace()
    )

    firmware_version = "<unknown>"

    if node.resolve_service_name("firmware/get_firmware_version") in [
        service[0] for service in services
    ]:
        get_firmware_version: Client = node.create_client(
            Trigger, "firmware/get_firmware_version"
        )
        future = get_firmware_version.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.done() and not future.exception():
            result: Trigger.Response | None = future.result()
            assert result is not None
            firmware_version = result.message
        get_firmware_version.destroy()

    return firmware_version


def check_firmware_node(node: rclpy.Node) -> Optional[tuple[BoardType, str]]:
    """
    Verify that the firmware node is running and report what it runs.

    Logs the board type and the firmware version the node reports.

    :param node: Node used to query the ROS graph
    :type node: rclpy.Node
    :return: The board type and the reported firmware version, or None if the
        firmware node is not usable
    :rtype: Optional[tuple[BoardType, str]]
    """
    try:
        with log_step("Checking if firmware node is active"):
            if (
                "firmware",
                node.get_namespace(),
            ) not in node.get_node_names_and_namespaces():
                msg = (
                    "Firmware node is not active. "
                    "Try to flash the firmware or restart the Micro-ROS Agent."
                )
                raise ValueError(msg)

        with log_step("Determining the board type"):
            board_type = determine_board(node)
            if board_type is None:
                msg = (
                    "Can not determine board type. "
                    "Update the firmware and try to rerun the script."
                )
                raise ValueError(msg)
    except ValueError as exc:
        _log.error("Will not be able to validate hardware: %s", exc)
        return None

    with log_step("Reading the current firmware version"):
        current_firmware_version = check_firmware_version(node)

    if current_firmware_version == "<unknown>":
        _log.warning("Could not read the firmware version.")

    if board_type == BoardType.CORE2:
        _log.info("Board type: Husarion CORE2")
    elif board_type == BoardType.LEOCORE:
        _log.info("Board type: LeoCore")
    _log.info(f"Firmware version: {current_firmware_version}")

    return board_type, current_firmware_version
