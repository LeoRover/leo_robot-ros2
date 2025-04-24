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

    if node.get_namespace() + "firmware/get_board_type" in [
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

    if node.get_namespace() + "firmware/get_firmware_version" in [
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
