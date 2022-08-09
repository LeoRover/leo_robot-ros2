from __future__ import annotations

from enum import Enum
from typing import Optional

import rclpy

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
        get_board_type = node.create_client(Trigger, "firmware/get_board_type")
        future = get_board_type.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)
        if future.done() and not future.exception():
            result = future.result()
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
        get_firmware_version = node.create_client(
            Trigger, "firmware/get_firmware_version"
        )
        future = get_firmware_version.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)
        if future.done() and not future.exception():
            result = future.result()
            firmware_version = result.message
        get_firmware_version.destroy()

    return firmware_version
