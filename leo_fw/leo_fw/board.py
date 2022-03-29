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

    print(services)

    board_type = None

    if node.get_namespace() + "firmware/get_board_type" in services[:, 0]:
        get_board_type = node.create_client(Trigger, "firmware/get_board_type")
        type_str = get_board_type.call(Trigger.Request()).result().message
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

    if node.get_namepace() + "firmware/get_firmware_version" in services[:, 0]:
        get_firmware_version = node.create_client(
            Trigger, "firmware/get_firmware_version"
        )
        firmware_version = get_firmware_version.call(Trigger.Request()).result().message
        get_firmware_version.destroy()

    return firmware_version
