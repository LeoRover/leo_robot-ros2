import os
import subprocess

from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory

from .utils import is_tool, write_flush, query_yes_no, prompt_options
from .board import BoardType, determine_board, check_firmware_version


def flash_core2(bootloader_path: str, firmware_path: str):
    print("--> Disabling read/write protections and erasing flash")
    subprocess.check_call("stm32loader -c rpi -f F4 -R -u -W", shell=True)

    print("--> Erasing flash")
    subprocess.check_call("stm32loader -c rpi -f F4 -R -e", shell=True)

    print("--> Flashing bootloader")
    subprocess.check_call(
        f"stm32loader -c rpi -f F4 -R -w -v {bootloader_path}", shell=True
    )

    print("--> Flashing firmware")
    subprocess.check_call(
        f"stm32loader -c rpi -f F4 -R -a 0x08010000 -w -v {firmware_path}", shell=True
    )

    print("--> Flashing completed!")


def flash_leocore(firmware_path: str):
    print("--> Disabling flash read/write protections")
    subprocess.check_call("stm32loader -c rpi -f F4 -u -W", shell=True)

    print("--> Erasing flash")
    subprocess.check_call("stm32loader -c rpi -f F4 -e", shell=True)

    print("--> Flashing firmware")
    subprocess.check_call(f"stm32loader -c rpi -f F4 -w -v {firmware_path}", shell=True)

    print("--> Flashing completed!")


# pylint: disable=too-many-branches,too-many-statements
def flash_firmware(
    firmware_path: Optional[str] = None,
    board_type: Optional[BoardType] = None,
    check_version: bool = True,
):
    write_flush("--> Checking if stm32loader is installed.. ")

    if is_tool("stm32loader"):
        print("YES")
    else:
        print("NO")
        print(
            "ERROR: Cannot find the stm32loader tool. "
            "Make sure the python3-stm32loader package is installed."
        )
        return

    #####################################################

    write_flush("--> Initializing ROS node.. ")
    node = rclpy.create_node("firmware_flasher")
    print("DONE")

    #####################################################

    write_flush("--> Checking if Micro-ROS Agent is running.. ")

    # TODO Check if Micro-ROS agent is running

    print("YES")
    uros_agent_running = True

    #####################################################

    if uros_agent_running:
        write_flush("--> Checking if firmware node is active.. ")

        if node.get_namespace() + "firmware" in node.get_node_names():
            print("YES")
            firmware_node_active = True
        else:
            print("NO")
            firmware_node_active = False
            if check_version:
                print(
                    "Firmware node is not active. "
                    "Will not be able to check the current firmware version."
                )
                if not query_yes_no("Are you sure you want to continue?", default="no"):
                    return

    #####################################################

    if uros_agent_running and firmware_node_active and board_type is None:
        write_flush("--> Trying to determine board type.. ")

        board_type = determine_board(node)

        if board_type is not None:
            print("SUCCESS")
        else:
            print("FAIL")

    #####################################################

    current_firmware_version = "<unknown>"

    if uros_agent_running and firmware_node_active and check_version:
        write_flush("--> Trying to check the current firmware version.. ")

        current_firmware_version = check_firmware_version(node)

        if current_firmware_version != "<unknown>":
            print("SUCCESS")
        else:
            print("FAIL")

    #####################################################

    if board_type is None:
        print("Was not able to determine the board type. Choose the board manually: ")

        board_type = prompt_options(
            [
                ("LeoCore", BoardType.LEOCORE),
                ("Husarion CORE2", BoardType.CORE2),
            ]
        )

    #####################################################

    if firmware_path is not None:
        firmware_version = "<unknown>"
    else:
        if board_type == BoardType.CORE2:
            firmware_version = "0.0.0"
        elif board_type == BoardType.LEOCORE:
            firmware_version = "0.0.0"

    print(f"Current firmware version: {current_firmware_version}")
    print(f"Version of the firmware to flash: {firmware_version}")

    if not query_yes_no("Flash the firmware?"):
        return

    #####################################################

    if uros_agent_running:
        write_flush("--> Stopping the Micro-ROS Agent.. ")
        # TODO Stop micro-ROS agent
        print("DONE")

    #####################################################

    if board_type == BoardType.CORE2:
        bootloader_path = os.path.join(
            get_package_share_directory("leo_fw"),
            "firmware/bootloader_1_0_0_core2.bin",
        )

        if firmware_path is None:
            firmware_path = os.path.join(
                get_package_share_directory("leo_fw"),
                "firmware/core2_firmware.bin",
            )

        flash_core2(bootloader_path, firmware_path)

    elif board_type == BoardType.LEOCORE:
        if firmware_path is None:
            firmware_path = os.path.join(
                get_package_share_directory("leo_fw"),
                "firmware/leocore_firmware.bin",
            )

        flash_leocore(firmware_path)

    #####################################################

    if uros_agent_running:
        write_flush("--> Starting the Micro-ROS Agent.. ")
        # TODO Start Micro-ROS Agent
        print("DONE")
