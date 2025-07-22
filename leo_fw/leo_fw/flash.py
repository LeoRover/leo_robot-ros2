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
import subprocess

from typing import Optional
from argparse import Namespace

from ament_index_python.packages import get_package_share_directory
from ros2cli.node.direct import DirectNode

from .utils import (
    is_tool,
    write_flush,
    query_yes_no,
    prompt_options,
    print_ok,
    print_warn,
    print_fail,
)
from .board import BoardType, determine_board, check_firmware_version
from .agent import agent_check_active, agent_start, agent_stop


def write_binary(binary_path: str, reset_high=False):
    base_cmd = f"stm32loader -c rpi -f F4 {'-R' if reset_high else ''}"

    print("--> Disabling flash read/write protections")
    subprocess.check_call(f"{base_cmd} -uW", shell=True)

    print("--> Erasing flash")
    subprocess.check_call(f"{base_cmd} -e", shell=True)

    print("--> Flashing firmware")
    subprocess.check_call(f"{base_cmd} -wv {binary_path}", shell=True)

    print("--> Flashing completed!")


# pylint: disable=too-many-branches,too-many-statements
def flash_firmware(
    firmware_path: Optional[str] = None,
    board_type: Optional[BoardType] = None,
    check_version: bool = True,
):
    write_flush("--> Checking if stm32loader is installed.. ")

    if is_tool("stm32loader"):
        print_ok("YES")
    else:
        print_fail("NO")
        print_warn(
            "ERROR: Cannot find the stm32loader tool. "
            "Make sure the python3-stm32loader package is installed."
        )
        return

    #####################################################

    write_flush("--> Checking if Micro-ROS Agent is running.. ")

    uros_agent_running = agent_check_active()
    if uros_agent_running:
        print_ok("YES")
    else:
        print_fail("NO")

    #####################################################

    if uros_agent_running and (board_type is None or check_version):
        write_flush("--> Initializing ROS node.. ")
        node_wrapper = DirectNode(
            Namespace(node_name_suffix="firmware_flasher", spin_time=3.0)
        )
        node = node_wrapper.node
        print_ok("DONE")

    #####################################################

    if uros_agent_running and (board_type is None or check_version):
        write_flush("--> Checking if firmware node is active.. ")

        if ("firmware", node.get_namespace()) in node.get_node_names_and_namespaces():
            print_ok("YES")
            firmware_node_active = True
        else:
            print_fail("NO")
            firmware_node_active = False
            if check_version:
                print_warn(
                    "Firmware node is not active. "
                    "Will not be able to check the board type or current firmware version."
                )
                if not query_yes_no("Are you sure you want to continue?", default="no"):
                    return

    #####################################################

    if uros_agent_running and board_type is None and firmware_node_active:
        write_flush("--> Trying to determine board type.. ")

        board_type = determine_board(node)

        if board_type is not None:
            print_ok("SUCCESS")
        else:
            print_fail("FAIL")

    #####################################################

    current_firmware_version = "<unknown>"

    if uros_agent_running and check_version and firmware_node_active:
        write_flush("--> Trying to check the current firmware version.. ")

        current_firmware_version = check_firmware_version(node)

        if current_firmware_version != "<unknown>":
            print_ok("SUCCESS")
        else:
            print_fail("FAIL")

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
            firmware_version = "1.3.0"
        elif board_type == BoardType.LEOCORE:
            firmware_version = "2.0.0"

    print(f"Current firmware version: {current_firmware_version}")
    print(f"Version of the firmware to flash: {firmware_version}")

    if not query_yes_no("Flash the firmware?"):
        return

    #####################################################

    if uros_agent_running:
        write_flush("--> Stopping the Micro-ROS Agent.. ")
        agent_stop()
        print_ok("DONE")

    #####################################################

    if board_type == BoardType.CORE2:
        if firmware_path is None:
            firmware_path = os.path.join(
                get_package_share_directory("leo_fw"),
                "data/firmware_binaries/core2_firmware.bin",
            )

        write_binary(firmware_path, reset_high=True)

    elif board_type == BoardType.LEOCORE:
        if firmware_path is None:
            firmware_path = os.path.join(
                get_package_share_directory("leo_fw"),
                "data/firmware_binaries/leocore_firmware.bin",
            )

        write_binary(firmware_path, reset_high=False)

    #####################################################

    if uros_agent_running:
        write_flush("--> Starting the Micro-ROS Agent.. ")
        agent_start()
        print_ok("DONE")
