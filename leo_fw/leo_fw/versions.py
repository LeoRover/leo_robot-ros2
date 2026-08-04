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

"""Utility functions for retrieving version information."""

from __future__ import annotations

import os
import re

from ament_index_python.packages import get_package_share_directory

from .board import BoardType

FIRMWARE_BINARIES = {
    BoardType.CORE2: "core2_firmware.bin",
    BoardType.LEOCORE: "leocore_firmware.bin",
}

_VERSION_PATTERN = re.compile(r"\d+\.\d+\.\d+")


def get_firmware_binary_path(board_type: BoardType) -> str:
    """
    Get the path to the firmware binary shipped for the given board.

    :param board_type: The board the firmware is built for
    :type board_type: BoardType
    :return: Path to the firmware binary
    :rtype: str
    """
    return os.path.join(
        get_package_share_directory("leo_fw"),
        "data",
        "firmware_binaries",
        FIRMWARE_BINARIES[board_type],
    )


def get_firmware_version(path: str, board_type: BoardType) -> str:
    """
    Get the firmware version from the specified binary.

    The firmware stores the board name and its version as adjacent
    null-terminated strings, so the board name is used as the marker.
    Scanning for the first version-looking string would not work, as the
    binaries contain unrelated version strings of the libraries they use.

    :param path: Path to the firmware binary
    :type path: str
    :param board_type: The board the firmware is built for
    :type board_type: BoardType
    :return: The human-readable version string
    :rtype: str
    :raises ValueError: If the marker is missing or is not followed by a version
    """
    with open(path, "rb") as binary:
        data = binary.read()

    marker = str(board_type).encode("ascii") + b"\x00"
    idx = data.find(marker)
    if idx == -1:
        msg = f"Board name marker '{board_type}' not found in {path}"
        raise ValueError(msg)

    start = idx + len(marker)
    end = data.find(b"\x00", start)
    if end == -1:
        msg = f"Firmware version string is not terminated in {path}"
        raise ValueError(msg)

    version = data[start:end].decode("ascii", errors="replace")
    if not _VERSION_PATTERN.fullmatch(version):
        msg = f"Marker '{board_type}' in {path} is not followed by a version, got '{version}'"
        raise ValueError(msg)

    return version
