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

import sys
from enum import Enum
from typing import TypeVar, Optional

import yaml  # type: ignore
from whichcraft import which

# pylint: disable=invalid-name
T = TypeVar("T")


class CSIColor(str, Enum):
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    RESET = "\033[0m"


def is_tool(name: str) -> bool:
    """
    Check whether an executable exists on PATH.

    :param name: Name of the executable
    :type name: str
    :return: True if executable exists, False otherwise
    :rtype: bool
    """
    return which(name) is not None


def write_flush(msg: str):
    """Write a message to standard output and flush the buffer."""
    sys.stdout.write(msg)
    sys.stdout.flush()


def print_ok(msg: str):
    print(CSIColor.GREEN + msg + CSIColor.RESET)


def print_warn(msg: str):
    print(CSIColor.YELLOW + msg + CSIColor.RESET)


def print_fail(msg: str):
    print(CSIColor.RED + msg + CSIColor.RESET)


def print_test_result(res: tuple[bool, Optional[str]]):
    if res[0]:
        print_ok("PASSED")
    else:
        assert res[1] is not None
        print_fail("FAILED (" + res[1] + ")")


def query_yes_no(question: str, default: str = "yes") -> bool:
    """
    Ask a yes/no question via input() and return their answer.

    :param question: The question that is presented to the user.
    :type question: str
    :param default: The presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).
    :type default: str, optional
    :return: True if the answer is "yes", False otherwise
    :rtype: bool
    """
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError(f"invalid default answer: '{default}'")

    while True:
        write_flush(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        if choice in valid:
            return valid[choice]
        print("Please respond with 'yes' or 'no' (or 'y' or 'n').")


def prompt_options(options: list[tuple[str, T]], default: int = 1) -> T:
    for i, (name, _) in enumerate(options):
        print(f"{i+1}) {name}")

    while True:
        input_raw = input(f"Your selection [{default}]: ")
        if input_raw == "":
            selected_nr = default - 1
        else:
            selected_nr = int(input_raw) - 1
        if 0 <= selected_nr < len(options):
            _, selected = options[selected_nr]
            return selected
        print("Please select a valid option")


def parse_yaml(file_path: str):
    with open(file_path, "r", encoding="utf-8") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    return {}
