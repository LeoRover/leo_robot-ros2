# Copyright 2022 Kell Ideas sp. z o.o.
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

import dbus

AGENT_UNIT_NAME = "uros-agent.service"


def _get_unit_interfaces() -> tuple[dbus.Interface, dbus.Interface]:
    bus = dbus.SessionBus()
    manager_proxy = bus.get_object(
        "org.freedesktop.systemd1", "/org/freedesktop/systemd1"
    )
    manager_interface = dbus.Interface(
        manager_proxy, "org.freedesktop.systemd1.Manager"
    )
    unit_path = manager_interface.GetUnit(AGENT_UNIT_NAME)
    unit_proxy = bus.get_object("org.freedesktop.systemd1", unit_path)
    unit_interface = dbus.Interface(unit_proxy, "org.freedesktop.systemd1.Unit")
    unit_properties_interface = dbus.Interface(
        unit_proxy, "org.freedesktop.DBus.Properties"
    )
    return unit_interface, unit_properties_interface


def agent_check_active() -> bool:
    _, unit_properties_interface = _get_unit_interfaces()
    state: dbus.String = unit_properties_interface.Get(
        "org.freedesktop.systemd1.Unit", "ActiveState"
    )
    return state == "active"


def agent_start():
    unit_interface, _ = _get_unit_interfaces()
    unit_interface.Start("fail")


def agent_stop():
    unit_interface, _ = _get_unit_interfaces()
    unit_interface.Stop("fail")
