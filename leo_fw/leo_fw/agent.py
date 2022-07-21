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
