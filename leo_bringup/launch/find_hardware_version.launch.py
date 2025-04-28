from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration


def generate_launch_description():
    leo_hardware_version = 1

    try:
        with open("/proc/device-tree/model", "r", encoding="utf-8") as f:
            model = f.read().strip()
        if "Raspberry Pi 5" in model:
            leo_hardware_version = 2
        elif "Raspberry Pi 4" in model:
            leo_hardware_version = 1
    except (FileNotFoundError, IOError) as e:
        print(f"Failed to read Raspberry Pi model: {e}")

    return LaunchDescription(
        [
            SetLaunchConfiguration(
                name="leo_hardware_version", value=str(leo_hardware_version)
            ),
        ]
    )
