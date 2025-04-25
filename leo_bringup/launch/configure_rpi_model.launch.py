from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration


def generate_launch_description():
    rpi_model = 5

    try:
        with open("/proc/device-tree/model", "r", encoding="utf-8") as f:
            model = f.read().strip()
        if "Raspberry Pi 5" in model:
            rpi_model = 5
        elif "Raspberry Pi 4" in model:
            rpi_model = 4
    except (FileNotFoundError, IOError) as e:
        print(f"Failed to read Raspberry Pi model: {e}")

    return LaunchDescription(
        [
            SetLaunchConfiguration(name="rpi_model", value=str(rpi_model)),
        ]
    )
