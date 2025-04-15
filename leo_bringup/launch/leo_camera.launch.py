import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def get_rpi_model():
    try:
        with open("/proc/device-tree/model", "r", encoding="utf-8") as f:
            model = f.read().strip()
        if "Raspberry Pi 5" in model:
            return 5
        if "Raspberry Pi 4" in model:
            return 4
    except (FileNotFoundError, IOError) as e:
        print(f"Failed to read Raspberry Pi model: {e}")
    return -1


def generate_launch_description():
    rpi_model = get_rpi_model()
    tuning_file = ""
    config_path = ""

    if rpi_model == 5:
        tuning_file = os.path.join(
            get_package_share_directory("leo_bringup"),
            "camera_tuning_files",
            "Arducam-477M-Pi5.json",
        )

        config_path = os.path.join(
            get_package_share_directory("leo_bringup"),
            "config",
            "leo_camera_imx477.yaml",
        )

    if rpi_model == 4:
        config_path = os.path.join(
            get_package_share_directory("leo_bringup"),
            "config",
            "leo_camera_ov5647.yaml",
        )

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[
            ComposableNode(
                name="camera",
                package="leo_camera",
                plugin="leo_camera::CameraNode",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                namespace="camera",
                name="debayer",
                package="image_proc",
                plugin="image_proc::DebayerNode",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                namespace="camera",
                name="rectify_mono",
                package="image_proc",
                plugin="image_proc::RectifyNode",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("image", "image_mono")],
            ),
            ComposableNode(
                namespace="camera",
                name="rectify_color",
                package="image_proc",
                plugin="image_proc::RectifyNode",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[
                    ("image", "image_color"),
                    ("image_rect", "image_rect_color"),
                    ("image_rect/compressed", "image_rect_color/compressed"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("LIBCAMERA_RPI_TUNING_FILE", tuning_file),
            container,
        ]
    )
