import os

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def get_rpi_model():
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().strip()
        if "Raspberry Pi 5" in model:
            return 5
        elif "Raspberry Pi 4" in model:
            return 4
    except Exception as e:
        print(f"Failed to read Raspberry Pi model: {e}")
    return -1

def generate_launch_description():
    rpi_model = get_rpi_model()
    tuning_file = ""

    if rpi_model == 5:
        tuning_file = os.path.join(
            get_package_share_directory("leo_bringup"), "camera_tuning_files", "Arducam-477M-Pi5.json"
        )
    elif rpi_model == 4:
        #TODO: change this to use correct tuning file for Pi 4 camera
        tuning_file = os.path.join(
            get_package_share_directory("leo_bringup"), "camera_tuning_files", "Arducam-477M-Pi4.json"
        )

    config_path = os.path.join(
        get_package_share_directory("leo_bringup"), "config", "camera.yaml"
    )

    container = ComposableNodeContainer(
        name="image_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="camera",
                package="leo_camera_ros",
                plugin="camera::CameraNode",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                namespace="camera",
                name="debayer",
                package="image_proc",
                plugin="image_proc::DebayerNode",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                namespace="camera",
                name="rectify_mono",
                package="image_proc",
                plugin="image_proc::RectifyNode",
                extra_arguments=[{"use_intra_process_comms": True}],
                remappings=[("image", "image_mono")],
            ),
            ComposableNode(
                namespace="camera",
                name="rectify_color",
                package="image_proc",
                plugin="image_proc::RectifyNode",
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
    
    return LaunchDescription([
        SetEnvironmentVariable("LIBCAMERA_RPI_TUNING_FILE", tuning_file),
        container,
    ])
