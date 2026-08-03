"""Generate an organized RGB-D point cloud from depth image and camera info."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    points_topic = LaunchConfiguration("points_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("depth_image_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("depth_camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("points_topic", default_value="/camera/depth/points"),
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyz_node",
            name="depth_to_pointcloud",
            output="screen",
            remappings=[
                ("image_rect", depth_image_topic),
                ("camera_info", depth_camera_info_topic),
                ("points", points_topic),
            ],
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "lazy": False,
                "depth_image_transport": "raw",
                "queue_size": 10,
            }],
        ),
    ])
