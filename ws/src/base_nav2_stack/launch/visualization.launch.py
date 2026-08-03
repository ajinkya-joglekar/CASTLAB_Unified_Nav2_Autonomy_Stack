"""Optional RViz wrapper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz_config"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", LaunchConfiguration("rviz_config")],
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
