"""Launch managed dual-EKF GPS localization with configurable input topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("ekf_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom = LaunchConfiguration("odom_topic")
    imu = LaunchConfiguration("imu_topic")
    gps = LaunchConfiguration("gps_topic")

    common = [config, {"use_sim_time": use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument("ekf_config"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
        Node(
            package="robot_localization", executable="ekf_node",
            name="ekf_filter_node_odom", output="screen", parameters=common,
            remappings=[("odometry/filtered", "odometry/local"),
                        ("odom", odom), ("imu/data", imu)],
        ),
        Node(
            package="robot_localization", executable="ekf_node",
            name="ekf_filter_node_map", output="screen", parameters=common,
            remappings=[("odometry/filtered", "odometry/filtered"),
                        ("odom", odom), ("imu/data", imu)],
        ),
        Node(
            package="robot_localization", executable="navsat_transform_node",
            name="navsat_transform", output="screen", parameters=common,
            remappings=[("imu/data", imu), ("gps/fix", gps),
                        ("odometry/filtered", "odometry/filtered"),
                        ("odometry/gps", "odometry/gps")],
        ),
    ])
