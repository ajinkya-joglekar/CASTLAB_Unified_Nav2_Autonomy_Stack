"""Gazebo Harmonic simulation for a registry-resolved vehicle and world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("base_nav2_stack")
    world_file = LaunchConfiguration("world_file")
    world_name = LaunchConfiguration("world_name")
    urdf = LaunchConfiguration("urdf")
    model_name = LaunchConfiguration("model_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    vehicle_cmd_vel_topic = LaunchConfiguration("vehicle_cmd_vel_topic")
    vehicle_odom_topic = LaunchConfiguration("vehicle_odom_topic")
    vehicle_imu_topic = LaunchConfiguration("vehicle_imu_topic")
    vehicle_gps_topic = LaunchConfiguration("vehicle_gps_topic")
    vehicle_scan_topic = LaunchConfiguration("vehicle_scan_topic")
    vehicle_points_topic = LaunchConfiguration("vehicle_points_topic")
    vehicle_camera_image_topic = LaunchConfiguration("vehicle_camera_image_topic")
    vehicle_camera_info_topic = LaunchConfiguration("vehicle_camera_info_topic")
    vehicle_camera_depth_image_topic = LaunchConfiguration("vehicle_camera_depth_image_topic")
    clock_bridge = PythonExpression([
        "'/world/' + '", world_name,
        "' + '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'",
    ])
    clock_topic = PythonExpression(["'/world/' + '", world_name, "' + '/clock'"])
    cmd_vel_bridge = PythonExpression(["'", vehicle_cmd_vel_topic, "' + '@geometry_msgs/msg/Twist@gz.msgs.Twist'"])
    odom_bridge = PythonExpression(["'", vehicle_odom_topic, "' + '@nav_msgs/msg/Odometry@gz.msgs.Odometry'"])
    scan_bridge = PythonExpression(["'", vehicle_scan_topic, "' + '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'"])
    points_bridge = PythonExpression(["'", vehicle_points_topic, "' + '@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'"])
    imu_bridge = PythonExpression(["'", vehicle_imu_topic, "' + '@sensor_msgs/msg/Imu@gz.msgs.IMU'"])
    gps_bridge = PythonExpression(["'", vehicle_gps_topic, "' + '@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat'"])
    camera_image_bridge = PythonExpression(["'", vehicle_camera_image_topic, "' + '@sensor_msgs/msg/Image@gz.msgs.Image'"])
    camera_info_bridge = PythonExpression(["'", vehicle_camera_info_topic, "' + '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'"])
    camera_depth_image_bridge = PythonExpression(["'", vehicle_camera_depth_image_topic, "' + '@sensor_msgs/msg/Image@gz.msgs.Image'"])

    return LaunchDescription([
        DeclareLaunchArgument("world_file"),
        DeclareLaunchArgument("world_name"),
        DeclareLaunchArgument("urdf"),
        DeclareLaunchArgument("model_name", default_value="ranger"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("vehicle_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("vehicle_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("vehicle_imu_topic", default_value="/imu"),
        DeclareLaunchArgument("vehicle_gps_topic", default_value="/gps/fix"),
        DeclareLaunchArgument("vehicle_scan_topic", default_value="/lidar"),
        DeclareLaunchArgument("vehicle_points_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("vehicle_camera_image_topic", default_value="/camera/image"),
        DeclareLaunchArgument("vehicle_camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("vehicle_camera_depth_image_topic", default_value="/camera/depth_image"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join([os.path.join(share, "worlds"), os.path.join(share, "worlds","baylands_world"), share,
                             os.path.join(share, "meshes")]),
        ),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_file], output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": "-v4 -g"}.items(),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            output="screen", parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str),
            }],
        ),
        TimerAction(period=2.0, actions=[Node(
            package="ros_gz_sim", executable="create", name="spawn_vehicle", output="screen",
            arguments=["-name", model_name, "-topic", "robot_description",
                       "-x", LaunchConfiguration("spawn_x"),
                       "-y", LaunchConfiguration("spawn_y"),
                       "-z", LaunchConfiguration("spawn_z"),
                       "-Y", LaunchConfiguration("spawn_yaw")],
        )]),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="ros_gz_bridge",
            output="screen",
            arguments=[clock_bridge,
                cmd_vel_bridge,
                odom_bridge,
                scan_bridge,
                points_bridge,
                imu_bridge,
                "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                gps_bridge,
                camera_image_bridge,
                camera_info_bridge,
                camera_depth_image_bridge],
            remappings=[(clock_topic, "/clock"),
                        (vehicle_cmd_vel_topic, "/cmd_vel"),
                        (vehicle_odom_topic, "/odom"),
                        (vehicle_scan_topic, "/scan"),
                        (vehicle_points_topic, "/points"),
                        (vehicle_imu_topic, "/vectornav/imu/data"),
                        (vehicle_gps_topic, "/gps/fix"),
                        (vehicle_camera_image_topic, "/camera/image_raw"),
                        (vehicle_camera_info_topic, "/camera/depth/camera_info"),
                        (vehicle_camera_depth_image_topic, "/camera/depth/image_raw")],
        ),
    ])
