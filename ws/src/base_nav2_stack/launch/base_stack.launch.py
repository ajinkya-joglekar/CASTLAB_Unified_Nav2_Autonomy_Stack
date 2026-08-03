"""Unified registry-driven entry point for simulation and hardware."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from base_nav2_stack.profiles import resolve_asset, validate_registries


def _truth(value):
    return value.strip().lower() in ("true", "1", "yes", "on")


def _python_bool(value):
    return "True" if _truth(value) else "False"


def _launch(context):
    share = Path(get_package_share_directory("base_nav2_stack"))
    launch_dir = share / "launch"
    vehicles, worlds = validate_registries(share)
    mode = LaunchConfiguration("mode").perform(context)
    launch_stage = LaunchConfiguration("launch_stage").perform(context)
    vehicle_name = LaunchConfiguration("vehicle").perform(context)
    world_profile = LaunchConfiguration("world").perform(context)
    world_name_override = LaunchConfiguration("world_name").perform(context).strip()
    localization_mode = LaunchConfiguration("localization_mode").perform(context)
    if mode not in ("sim", "hardware"):
        raise ValueError("mode must be 'sim' or 'hardware'")
    if launch_stage not in ("all", "sim", "nav"):
        raise ValueError("launch_stage must be 'all', 'sim', or 'nav'")
    if launch_stage == "sim" and mode != "sim":
        raise ValueError("launch_stage 'sim' requires mode 'sim'")
    if localization_mode not in ("managed", "external"):
        raise ValueError("localization_mode must be 'managed' or 'external'")
    if vehicle_name not in vehicles:
        raise ValueError(f"unknown vehicle '{vehicle_name}'; available: {sorted(vehicles)}")
    vehicle = vehicles[vehicle_name]

    def chosen(argument, default):
        override = LaunchConfiguration(argument).perform(context).strip()
        if override:
            path = Path(override).expanduser()
            if not path.is_absolute() and not path.is_file():
                path = share / path
        else:
            path = resolve_asset(share, default)
        if not path.is_file():
            raise FileNotFoundError(path)
        return str(path)

    nav2_config = chosen("nav2_config", vehicle["nav2_config"])
    ekf_config = chosen("ekf_config", vehicle["ekf_config"])
    topics = vehicle["topics"]

    def topic(argument, default):
        override = LaunchConfiguration(argument).perform(context).strip()
        return override or default

    cmd_vel_topic = topic("cmd_vel_topic", topics["cmd_vel"])
    odom_topic = topic("odom_topic", topics["odom"])
    filtered_odom_topic = topic("filtered_odom_topic", topics["filtered_odom"])
    imu_topic = topic("imu_topic", topics["imu"])
    gps_topic = topic("gps_topic", topics["gps"])
    scan_topic = topic("scan_topic", topics["scan"])
    points_topic = topic("points_topic", topics["points"])
    camera_image_topic = topic("camera_image_topic", topics.get("camera_image", ""))
    camera_info_topic = topic("camera_info_topic", topics.get("camera_info", ""))
    camera_depth_image_topic = topic("camera_depth_image_topic", topics.get("camera_depth_image", ""))
    camera_points_topic = topic("camera_points_topic", topics.get("camera_points", ""))
    use_perception = _truth(LaunchConfiguration("use_perception").perform(context))
    use_depth_image_proc = _truth(LaunchConfiguration("use_depth_image_proc").perform(context))
    use_sim_time = "true" if mode == "sim" else "false"
    autostart = _python_bool(LaunchConfiguration("autostart").perform(context))
    use_composition = _python_bool(LaunchConfiguration("use_composition").perform(context))
    use_respawn = _python_bool(LaunchConfiguration("use_respawn").perform(context))
    actions = []

    def relay_name(prefix, source, target):
        cleaned = prefix + "_" + source + "_to_" + target
        return "".join(char if char.isalnum() else "_" for char in cleaned).strip("_")

    def add_topic_relay(source, target, message_type, prefix, input_reliability="best_effort"):
        if not source or not target or source == target:
            return
        actions.append(Node(
            package="base_nav2_stack",
            executable="topic_relay",
            name=relay_name(prefix, source, target),
            output="screen",
            parameters=[{
                "input_topic": source,
                "output_topic": target,
                "message_type": message_type,
                "input_reliability": input_reliability,
                "output_reliability": "reliable",
            }],
        ))

    if mode == "sim" and launch_stage in ("all", "sim"):
        if world_profile not in worlds:
            raise ValueError(f"unknown world '{world_profile}'; available: {sorted(worlds)}")
        compatible = vehicle.get("compatible_worlds", [])
        if compatible and world_profile not in compatible:
            raise ValueError(f"vehicle '{vehicle_name}' is not configured for '{world_profile}'")
        world = worlds[world_profile]
        effective_world_name = world_name_override or str(world["gazebo_name"])
        spawn = world["spawn"]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "simulation.launch.py")),
            launch_arguments={
                "world_file": str(resolve_asset(share, world["sdf"])),
                "world_name": effective_world_name,
                "urdf": str(resolve_asset(share, vehicle["urdf"])),
                "model_name": vehicle_name,
                "use_sim_time": use_sim_time,
                "vehicle_cmd_vel_topic": cmd_vel_topic,
                "vehicle_odom_topic": odom_topic,
                "vehicle_imu_topic": imu_topic,
                "vehicle_gps_topic": gps_topic,
                "vehicle_scan_topic": scan_topic,
                "vehicle_points_topic": points_topic,
                "vehicle_camera_image_topic": camera_image_topic,
                "vehicle_camera_info_topic": camera_info_topic,
                "vehicle_camera_depth_image_topic": camera_depth_image_topic,
                "spawn_x": LaunchConfiguration("spawn_x").perform(context) or str(spawn["x"]),
                "spawn_y": LaunchConfiguration("spawn_y").perform(context) or str(spawn["y"]),
                "spawn_z": LaunchConfiguration("spawn_z").perform(context) or str(spawn["z"]),
                "spawn_yaw": LaunchConfiguration("spawn_yaw").perform(context) or str(spawn["yaw"]),
            }.items(),
        ))

    if launch_stage == "sim":
        return actions

    if mode == "hardware":
        add_topic_relay("/cmd_vel", cmd_vel_topic, "geometry_msgs/msg/Twist", "cmd_vel", "reliable")
        add_topic_relay(odom_topic, "/odom", "nav_msgs/msg/Odometry", "odom")
        add_topic_relay(imu_topic, "/vectornav/imu/data", "sensor_msgs/msg/Imu", "imu")
        add_topic_relay(gps_topic, "/gps/fix", "sensor_msgs/msg/NavSatFix", "gps")
        add_topic_relay(scan_topic, "/scan", "sensor_msgs/msg/LaserScan", "scan")
        add_topic_relay(points_topic, "/points", "sensor_msgs/msg/PointCloud2", "points")
        add_topic_relay(camera_image_topic, "/camera/image_raw", "sensor_msgs/msg/Image", "camera_image")
        add_topic_relay(camera_info_topic, "/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo", "camera_info")
        add_topic_relay(camera_depth_image_topic, "/camera/depth/image_raw", "sensor_msgs/msg/Image", "camera_depth")
        if not (use_perception and use_depth_image_proc):
            add_topic_relay(camera_points_topic, "/camera/depth/points", "sensor_msgs/msg/PointCloud2", "camera_points")
        if localization_mode == "external":
            add_topic_relay(filtered_odom_topic, "/odometry/filtered",
                            "nav_msgs/msg/Odometry", "filtered_odom")

    if use_perception:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "perception.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "segmentation_model_registry": LaunchConfiguration("segmentation_model_registry"),
                "segmentation_model": LaunchConfiguration("segmentation_model"),
                "segmentation_model_path": LaunchConfiguration("segmentation_model_path"),
                "segmentation_config_path": LaunchConfiguration("segmentation_config_path"),
                "terrain_mapper_config": LaunchConfiguration("terrain_mapper_config"),
                "terrain_semantic_profile": LaunchConfiguration("terrain_semantic_profile"),
                "use_depth_image_proc": LaunchConfiguration("use_depth_image_proc"),
                "segmentation_input_topic": "/camera/image_raw",
                "depth_image_topic": "/camera/depth/image_raw",
                "depth_camera_info_topic": "/camera/depth/camera_info",
                "terrain_points_topic": "/camera/depth/points",
            }.items(),
        ))

    if localization_mode == "managed":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "localization.launch.py")),
            launch_arguments={
                "ekf_config": ekf_config, "use_sim_time": use_sim_time,
                "odom_topic": "/odom",
                "imu_topic": "/vectornav/imu/data",
                "gps_topic": "/gps/fix",
            }.items(),
        ))

    configured_nav2 = RewrittenYaml(
        source_file=nav2_config, root_key="",
        param_rewrites={"use_sim_time": use_sim_time,
                        "odom_topic": "/odometry/filtered"},
        convert_types=True,
    )
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time, "params_file": configured_nav2,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    ))
    if _truth(LaunchConfiguration("use_rviz").perform(context)):
        rviz_config = vehicle.get("rviz_config", "rviz/ranger_nav2_custom.rviz")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "visualization.launch.py")),
            launch_arguments={"rviz_config": str(resolve_asset(share, rviz_config)),
                              "use_sim_time": use_sim_time}.items(),
        ))
    if _truth(LaunchConfiguration("use_mapviz").perform(context)):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "mapviz.launch.py"))))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("mode", default_value="sim"),
        DeclareLaunchArgument("launch_stage", default_value="all"),
        DeclareLaunchArgument("vehicle", default_value="ranger"),
        DeclareLaunchArgument("world", default_value="CAST_new"),
        DeclareLaunchArgument("world_name", default_value=""),
        DeclareLaunchArgument("localization_mode", default_value="managed"),
        DeclareLaunchArgument("ekf_config", default_value=""),
        DeclareLaunchArgument("nav2_config", default_value=""),
        DeclareLaunchArgument("filtered_odom_topic", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value=""),
        DeclareLaunchArgument("imu_topic", default_value=""),
        DeclareLaunchArgument("gps_topic", default_value=""),
        DeclareLaunchArgument("scan_topic", default_value=""),
        DeclareLaunchArgument("points_topic", default_value=""),
        DeclareLaunchArgument("camera_image_topic", default_value=""),
        DeclareLaunchArgument("camera_info_topic", default_value=""),
        DeclareLaunchArgument("camera_depth_image_topic", default_value=""),
        DeclareLaunchArgument("camera_points_topic", default_value=""),
        DeclareLaunchArgument("use_perception", default_value="False"),
        DeclareLaunchArgument("use_depth_image_proc", default_value="True"),
        DeclareLaunchArgument("segmentation_model_registry", default_value="config/perception/segmentation_models.yaml"),
        DeclareLaunchArgument("segmentation_model", default_value="rellis_7_class"),
        DeclareLaunchArgument("segmentation_model_path", default_value=""),
        DeclareLaunchArgument("segmentation_config_path", default_value=""),
        DeclareLaunchArgument("terrain_mapper_config", default_value="config/perception/terrain_mapper.yaml"),
        DeclareLaunchArgument("terrain_semantic_profile", default_value=""),
        DeclareLaunchArgument("spawn_x", default_value=""),
        DeclareLaunchArgument("spawn_y", default_value=""),
        DeclareLaunchArgument("spawn_z", default_value=""),
        DeclareLaunchArgument("spawn_yaw", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("use_mapviz", default_value="False"),
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument("autostart", default_value="True"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch)])
