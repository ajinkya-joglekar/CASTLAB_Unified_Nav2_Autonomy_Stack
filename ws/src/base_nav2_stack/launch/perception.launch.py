"""Optional semantic segmentation and terrain traversability bringup."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_path(value, default_package=None):
    value = str(value).strip()
    if not value:
        return ""
    if value.startswith("package://"):
        rest = value[len("package://"):]
        package, relative = rest.split("/", 1)
        return str(Path(get_package_share_directory(package)) / relative)
    path = Path(value).expanduser()
    if path.is_absolute():
        return str(path)
    if default_package:
        return str(Path(get_package_share_directory(default_package)) / path)
    return str(path)


def _load_model_profile(registry_path, model_name):
    with open(registry_path, "r", encoding="utf-8") as stream:
        registry = yaml.safe_load(stream) or {}
    models = registry.get("models", {})
    if model_name not in models:
        raise ValueError(f"unknown segmentation_model '{model_name}'; available: {sorted(models)}")
    return models[model_name]


def _launch(context):
    share = Path(get_package_share_directory("base_nav2_stack"))
    launch_dir = share / "launch"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = use_sim_time.strip().lower() in ("true", "1", "yes", "on")
    use_depth_image_proc = LaunchConfiguration("use_depth_image_proc").perform(context).lower() in (
        "true", "1", "yes", "on"
    )
    registry = LaunchConfiguration("segmentation_model_registry").perform(context).strip()
    registry_path = Path(_resolve_path(registry, "base_nav2_stack") if registry else
                         share / "config/perception/segmentation_models.yaml")
    terrain_config = LaunchConfiguration("terrain_mapper_config").perform(context).strip()
    terrain_config_path = _resolve_path(
        terrain_config or "config/perception/terrain_mapper.yaml",
        "base_nav2_stack",
    )

    model_name = LaunchConfiguration("segmentation_model").perform(context).strip()
    profile = _load_model_profile(registry_path, model_name)

    model_path = LaunchConfiguration("segmentation_model_path").perform(context).strip()
    config_path = LaunchConfiguration("segmentation_config_path").perform(context).strip()
    semantic_profile = LaunchConfiguration("terrain_semantic_profile").perform(context).strip()

    resolved_model_path = _resolve_path(model_path or profile["model_path"])
    resolved_config_path = _resolve_path(config_path or profile["config_path"])
    effective_semantic_profile = semantic_profile or str(profile.get("semantic_profile", "auto"))

    segmentation_input_topic = LaunchConfiguration("segmentation_input_topic").perform(context)
    depth_image_topic = LaunchConfiguration("depth_image_topic").perform(context)
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic").perform(context)
    terrain_points_topic = LaunchConfiguration("terrain_points_topic").perform(context)

    actions = []
    if use_depth_image_proc:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "rgbd_to_points.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "depth_image_topic": depth_image_topic,
                "depth_camera_info_topic": depth_camera_info_topic,
                "points_topic": terrain_points_topic,
            }.items(),
        ))

    actions.extend([
        Node(
            package="semantic_segmentation_node",
            executable="segmentation_node",
            name="segmentation_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time_bool,
                "input_topic": segmentation_input_topic,
                "model_path": resolved_model_path,
                "config_path": resolved_config_path,
                "input_width": int(profile.get("input_width", 512)),
                "input_height": int(profile.get("input_height", 512)),
                "resize_input": bool(profile.get("resize_input", True)),
                "normalize": bool(profile.get("normalize", True)),
                "device": str(profile.get("device", "auto")),
                "publish_overlay": bool(profile.get("publish_overlay", True)),
                "mask_topic": "/segmentation/mask",
                "confidence_topic": "/segmentation/confidence",
                "label_info_topic": "/segmentation/label_info",
                "overlay_topic": "/segmentation/overlay",
                "debug_histogram": bool(profile.get("debug_histogram", True)),
                "debug_every_n_frames": int(profile.get("debug_every_n_frames", 30)),
            }],
        ),
        Node(
            package="terrain_obstacle_mapper",
            executable="terrain_obstacle_grid_node",
            name="terrain_obstacle_grid_node",
            output="screen",
            parameters=[
                terrain_config_path,
                {
                    "use_sim_time": use_sim_time_bool,
                    "points_topic": terrain_points_topic,
                    "mask_topic": "/segmentation/mask",
                    "label_info_topic": "/segmentation/label_info",
                    "grid_topic": "/terrain/obstacle_grid",
                    "semantic_grid_topic": "/terrain/semantic_grid",
                    "semantic_profile": effective_semantic_profile,
                },
            ],
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("segmentation_model_registry", default_value="config/perception/segmentation_models.yaml"),
        DeclareLaunchArgument("segmentation_model", default_value="rellis_7_class"),
        DeclareLaunchArgument("segmentation_model_path", default_value=""),
        DeclareLaunchArgument("segmentation_config_path", default_value=""),
        DeclareLaunchArgument("terrain_mapper_config", default_value="config/perception/terrain_mapper.yaml"),
        DeclareLaunchArgument("terrain_semantic_profile", default_value=""),
        DeclareLaunchArgument("use_depth_image_proc", default_value="true"),
        DeclareLaunchArgument("segmentation_input_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("depth_image_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("depth_camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("terrain_points_topic", default_value="/camera/depth/points"),
        OpaqueFunction(function=_launch),
    ])
