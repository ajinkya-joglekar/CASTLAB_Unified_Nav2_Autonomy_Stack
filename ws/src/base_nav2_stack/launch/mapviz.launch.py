import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

gps_wpf_dir = get_package_share_directory("base_nav2_stack")
mapviz_config_file = os.path.join(gps_wpf_dir, "config", "gps_wpf_castlab.mvc")
print(mapviz_config_file)
# exit()


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": mapviz_config_file}]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "gps/filtered"),
            ],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
    ])
