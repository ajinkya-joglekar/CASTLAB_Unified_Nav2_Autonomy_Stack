from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE = "base_nav2_stack"


def package_tree(directory):
    entries = []
    for root, _, files in os.walk(directory):
        if files:
            entries.append(
                (os.path.join("share", PACKAGE, root),
                 [os.path.join(root, name) for name in files])
            )
    return entries


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
    ("share/" + PACKAGE, ["package.xml"]),
]
for asset_dir in ("config", "launch", "meshes", "rviz", "urdf", "worlds"):
    data_files.extend(package_tree(asset_dir))

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Ajinkya Joglekar",
    maintainer_email="ajoglek@tamu.edu",
    description="Profile-driven Nav2 GPS baseline for simulation and hardware",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "launch_gui = base_nav2_stack.launch_gui:main",
            "logged_waypoint_follower = base_nav2_stack.logged_waypoint_follower:main",
            "interactive_waypoint_follower = base_nav2_stack.interactive_waypoint_follower:main",
            "gps_waypoint_logger = base_nav2_stack.gps_waypoint_logger:main",
            "topic_relay = base_nav2_stack.topic_relay:main",
        ],
    },
)
