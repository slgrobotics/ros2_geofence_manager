import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("geofence_manager")
    geofence_file = os.path.join(pkg_share, "config", "geofence_polygon.yaml")
    params_file = os.path.join(pkg_share, "config", "geofence_params.yaml")

    return LaunchDescription([
        Node(
            package="geofence_manager",
            executable="geofence_manager_node",
            name="geofence_manager",
            output="screen",
            parameters=[
                params_file,
                {"geofence_file": geofence_file},
            ],
        )
    ])
