from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("geofence_manager")
    geofence_file = os.path.join(pkg_share, "config", "geofence.yaml")

    return LaunchDescription([
        Node(
            package="geofence_manager",
            executable="geofence_manager_node",
            name="geofence_manager",
            output="screen",
            parameters=[
                {"geofence_file": geofence_file},
                {"pose_topic": "/robot_pose"},
                {"pose_source_type": "pose_stamped"},
                {"world_frame": "map"},
                {"near_boundary_threshold_m": 2.0},
                {"publish_markers": True},
                {"publish_polygon": True},
                {"status_publish_rate_hz": 5.0},
                {"localization_timeout_sec": 2.0},
            ],
        )
    ])
