import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#
# colcon build; source install/setup.bash; ros2 launch geofence_manager geofence.launch.py use_sim_time:=true
#

def generate_launch_description():
    pkg_share = get_package_share_directory("geofence_manager")

    geofence_file = os.path.join(pkg_share, "plans", "geofence_polygon.yaml")
    #geofence_file = os.path.join(pkg_share, "plans", "geofence-1.plan")

    params_file = os.path.join(pkg_share, "config", "geofence_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (/clock)"
        ),

        Node(
            package="geofence_manager",
            executable="geofence_manager_node",
            name="geofence_manager",
            output="screen",
            parameters=[
                params_file,
                {
                    "geofence_file": geofence_file,
                    "use_sim_time": use_sim_time,
                },
            ],
        )
    ])
