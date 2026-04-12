from launch import LaunchDescription
from launch_ros.actions import Node

#
# ros2 launch geofence_manager geofence.launch.py
#

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='geofence_manager',
            executable='geofence_manager_node',
            name='geofence_manager',
            output='screen',
            parameters=[
                {'geofence_file': 'config/geofence.yaml'},
                {'world_frame': 'map'},
                {'robot_frame': 'base_link'},
                {'pose_topic': '/odometry/global'},
                {'near_boundary_threshold_m': 2.0},
            ]
        )
    ])
