from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='obstacle_node',
            name='obstacle_avoidance',
            output='screen'
        )
    ])
