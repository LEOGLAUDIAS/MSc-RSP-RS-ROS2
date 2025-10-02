from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='teleop_node',
            name='teleop_keyboard',
            output='screen'
        )
    ])
