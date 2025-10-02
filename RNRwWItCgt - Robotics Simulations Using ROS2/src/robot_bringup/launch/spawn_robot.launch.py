from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_pkg = get_package_share_directory('robot_description')
    urdf_path = os.path.join(description_pkg, 'urdf', 'two_wheeler_robot.urdf')
    world_path = os.path.join(description_pkg, 'world', 'maze_world.world')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Spawn robot at corridor entry (y = -3 instead of 0)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'two_wheeler_robot',
                       '-x', '0.0', '-y', '-3.0', '-z', '0.1'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
    ])
