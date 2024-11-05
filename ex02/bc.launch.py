from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_main',
        ),
        Node(
            package='m4e2_carrot',
            executable='broadcaster',
            name='broadcaster_main',
            parameters=[
                {'turtlename': 'turtle1'},
            ],
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_main',
            prefix='gnome-terminal -- ',
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 "ros2 run tf2_ros tf2_echo world turtle1"],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 "ros2 topic echo /tf"],
            output='screen',
        ),
    ])
