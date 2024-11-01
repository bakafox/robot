from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_main',
        ),
        Node(
            package='m4e3_timetravel',
            executable='broadcaster',
            name='broadcaster_main',
            parameters=[
                {'turtlename': 'turtle1'},
            ],
        ),
        Node(
            package='m4e3_timetravel',
            executable='broadcaster',
            name='broadcaster_follower',
            parameters=[
                {'turtlename': 'turtle2'},
            ],
        ),
        Node(
            package='m4e3_timetravel',
            executable='listener',
            name='listener_follower',
            parameters=[
                {'turtlename': 'turtle2'},
                {'target_frame': 'turtle1'},
                {'delay_sec': 2.0}
            ],
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_main',
            prefix='gnome-terminal -- ',
        ),
    ])
