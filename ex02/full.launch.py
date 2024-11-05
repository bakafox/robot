from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('m4e2_carrot'), 'carrot.rviz']
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_carrot',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_main'
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
            package='m4e2_carrot',
            executable='carrot',
            name='broadcaster_carrot',
            parameters=[
                {'name': 'carrot1'},
                {'target': 'turtle1'},
                {'radius': 2},
                {'direction': 666},
            ],
        ),
        Node(
            package='m4e2_carrot',
            executable='broadcaster',
            name='broadcaster_follower',
            parameters=[
                {'turtlename': 'turtle2'},
            ],
        ),
        Node(
            package='m4e2_carrot',
            executable='listener',
            name='listener_follower',
            parameters=[
                {'turtlename': 'turtle2'},
                {'target_frame': 'carrot1'},
            ],
        ),
        Node(
            package='m4e2_carrot',
            executable='carrot',
            name='broadcaster_carrot',
            parameters=[
                {'name': 'carrot2'},
                {'target': 'turtle1'},
                {'radius': 1},
                {'direction': -1},
            ],
        ),
        Node(
            package='m4e2_carrot',
            executable='broadcaster',
            name='broadcaster_follower',
            parameters=[
                {'turtlename': 'turtle3'},
            ],
        ),
        Node(
            package='m4e2_carrot',
            executable='listener',
            name='listener_follower',
            parameters=[
                {'turtlename': 'turtle3'},
                {'target_frame': 'carrot2'},
            ],
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_main',
            prefix='gnome-terminal --',
        ),
    ])
