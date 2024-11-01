from launch import LaunchDescription
from launch_ros.actions import Node

import numpy as np


def generate_launch_description():
    LOLCHAIN = [
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
    ]

    for delay in np.arange(0.0, 5.0, 0.5):
        # print(delay, float(delay))
        LOLCHAIN.extend([
            Node(
                package='m4e3_timetravel',
                executable='broadcaster',
                name='broadcaster_follower',
                parameters=[
                    {'turtlename': f'turtle{int(delay*2)+1}'},
                ],
            ),
            Node(
                package='m4e3_timetravel',
                executable='listener',
                name='listener_follower',
                parameters=[
                    {'turtlename': f'turtle{int(delay*2)+1}'},
                    {'target_frame': 'turtle1'},
                    {'delay_sec': float(delay)}
                ],
            ),
        ])

    LOLCHAIN.extend([
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_main',
            prefix='gnome-terminal -- ',
        ),
    ])

    return LaunchDescription(LOLCHAIN)
