from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlesim',
            namespace='_4EPEnAXA_01',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='turtlesim',
            namespace='_4EPEnAXA_02',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/_4EPEnAXA_01/turtle1/pose'),
                ('/output/cmd_vel', '/_4EPEnAXA_02/turtle1/cmd_vel'),
            ]
        ),

        Node(
            package='turtlesim',
            namespace='_4EPEnAXA_03',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/_4EPEnAXA_02/turtle1/pose'),
                ('/output/cmd_vel', '/_4EPEnAXA_03/turtle1/cmd_vel'),
            ]
        )

    ])
