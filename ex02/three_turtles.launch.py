from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='2.0',
        description='Radius of the carrot frame'
    )

    direction_arg = DeclareLaunchArgument(
        'direction_of_rotation',
        default_value='1',
        description='Direction of rotation: 1 for clockwise, -1 for counterclockwise'
    )

    # Path to the RViz configuration file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('turtle_tf2_carrot'), 'rviz', 'carrot.rviz']
    )

    return LaunchDescription([
        radius_arg,
        direction_arg,
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "{x: 4.0, y: 2.0, theta: 0.0, name: 'turtle2'}"],
            output='screen'
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='turtle1_tf_broadcaster',
            name='turtle1_tf_broadcaster',
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='turtle2_tf_broadcaster',
            name='turtle2_tf_broadcaster',
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='carrot_tf_broadcaster',
            name='carrot_tf_broadcaster',
            parameters=[
                {'radius': LaunchConfiguration('radius')},
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
            ]
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='turtle2_tf_listener',
            name='turtle2_tf_listener',
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_teleop_key',
            prefix='xterm -e',  # Opens turtle_teleop_key in a new terminal
        ),
        # Add RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
