import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

# https://docs.nav2.org/setup_guides/urdf/setup_urdf.html (версия без привязки к пакету)

def generate_launch_description():
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('jspg'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('jspg'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='jspg', description='Нужно ли открывать joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='urdf', description='Путь до файла модели робота в формате urdf'),
        launch.actions.DeclareLaunchArgument(name='rviz', description='Путь до файла настроек rviz2 в формате rviz'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
