from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import ament_index_python.packages

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command

import os


def generate_launch_description():
    pkg__ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg__this = get_package_share_directory('m5e5_kuvshinka')

    # Запускаем публикатор состояния робота в tf2, передаём сразу xacro вот так:
    # (https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/)
    xacro_path = os.path.join(pkg__this, 'src', 'description', 'kuvshinka.urdf.xacro')
    xacro_data = Command(['xacro', ' ', xacro_path])

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': xacro_data,
                'frame_prefix': 'kuvshinka/'
            }],
            output='screen'
    )

    # Запускаем публикатор состояния подвижных соед-й робота в tf2
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': xacro_data,
            'frame_prefix': 'kuvshinka/'
        }],
        output='screen'
    )

    # Запускаем визуализацию Gazebo в определённом мире
    init_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg__ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r shapes.sdf'}.items(),
    )

    # Запускаем среду RViz с заранее заготовленным конфигом
    init_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg__this, 'rviz', 'urdf_config.rviz')
        ],
        output='screen'
    )

    # Мост, транслирующий сообщения из топиков ROS в сообщения Gazebo, и наоборот
    # (список: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg__this, 'ros_gz', 'bridge_config.yaml'),
            'qos_overrides./model/kuvshinka.subscriber.reliability': 'best_effort'
        }],
        output='screen'
    )

    # Создаём робота в среде Gazebo
    
    kuvshinka_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'kuvshinka',
            '-x', '10.0',
            '-y', '10.0',
            '-z', '1.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Из этого же пакета запускаем скрипт, который будет отправлять
    # команды для управления роботом через топик /cmd_vel
    start_moving = Node(
        package='m5e5_kuvshinka',
        executable='vosmerka',
        name='vosmerka'
    )

    # Ну и всё это исполняем... дежавю
    return LaunchDescription([
        init_gazebo,
        init_rviz,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        kuvshinka_spawn,
        start_moving
    ])
