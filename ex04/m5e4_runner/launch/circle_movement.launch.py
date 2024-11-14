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
    pkg__this = get_package_share_directory('m5e4_runner')

    # Запускаем публикатор состояния робота в tf2, передаём сразу xacro вот так:
    # (https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/)
    xacro_path = os.path.join(pkg__this, 'src', 'description', 'runner.urdf.xacro')
    xacro_data = Command(['xacro', ' ', xacro_path])

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': xacro_data,
                'frame_prefix': 'runner/'
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
            'frame_prefix': 'runner/'
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
            'qos_overrides./model/runner.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )

    # Создаём робота в среде Gazebo
    runner_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'runner',
            '-x', LaunchConfiguration('rad'),
            '-y', '0.0',
            '-z', '1.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Из этого же пакета запускаем скрипт, который будет отправлять
    # команды для управления роботом через топик /cmd_vel
    circle_movement = Node(
        package='m5e4_runner',
        executable='circle',
        name='circle_movement',
            arguments=[
                LaunchConfiguration('vel'),
                LaunchConfiguration('rad'),
                LaunchConfiguration('dir'),
            ],
    )

    # Запускаем teleop_twist_keyboard для управления роботом в газебре
    # (его надо поставить отсюда: https://index.ros.org/p/teleop_twist_keyboard/)
    telepop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='gnome-terminal -- ',
        output='screen',
        condition=IfCondition(LaunchConfiguration('telepop'))
    )

    # Ну и собственно парим аргументы запуска и всё это исполняем, да
    return LaunchDescription([
        DeclareLaunchArgument(name='vel', description='Скорость, с которой побежит, бегун'),
        DeclareLaunchArgument(name='rad', description='Радиус кругов, которые бежит бегун'),
        DeclareLaunchArgument(name='dir', description='В какую сторону будет бежать бегун'),
        DeclareLaunchArgument(name='telepop', description='Нужна ли клавиатура для ручного управления бегуном'),

        init_gazebo,
        init_rviz,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        runner_spawn,
        circle_movement,
        telepop
    ])
