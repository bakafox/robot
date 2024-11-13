from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    # Компилируем XACRO в URDF, затем прочитываем получившийся файл и удаляем
    os.system('xacro robot.urdf.xacro > generated_from_xarco.urdf && echo "Генерация URDF из XACRO успешно завершена."')
    robot_file = open(os.path.join(os.getcwd(), 'generated_from_xarco.urdf'), 'r')
    robot_data = robot_file.read()
    os.system('rm generated_from_xarco.urdf && echo "Временный файл URDF успешно прочитан и удалён."')

    # Запускаем публикатор состояния робота в tf2
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_data,
                'frame_prefix': 'myfkinrobot/'
            }],
            output='screen'
    )

    # Запускаем публикатор состояния подвижных соед-й робота в tf2
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_data,
            'frame_prefix': 'myfkinrobot/'
        }],
        output='screen'
    )

    # Запускаем визуализацию Gazebo в определённом мире
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    init_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r shapes.sdf'}.items(),
    )

    # Запускаем среду RViz с заранее заготовленным конфигом
    init_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(os.getcwd(), 'urdf_config.rviz')
        ],
        output='screen'
    )

    # Мост, транслирующий сообщения из топиков ROS в сообщения Gazebo, и наоборот
    # (список: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)
    bridge1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/myfkinrobot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/myfkinrobot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        parameters=[{
            'qos_overrides./model/myfkinrobot.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )
    bridge2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(os.getcwd(), 'bridge_config.yaml'),
            'qos_overrides./model/myfkinrobot.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )

    # Создаём робота в среде Gazebo
    robot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'myfkinrobot',
            '-x', '10.0',
            '-y', '10.0',
            # '-x', '2.0',
            # '-y', '0.0',
            '-z', '4.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Запускаем teleop_twist_keyboard для управления роботом в газебре
    # (его надо поставить отсюда: https://index.ros.org/p/teleop_twist_keyboard/)
    telepop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        prefix='gnome-terminal -- ',
        output='screen'
    )

    # Ну и собственно всё это исполняем, да
    return LaunchDescription([
        init_gazebo,
        init_rviz,
        robot_state_publisher,
        joint_state_publisher_gui,
        bridge2,
        robot_spawn,
        telepop
    ])
