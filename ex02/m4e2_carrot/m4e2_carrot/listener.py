import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

import numpy as np


class FrameListener(Node):
    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Подстраиваемся под имя топика черепашки, переданное в параметрах запуска
        self.turtlename = self.declare_parameter('turtlename').get_parameter_value().string_value

        # Получаем из параметров запуска фрейм, за которым нужно следовать
        self.target_frame = self.declare_parameter('target_frame').get_parameter_value().string_value

        # Получает tf2-преобразования и хранит из в буфере до 10 сек
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Для обращения в /srv/Spawn черепашки, что будет использоваться
        # для проверки, что она уже существует, и создания, если нет
        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        # Создаём публикатор в топик /cmd_vel (движения черепашки)
        self.publisher = self.create_publisher(Twist, f'{self.turtlename}/cmd_vel', 1)

        # Вызов фукнции каждую секунду
        self.timer = self.create_timer(1.0, self.on_timer)


    def on_timer(self):
        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # На основе последних в буфере tf2-преобразованиях
                # turtle1 и turtle2 получаем tf2-преобразование такое,
                # чтобы оно максимально приближало turtle2 к turtle1
                try:
                    t = self.tf_buffer.lookup_transform(
                        self.turtlename, # Нач. точка
                        self.target_frame, # Кон. точка
                        rclpy.time.Time() # Время отн. буфера
                    )
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {self.turtlename} to {self.turtlename}: {ex}')
                    return

                # На основе полученного tf2-преобразования создаём
                # сообщение Twish для публикации в топик /cmd_vel
                msg = Twist()
                msg.angular.z = 1.2 * np.arctan2(
                    t.transform.translation.y,
                    t.transform.translation.x
                )
                msg.linear.x = 1.0 * np.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2
                )
                self.publisher.publish(msg)

            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')

        else:
            if self.spawner.service_is_ready():
                # Создаём запрос создания черепашки turtle2
                # и отправляем его в /srv/Spawn соотв. топика
                request = Spawn.Request()
                request.name = self.turtlename
                request.x = np.random.uniform(1, 10)
                request.y = np.random.uniform(1, 10)
                request.theta = np.random.uniform(1, np.pi*2)

                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                self.get_logger().info('Service is not ready')


def main():
    rclpy.init()
    node = FrameListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
