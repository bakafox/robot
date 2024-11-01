import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np


class DynamicFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')

        # Парсим все требуемые параметры (в этот раз их целых 4)
        self.name = self.declare_parameter('name').get_parameter_value().string_value
        self.target = self.declare_parameter('target').get_parameter_value().string_value
        self.radius = self.declare_parameter('radius').get_parameter_value().integer_value
        self.direction = self.declare_parameter('direction').get_parameter_value().integer_value
        self.direction = (1.0 if (self.direction > 0) else -1.0)
        self.get_logger().warn('============\n %s %s %f %f \n============' %
                               (self.target, self.name, self.radius, self.direction))

        # Просто публикуем tf2-преобразования каждые N сек по таймеру
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.broadcast_timer_callback)


    def broadcast_timer_callback(self):
        time, _ = self.get_clock().now().seconds_nanoseconds()

        t = TransformStamped() # <-- Объект сообщения преобразования tf2

        # Перед отправкой сообщения преобразования очень важно добавить
        # метаданные в заголовок -- таймштамп, названия фрейма и его родителя
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.target
        t.child_frame_id = self.name

        # Следом сами данные преобразования -- 
        # 1. Смещение в XYZ (черепашка Z не исопльзует)
        t.transform.translation.x = self.radius * np.sin(time * self.direction)
        t.transform.translation.y = self.radius * np.cos(time * self.direction)
        t.transform.translation.z = 0.0
        self.get_logger().warn('======\n %f %f %f %f \n ======' %
                               (time, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z))

        # 2. Углы поворота в кватернионах
        # (https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 2.0

        # Наконец, отправляем сообщение преобразования
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
