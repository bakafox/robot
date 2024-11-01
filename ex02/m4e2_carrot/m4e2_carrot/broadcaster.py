import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose

import numpy as np


# Преобразование углов Эйлера в кватернионы
# (на вход -- вращения кокруг осей X, Y, Z, на выход --
# x-, y-, z- и скалярная компоненты кватерниона)
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    
    ci = np.cos(ai); si = np.sin(ai)
    cj = np.cos(aj); sj = np.sin(aj)
    ck = np.cos(ak); sk = np.sin(ak)

    cc = ci*ck; cs = ci*sk
    sc = si*ck; ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):
    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Подстраиваемся под имя топика черепашки, переданное в параметрах запуска
        self.turtlename = self.declare_parameter('turtlename').get_parameter_value().string_value

        # Публикует tf2-преобразования по вызову
        self.tf_broadcaster = TransformBroadcaster(self)

        # Для Publisher-а, пыполняем подписку на топик
        # и задаём колбэк при получении новой публикации
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1
        )


    def handle_turtle_pose(self, req):
        res = TransformStamped() # <-- Объект сообщения преобразования tf2

        # Перед отправкой сообщения преобразования очень важно добавить
        # метаданные в заголовок -- таймштамп, названия фрейма и его родителя
        res.header.stamp = self.get_clock().now().to_msg()
        res.header.frame_id = 'world'
        res.child_frame_id = self.turtlename

        # Следом сами данные преобразования -- 
        # 1. Смещение в XYZ (черепашка Z не исопльзует)
        res.transform.translation.x = req.x
        res.transform.translation.y = req.y
        res.transform.translation.z = 0.0

        # 2. Углы поворота в кватернионах
        # (https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
        q = quaternion_from_euler(0, 0, req.theta)
        res.transform.rotation.x = q[0]
        res.transform.rotation.y = q[1]
        res.transform.rotation.z = q[2]
        res.transform.rotation.w = q[3]

        # Наконец, отправляем сообщение преобразования
        self.tf_broadcaster.sendTransform(res)


def main():
    rclpy.init()
    node = FramePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
