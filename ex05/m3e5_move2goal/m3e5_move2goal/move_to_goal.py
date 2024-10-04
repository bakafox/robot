import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import sys
import time
import numpy as np


class MoveToGoal(Node):
    def __init__(self, turtleX):
        super().__init__('move_to_goal')

        if (len(sys.argv) != 4):
            self.get_logger().error('Ошибка -- укажите X, Y и угол THETA.')
            sys.exit(1)

        self.X_end = float(sys.argv[1])
        self.Y_end = float(sys.argv[2])
        self.THETA_end = float(sys.argv[3])

        self.X_start = self.Y_start = self.THETA_start = 0

        self.subscription = self.create_subscription(
            Pose,     # Тип принимаемых СООБЩЕНИЙ
            f'/{turtleX}/pose', # Название ТОПИКА, который слушаем
            self.move_to_goal,   # Колбэк по получении СООБЩЕНИЯ
            10  # Глубина истории принятых сообщений (не исп.)
        )

        self.publisher_ = self.create_publisher(
            Twist,      # Тип публикуемых СООБЩЕНИЙ
            f'/{turtleX}/cmd_vel',  # Название ТОПИКА, в который публикуем
            10  # Глубина истории отправленных сообщений (не исп.)
        )

    def move_to_goal(self, req):
        # print('\n======\n', req, '\n======\n')
        self.X_start = req.x
        self.Y_start = req.y
        self.THETA_start = req.theta
    
        deg2rad_coef = 0.01745329251994329576923690768489

        # Сперва находим расстояние между текущим XY черепашки и требуемым XY,
        # а также угол между текущим поворотом и поворотом до прямой расстояния:
        distance = np.sqrt((self.X_end - self.X_start)**2 + (self.Y_end - self.Y_start)**2)
        # angle = ((np.arctan2((self.Y_end - self.Y_start), (self.X_end - self.X_start))
        #         - self.THETA_start) % (2*np.pi))
        angle = np.arctan2((self.Y_end - self.Y_start), (self.X_end - self.X_start))
        angle1 = np.arctan2(np.sin(angle - self.THETA_start), np.cos(angle - self.THETA_start))
        angle2 = self.THETA_end - angle
        
        self.get_logger().info('Расстояние до точки: %f, угол 1: %f, угол 2: %f' %
                               (distance, angle1, angle2))

        # Здесь и далее: в turtlesim все команды выполняются примерно 1 секунду,
        # поэтому можно обойтись просто sleep-ами. Если бы тут была более реальная
        # задача, перед отправкой каждого сообщения пришлось бы отслеживать, пока
        # ОБЕ velocity в Pose не =0 (что означает, что прошлая команда окончена).

        # Поворачиваем черепашку на угол до расстояния
        res = Twist()
        res.angular.z = angle1
        # print('\n======\n', res, '\n======\n')
        self.publisher_.publish(res)
        time.sleep(1.5)

        # Проходим расстояние до требуемых XY
        res = Twist()
        res.linear.x = distance
        # print('\n======\n', res, '\n======\n')
        self.publisher_.publish(res)
        time.sleep(1.5)

        # Наконец, поворачиваем её на требуемый угол
        res = Twist()
        res.angular.z = angle2
        # print('\n======\n', res, '\n======\n')
        self.publisher_.publish(res)
        time.sleep(1.5)

        self.get_logger().info('Дело сделано!')


def main(args=None):
    rclpy.init(args=args)

    move2goal = MoveToGoal('turtle1')

    rclpy.spin_once(move2goal)

    move2goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
