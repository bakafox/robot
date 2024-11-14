import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import rclpy.timer


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.i = -5

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.timer = self.create_timer(1.0, self.send_part_of_vosmerka)
        
    def send_part_of_vosmerka(self):
        msg = Twist()
        
        self.i += 1

        # Задержка первые N раз, чтобы дать газебре прогрузиться
        if (self.i < 0):
            return

        # 0 -- Прямая линия
        # 1 -- Начало скругления
        # 2 -- Активная часть скругления
        # 3 -- Конец скругления
        if (self.i % 2 == 1):
            msg.linear.x = 1.2
            msg.angular.z = 1.3
        elif (self.i % 4 == 2):
            msg.linear.x = 0.6
            msg.angular.z = 1.7
        else:
            msg.linear.x = 2.4
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Опубликовано сообщение #%d' % (self.i))
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.get_logger().info('Начинаю публикацию в топик /cmd_vel для ROS2!')
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
