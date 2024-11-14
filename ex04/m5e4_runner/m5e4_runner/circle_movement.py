import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.i = 0

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        vel = float(sys.argv[1])
        rad = float(sys.argv[2])
        dir = 1.0 if (float(sys.argv[3]) >= 0) else -1.0
        self.get_logger().info('%f %f %f' % (vel, rad, dir))

        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = dir * (vel / rad)

        self.publisher_.publish(msg)
        self.i += 1
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
