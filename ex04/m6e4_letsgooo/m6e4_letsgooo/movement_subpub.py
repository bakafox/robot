import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Movement_SubPub(Node):
    def __init__(self):
        super().__init__('retranslator')

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.retranslate,
            10
        )

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def retranslate(self, req):
        print(req) # Что делает хедер я не понял, но он видимо и не нужен
        print(req.ranges) # Каждое значение -- это расстояние, пройденное лазером,
                          # то есть чем больше (вплоть до inf), тем дальше.
        print(len(req.ranges)) # 360, по 1 на градус
        print('===')

        res = Twist()

        for dist in req.ranges[150:209]:
            if dist < 1.2:
                self.get_logger().info('Препятствие спереди слишком близко, я дальше не поеду.')
                res.linear.x = 0.0
                self.publisher_.publish(res)
                return

        for dist in (req.ranges[70:149] + req.ranges[210:279]):
            if dist < 0.6:
                self.get_logger().info('Препятствие сбоку слишком близко, я дальше не поеду.')
                res.linear.x = 0.0
                self.publisher_.publish(res)
                return

        self.get_logger().info('До ближайшего препятствия %f м, еду дальше...' %
                               (min(req.ranges[170:190])))
        res.linear.x = 1.0
        self.publisher_.publish(res)
        return


def main(args=None):
    rclpy.init(args=args)

    # Этот код написан на основе кода "ретранслятора" из m2e10... летит время, однако.

    retranslator = Movement_SubPub()
    retranslator.get_logger().warn('Внимание-внимание, запускаю ретранслятор!')

    rclpy.spin(retranslator)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
