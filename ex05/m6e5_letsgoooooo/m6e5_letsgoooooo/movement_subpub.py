import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class Movement_SubPub(Node):
    def __init__(self):
        super().__init__('retranslator')

        self.subscription = self.create_subscription(
            Image,
            '/depth/image',
            self.retranslate,
            10
        )
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def retranslate(self, req):
        print(req)
        print(req.data)
        print(len(req.data), req.width, req.height)   # Матрица 848x480 каких-то значений
        print(len(req.data) / req.width / req.height) # от 0 до 128, по 4 значения на ячейку
        
        # https://drive.google.com/file/d/1OV5_QhdR11FYU34sFdcM5H8VnLxaFno4/view
        cv_image = self.br.imgmsg_to_cv2(req, desired_encoding='passthrough') # Преобразование в картинку cv2
        depth_array = np.array(cv_image, dtype=np.float32).clip(min=0, max=10) # Обрезка от min=0 до max=10
        depth_image = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX) # Нормализация к спектру RGB
    
        print(depth_image)
        print(depth_image.shape) # 480x848 (транспонировалась? повернулась на 90?)

        # Начиная с 250-й строки, сенсор начинает задевать сперва пол, а затем
        # корпус самого робота, поэтому все строчки где-то с 300-й можно отбросить.
        for str in depth_image[100:299]:
            print(np.mean(str), min(str))


        res = Twist()

        # На основе попытных наблюдений, максимально полезными для
        # поиска препятстсвий будут значения где-то от 20-30 и до 50-60        
        for str in depth_image[100:299]:
            for color in str[360:639]:
                if (color < 48):
                    self.get_logger().info('Препятствие спереди слишком близко, я дальше не поеду.')
                    print(np.mean(str), min(str), color)
                    res.linear.x = 0.0
                    self.publisher_.publish(res)
                    return
            
            # Хороший вопрос, имеет ли делать что-то такое вообще смысл,
            # ведь поле зрения глубинной камера куда уже, чем у лидара... на практике,
            # даже с таким высоким значением, ничего почти никогда не обнаруживается.
            for color in (str[240:359] + str[640:759]):
                if (color < 60):
                    self.get_logger().info('Препятствие сбоку слишком близко, я дальше не поеду.')
                    print(np.mean(str), min(str), color)
                    res.linear.x = 0.0
                    self.publisher_.publish(res)
                    return

        # Увы, расст. до ближайшего препятствия здесь, в отличии от лидара, уже не узнать
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
