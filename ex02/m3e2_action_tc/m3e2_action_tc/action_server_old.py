import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from m3e2_action.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import numpy as np
import time


class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('tc_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turle_commands',
            self.kolbek_action
        )

        self.turtlesim_pose = Pose()
        self.turtlesim_found = False

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.kolbek_subscription,
            10
        )


    def kolbek_subscription(self, msg):
        self.turtlesim_pose = msg
        self.turtlesim_found = True


    def kolbek_action(self, goal_handle):
        # print('\n======\n', goal_handle, goal_handle.request, '\n======\n')
        self.get_logger().warn('Получен гоооол: %s, %d, %d' %
                               (goal_handle.request.command, goal_handle.request.s, goal_handle.request.angle))

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        feedback = MessageTurtleCommands.Feedback()
        feedback.odom = 0
        result = MessageTurtleCommands.Result()
        # rate = self.create_rate(10)

        turtlesim_cmd = Twist()
        # Проверяем, запущен ли черепах
        while not self.turtlesim_found:
            self.get_logger().warn('Черепах не найден, жду 1-й публикации в /turtle1/pose...')    
            if (rclpy.spin_once(self, timeout_sec=0.5)):
                self.turtlesim_found = True


        if (command not in ['forward', 'turn_left', 'turn_right']):
            self.get_logger().error('Получена неверная команда от клиента! Прерываю гоооол...\n')
            goal_handle.abort()
            result.result = False
            return result # Здесь и далее: мы должны вернуть результат, даже если вып-е
                          # goal-а было прервано или отменено. Иначе будет ошибка 0_o


        if command == 'forward':
            turtlesim_cmd.linear.x = 1.0
            initial_pose = self.turtlesim_pose
            
            while rclpy.ok() and feedback.odom < s:
                if goal_handle.is_cancel_requested:
                    # Остановка черепашки
                    turtlesim_cmd.linear.x = 0.0
                    self.publisher.publish(turtlesim_cmd)

                    goal_handle.canceled()
                    result.result = False
                    return result

                delta_x = self.turtlesim_pose.x - initial_pose.x
                delta_y = self.turtlesim_pose.y - initial_pose.y
                distance = np.sqrt(delta_x * delta_x + delta_y * delta_y)
                print('\n======\n', self.turtlesim_pose, distance, delta_x, delta_y, '\n======\n',)
                    
                self.publisher.publish(turtlesim_cmd)

                # По какой-то причине, одометрия имеет тип int32,
                # поэтому результат округляем до целого числа:
                feedback.odom = int(distance)
                self.get_logger().info('Отправляю фидбэк об одометрии расстояния клиенту...')
                print('\n======\n', feedback.odom, feedback.odom < s, '\n======\n')
                goal_handle.publish_feedback(feedback)

                # rate.sleep()
                time.sleep(1.5)


        elif command in ['turn_left', 'turn_right']:
            angle_rad = angle * np.pi / 180
            turtlesim_cmd.angular.z = (1.0 if (command == 'turn_left') else -1.0) * 1.0
            initial_theta = self.turtlesim_pose.theta
            
            while rclpy.ok() and feedback.odom < np.abs(angle_rad):
                if goal_handle.is_cancel_requested:
                    # Остановка черепашки
                    turtlesim_cmd.angular.z = 0.0
                    self.publisher.publish(turtlesim_cmd)

                    goal_handle.canceled()
                    result.result = False
                    return result

                # Нахождение наименьшей разницы между двумя углами
                current_theta = self.turtlesim_pose.theta
                delta_theta = np.abs(current_theta - initial_theta) % (2 * np.pi)
                print('\n======\n', self.turtlesim_pose, initial_theta, current_theta, delta_theta, '\n======\n',)

                self.publisher.publish(turtlesim_cmd)

                # По какой-то причине, одометрия имеет тип int32,
                # поэтому результат округляем до целого числа:
                feedback.odom = int(np.abs(delta_theta) * 180 / np.pi) 
                self.get_logger().info('Отправляю фидбэк об одометрии угла клиенту...')
                print('\n======\n', feedback.odom, feedback.odom < np.abs(angle_rad), '\n======\n')
                goal_handle.publish_feedback(feedback)

                # rate.sleep()
                time.sleep(1.5)


        # Остановка черепашки
        turtlesim_cmd.linear.x = 0.0
        turtlesim_cmd.angular.z = 0.0
        self.publisher.publish(turtlesim_cmd)

        goal_handle.succeed()
        result.result = True
        self.get_logger().info('Гоооол обработан! Отправляю результат обратно клиенту...\n')
        return result


def main(args=None):
    rclpy.init(args=args)
    tc_server = TurtleActionServer()
    executor = MultiThreadedExecutor()
    
    executor.add_node(tc_server)
    executor.spin()

    executor.shutdown()
    tc_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
