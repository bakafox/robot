import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from m3e2_action.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import numpy as np
import time
import sys


class TurtleActionServer(Node):
    def __init__(self, epsilon):
        super().__init__('turtle_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turle_commands',
            self.kolbek_action
        )

        self.epsilon = epsilon
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
        self.get_logger().info('Получен гоооол: %s, %d, %d\n' %
                               (goal_handle.request.command, goal_handle.request.s, goal_handle.request.angle))

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        feedback = MessageTurtleCommands.Feedback()
        feedback.odom = 0
        result = MessageTurtleCommands.Result()

        cmd_vel = Twist()
        start_pose = self.turtlesim_pose


        if command not in ['forward', 'turn_left', 'turn_right']:
            self.get_logger().error('Получена неверная команда от клиента! Прерываю гоооол...\n')
            goal_handle.abort()
            result.result = False
            return result # Здесь и далее: мы должны вернуть результат, даже если вып-е
                          # goal-а было прервано или отменено. Иначе будет ошибка 0_o

        if not self.wait_for_turtle():
            goal_handle.abort()
            result.result = False
            return result


        # Для одометрии вынесем показатели расстояния
        current_distance = 0.0
        prev_current_distance = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_turtle(cmd_vel)
                goal_handle.canceled()
                result.result = False
                return result

            if command == 'forward':
                cmd_vel.linear.x = (0.1/self.epsilon)
                prev_current_distance = current_distance
                current_distance = np.sqrt(
                    (self.turtlesim_pose.x - start_pose.x)**2 + (self.turtlesim_pose.y - start_pose.y)**2)
                print('\n======', '\nРАССТ-Е:', current_distance, '->', s, '\n======\n')

                if (s - current_distance) < np.sqrt(self.epsilon/0.01)*0.02:
                    # Останаваливаем черепашку
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.publisher.publish(cmd_vel)
                    break

                # По какой-то причине, одометрия имеет тип int32,
                # а не float, поэтому немноооого схитрим:
                feedback.odom = int(np.abs(prev_current_distance - current_distance) * 1000000)
                goal_handle.publish_feedback(feedback)
                self.get_logger().info('Отправляю фидбэк об одометрии расстояния клиенту: %d' %
                                    (feedback.odom))

            else:
                cmd_vel.angular.z = np.radians(np.pi*(2/self.epsilon))
                if command == 'turn_left':
                    cmd_vel.angular.z = cmd_vel.angular.z * (-1)
                current_angle = np.abs(self.turtlesim_pose.theta - start_pose.theta) % (np.pi*2)
                print('\n======', '\nПОВОРОТ:', current_angle, '->', np.radians(angle) % (np.pi*2), '\n======\n')

                if np.abs(current_angle - np.radians(angle) % (np.pi*2)) < np.sqrt(self.epsilon/0.01)*0.02:
                    # Останаваливаем черепашку
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.publisher.publish(cmd_vel)
                    break

            self.publisher.publish(cmd_vel)               

            time.sleep(self.epsilon)

        goal_handle.succeed()
        result.result = True
        self.get_logger().info('Гоооол обработан! Отправляю результат обратно клиенту...\n')
        return result

    def wait_for_turtle(self):
        while not self.turtlesim_found:   
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().warn('Черепах не найден, жду 1-й публикации в /turtle1/pose...')
        return True


def main(args=None):
    rclpy.init(args=args)

    # Чем меньше, тем точнее, но тем больше запросов!
    try:
        epsilon = float(sys.argv[1])
    except:
        epsilon = 0.5

    tc_server = TurtleActionServer(epsilon)
    executor = MultiThreadedExecutor()

    executor.add_node(tc_server)
    executor.spin()

    executor.shutdown()
    tc_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
