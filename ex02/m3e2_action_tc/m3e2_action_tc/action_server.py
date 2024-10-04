import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from m3e2_action.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('turtle_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turle_commands',
            self.kolbek
        )


    def kolbek(self, goal_handle):
        # print('\n======\n', goal_handle, goal_handle.request, '\n======\n')
        self.get_logger().warn('Получен гоооол: %s, %d, %d' %
                               (goal_handle.request.command, goal_handle.request.s, goal_handle.request.angle))

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        feedback = MessageTurtleCommands.Feedback()
        result = MessageTurtleCommands.Result()

        if (command not in ['forward', 'turn_left', 'turn_right']):
            self.get_logger().error('Получена неверная команда от клиента! Отклоняю гоооол...\n')
            goal_handle.abort()
            result.result = False
            return result

        # По какой-то причине, одометрия имеет тип int32,
        # поэтому результат округляем до целого числа.
        feedback.odom = int(s + angle)
        self.get_logger().info('Отправляю фидбэк об одометрии клиенту...')
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.result = True
        self.get_logger().info('Гоооол обработан! Отправляю результат обратно клиенту...\n')
        return result


def main(args=None):
    rclpy.init(args=args)

    tc_server = TurtleActionServer()

    rclpy.spin(tc_server)


if __name__ == '__main__':
    main()
