import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from m3e2_action.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleActionClient(Node):
    def __init__(self, cmd_array):
        super().__init__('turtle_action_client')

        self._action_client = ActionClient(
            self,
            MessageTurtleCommands,
            'message_turle_commands'
        )

        self.odom = 0 # Одометрия

        self.cmds = cmd_array
        self.curr_cmd = 0

        self.get_logger().info('Начинаю исполнение %d команд!\n' % len(self.cmds))
        self.sdelat_goal(self.cmds[self.curr_cmd]) # Вызываем в первый раз для запуска цикла


    def sdelat_goal(self, cmd):
        goal = MessageTurtleCommands.Goal()
        goal.command = cmd[0] # 'forward', 'turn_left', 'turn_right'
        goal.s = cmd[1] or 0
        goal.angle = cmd[2] or 0
        # print('\n======\n', goal.command, goal.s, goal.angle, '\n======\n')

        # Ждём наличия сервера перед попыткой отправки:
        while not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Сервер лежит, пжлст поднимите его...')

        # Определяем колбэки для фидбека и результатов,
        # которые будут вызываться сразу после отправки:
        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.kolbek_feedback
        )
        self._send_goal_future.add_done_callback(
            self.kolbek_response
        )
        self.get_logger().info('Гоооол отправлен на сервер, ждём ответ сервера...\n')


    def kolbek_response(self, future):
        goal_handle = future.result()
        # print('\n======\n', future, goal_handle, goal_handle.accepted, '\n======\n')

        if not goal_handle.accepted:
            self.get_logger().info('Сервер не принял гоооол 0_о')
            return

        self.get_logger().info('Сервер принял гоооол!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.kolbek_result)


    def kolbek_result(self, future):
        goal_handle = future.result()
        result = goal_handle.result.result
        # print('\n======\n', goal_handle, goal_handle.result, result, '\n======\n')
        
        if (result):
            self.get_logger().info('Получен результат исполнения от сервера: УСПЕХ\n')
        else:
            self.get_logger().error('Получен результат исполнения от сервера: ПРОВАЛ\n')

        self.curr_cmd += 1
        if (self.curr_cmd >= len(self.cmds)):
            self.get_logger().info('Исполнение всех команд завершено! Итоговая одометрия: %f' %
                        (self.odom))
            rclpy.shutdown() # Все команды выполнены, выходим.
        else:
            self.sdelat_goal(self.cmds[self.curr_cmd])


    def kolbek_feedback(self, future):
        feedback = future.feedback
        # print('\n======\n', future, feedback, feedback.odom, '\n======\n')

        odom_step = feedback.odom / 1000000.0
        self.odom += odom_step
        self.get_logger().info('Получено обновление данных одометрии от сервера: %f (итого: %f)' %
                               (odom_step, self.odom))
        # self.get_logger().info('Итоговая одометрия на текущий момент: %d' %
        #                        (self.odom))


def main(args=None):
    rclpy.init(args=args)

    cmd_array = [
        ['sdgsdfgfhkj', 123, 321],
        ['turn_left', 0, 180],
        ['turn_right', 0, 180],
        ['turn_left', 0, 360],
        ['forward', 4, 0], # ['forward', 2, 0], 
        ['turn_left', 0, 450], # ['turn_left', 0, 90],
        ['forward', 2, 0], # ['forward', 1, 0],
        ['turn_right', 0, 90],
        ['turn_left', 0, -180],
        ['forward', -2, 0],
    ]

    cmd_input = input('Введите первую команду в формате "КОМАНДА РАССТ-Е ПОВОРОТ" (пропуск - исп. образец): ')
    if cmd_input:
        cmd_array.clear()

        while cmd_input:
            cmd = cmd_input.split(' ')
            # print('\n======\n', cmd, '\n======\n')
            cmd_array.append([cmd[0], int(cmd[1]), int(cmd[2])])

            cmd_input = input('Введите очередную команду в этом же формате (пропуск - начать исполнение): ')


    tc_client = TurtleActionClient(cmd_array)

    rclpy.spin(tc_client)


if __name__ == '__main__':
    main()
