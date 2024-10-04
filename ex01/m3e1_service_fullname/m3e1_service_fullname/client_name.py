from m2e9_fullname.srv import FullNameSumService

import rclpy
from rclpy.node import Node

import sys


class SendNameParts(Node):
    def __init__(self):
        super().__init__('client_name') # Назв. в логгере
        self.cli = self.create_client(FullNameSumService, 'fullname')

        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Сервис упал, пжлст поднимите его...\n')
        self.request = FullNameSumService.Request()

    def sdelat_zapros(self):
        # print('\n======\n', sys.argv, len(sys.argv), '\n======\n')

        if (len(sys.argv) == 4): # 3 (ФИО)
            self.request.last_name = sys.argv[1]
            self.request.first_name = sys.argv[2]
            self.request.name = sys.argv[3]
        elif (len(sys.argv) == 3): # 2 (ФИ)
            self.request.last_name = sys.argv[1]
            self.request.first_name = sys.argv[2]
        else:
            self.get_logger().error('Ошибка -- укажите фамилию, отчество (если есть) и имя.')
            sys.exit(1)
        # print('\n======\n', self.request, '\n======\n')

        self.get_logger().info(
            'Отправляю запрос: "%s" "%s"%s...\n' % (
            self.request.last_name,
            self.request.first_name,
            (' "' + self.request.name + '"' if self.request.name else '')
        ))
        self.future = self.cli.call_async(self.request) # Отправляем сформированный запрос



def main(args=None):
    rclpy.init(args=args)

    fullname_client = SendNameParts()
    fullname_client.get_logger().info('Клиент запущен!\n')

    fullname_client.sdelat_zapros()

    rclpy.spin_once(fullname_client)

    if fullname_client.future.done():
        try:
            response = fullname_client.future.result()
            # print('\n======\n', response, '\n======\n')
        except Exception as e:
            fullname_client.get_logger().warn(
                'Ошибка -- %r' % (e,))
        else:
            fullname_client.get_logger().info(
                'Пришёл ответ на запрос -- "%s"!' % (
                response.full_name
            ))

    fullname_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
