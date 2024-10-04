from m2e9_fullname.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class SummFullName(Node):
    def __init__(self):
        super().__init__('service_name') # Назв. в логгере
        self.srv = self.create_service(FullNameSumService, 'fullname', self.kolbek)

    def kolbek(self, request, response):
        response.full_name = f'{request.last_name} {request.first_name}{" " + request.name if request.name else ""}'

        self.get_logger().info(
            'Запрос обработан: "%s" "%s"%s -> "%s".\n' % (
            request.last_name,
            request.first_name,
            (' "' + request.name + '"' if request.name else ''),
            response.full_name
        ))
        # print('\n======\n', request, response, '\n======\n')

        return response # Сервис автоматически отправит любое возвращённое значение


def main(args=None):
    rclpy.init(args=args)

    fullname_service = SummFullName()
    fullname_service.get_logger().info('Сервис запущен!\n')

    rclpy.spin(fullname_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
