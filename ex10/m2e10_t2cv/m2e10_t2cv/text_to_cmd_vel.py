import rclpy
from rclpy.node import Node

from std_msgs.msg import String     # Импорт типа СООБЩЕНИЙ (не использ.)
from geometry_msgs.msg import Twist     # Импорт типа СООБЩЕНИЙ (по усл.)


class Text2Cmd_Retranslator(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
            String,     # Тип принимаемых СООБЩЕНИЙ
            'cmd_text', # Название ТОПИКА, который слушаем
            self.retranslate,   # Колбэк по получении СООБЩЕНИЯ
            10          # Хз что это (глубина истории?)
        )

        self.publisher_ = self.create_publisher(
            Twist,      # Тип публикуемых СООБЩЕНИЙ
            '/turtle1/cmd_vel',  # Название ТОПИКА, в который публикуем
            10          # Хз что это (глубина истории?)
        )

    def retranslate(self, req):
        print('\nПолучено сообщение:')
        print(f'-- {req}')

        print('\nВыполняю ретрансляцию...')
        data = req.data.split(' ')

        if (data[0] not in ['turn_right', 'turn_left', 'move_forward', 'move_backward']):
            print('\nОшибка ретрансляции -- Неизвестная команда!')
            return
    
        if (len(data) >= 2):
            try:
                data[1] = float(data[1])
            except:
                print('\nОшибка ретрансляции -- Аргумент значения должен быть числом!')
                data[1] = False
                return
        else:
            data.append(False)

        res = Twist() # Ответное СООБЩЕНИЕ типа Twist
        match data[0]:
            case 'turn_right':
                    res.angular.z = (data[1] or 85.9437) * 0.01745329251994329576923690768489
            case 'turn_left':
                    res.angular.z = (data[1] or 85.9437) * -0.01745329251994329576923690768489
            case 'move_forward':
                    res.linear.x = data[1] or 1.0
            case 'move_backward':
                    res.linear.x = data[1] or -1.0
        
        self.publisher_.publish(res) # Отправляем ответ в указанный в конструкторе ТОПИК
        print('\nРетрансляция завершена!')
        print(f'-- {res}')


def main(args=None):
    rclpy.init(args=args)

    print('Внимание-внимание, запускаю ретранслятор!')

    t2c_retranslator = Text2Cmd_Retranslator()

    rclpy.spin(t2c_retranslator)  # Крутите барабан!

    rclpy.shutdown()
    # На самом деле, ни одна строк строка ниже spin() никогда не выполнится,
    # потому что оный выполняется как бесконечный цикл. Но пусть будет...


if __name__ == '__main__':
    main()
