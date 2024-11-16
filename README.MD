# Небольшой FAQ по упражнению 10


### Сборка
```bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select m2e10_t2cv
```


### Ошибка "setup.py is deprecated...", что делать?

![alt text](<setup.py install is deprecated warning shows up every time I open a terminal in VSCode - Stack Overflow.png>)


### Запуск
```bash
ros2 run m2e10_t2cv text_to_cmd_vel
```


### Примеры сообщений
```bash
# Неизвестная команда
ros2 topic pub --once /cmd_text std_msgs/msg/String "{data: 'adasd'}"

# Поворот на предустановленный угол В ГРАДУСАХ
ros2 topic pub --once /cmd_text std_msgs/msg/String "{data: 'turn_right'}"

# Движение вперёд на произвольное расстояние
ros2 topic pub --once /cmd_text std_msgs/msg/String "{data: 'move_forward 2.4'}"
```
