# Список разных проблем с газеброй и как я их фиксил, может кому-то пригодится


### Юзаю Ubuntu-based дистрибутив (т.е. НЕ саму Ubuntu, а что-то на её основе) и Gazebo не ставится вот с такой ошибкой пакетов (ниже). Что делать?

```bash
E: Репозиторий «http://packages.ros.org/ros/ubuntu ВЕРСИЯ_НЕЙМ Release» не содержит файла Release.
...
E: Невозможно найти пакет gz-harmonіc
```

Нужно немножко поменять путь до добавляемого репозитория, потому что по идее он должен был подставляется динамически, но в силу _альтернативной логики_ в команде из туториала подставляется коднейм не версии убунту, на которой данный дистрибутив основан, а коднейм самого дистрибутива! И, поэтому он, конечно же, не может ничего найти.

Например, если хотим поставить для дистра на основе Ubuntu 22.04 LTS (она же Jammy Jellyfish, или просто `jammy`)…

Вместо этого:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

Пишем вот это:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

Аналогично для 24.04-based (`noble`) и так далее…


### Ну вот газебра установилась и вроде всё прекрасно, но при попытке запустить, например, [демку diff_drive.sdf](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos#diff-drive), появляется только зависшее, полностью чёрное пустое окно. Что делать теперь?

Поставить апвоут вот этому ответу (у меня недостаточно кармы): https://robotics.stackexchange.com/questions/113152/gazebo-showing-blackscreen-after-launching-then-not-responding

>I believe the issue is that multicast is not enabled.
>
>See: https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast
>
>Basically you have to update your firewall configuration to allow multicast using ufw.
>
>```bash
>sudo ufw allow in proto udp to 224.0.0.0/4
>sudo ufw allow in proto udp from 224.0.0.0/4
>```
>
>Also, the first time running Gazebo may take some time because it may download models. Try also enabling verbose logging initially, as it can also help point as to what the issue is.


### Пакеты ros_gz не компилируются, ругаются на тонну недостающих зависимостей, и вообще у меня уже на этапе сборки не совпадают версии!

[Сами пакеты с GitHub-а](https://github.com/gazebosim/ros_gz) для Ubuntu компилировать вручную не нужно! Достаточно установить уже собранную версию, но только не `ros-rolling-ros-gz` (как в ридмишечке), а `ros-humble-ros-gz`, чтобы версия соответствовала (иначе тоже будет ± та же тонна ошибок). После этого все демки, включая модельки и launch-файлы, будут лежать в `/opt/ros/humble/share/ros_gz_sim_demos`.


### После установки газебры ros2 topic начал падать с ошибкой рантайма (ниже)… всё пропало, конфликт версий, нужно переустанаваливать ros2?

```bash
Traceback (most recent call last):
  File "/opt/ros/humble/bin/ros2", line 33, in <module>
    sys.exit(load_entry_point('ros2cli==0.18.11', 'console_scripts', 'ros2')())
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/cli.py", line 91, in main
    rc = extension.main(parser=parser, args=args)
...
  File "/usr/lib/python3.10/xmlrpc/client.py", line 668, in close
    raise Fault(**self._stack[0])
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
```

Нужно всего лишь перезапустить либо устройство/wsl, либо службу ros2 (https://github.com/ros2/rclpy/issues/1221):

>**AAA:** according to the stack, it looks like client receives the Fault from the server. can you restart the daemon to see if anything changes?
>```bash
>root@tomoyafujita:~/ros2_ws/colcon_ws# ros2 daemon stop
>The daemon is not running
>root@tomoyafujita:~/ros2_ws/colcon_ws# ros2 daemon start
>The daemon has been started
>root@tomoyafujita:~/ros2_ws/colcon_ws# ros2 daemon status
>The daemon is running
>root@tomoyafujita:~/ros2_ws/colcon_ws# ros2 topic list
>/parameter_events
>/rosout
>```
>**BBB:** With restart daemon, this issue is not reproduced now.
