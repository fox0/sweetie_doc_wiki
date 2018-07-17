1) Установить базовый пакет, для этого подключить репозитарий как описано тут https://github.com/slavanap/ros-build

Затем: `sudo apt-get update && sudo apt-get install ros-lunar-sweetie-bot-base`

2) Установить основной пакет, для этого заходим в главный репозитарий https://gitlab.com/sweetie-bot/sweetie_bot там находим кнопочку `CI/CD -> Pipelines`, кликаем на первую зеленую галочку в списке. Появится выпадающий список в нем выбираем свой дистрибутив, например `desktop-xenial` В открывшемся окне нажимаем `Download`. Скачается zip файл с deb пакетом внутри, разархивируем, устанавливаем его `sudo dpkg -i <имя>.deb` или просто двойным щелчком.

4) Создать директорию воркспейса:

`mkdir -p ~/ros/sweetie_bot/src`

5) Склонировать в src эти репозитории:

```
cd ~/ros/sweetie_bot/src
git clone git@gitlab.com:sweetie-bot/sweetie_bot_sounds.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_proto2_movements.git
git clone https://github.com/lucasw/rviz_textured_quads.git
```

6) Еще поведения:

```
cd ~/ros/sweetie_bot/src
mkdir flexbe && cd flexbe
git clone git@gitlab.com:sweetie-bot/sweetie_bot_flexbe_behaviors.git
git clone https://github.com/FlexBE/flexbe_app.git
git clone https://github.com/team-vigir/flexbe_behavior_engine.git
cd flexbe_behavior_engine
git fetch
git checkout feature/flexbe_app
```

7) Компилируем:

```
cd ~/ros/sweetie_bot
source /opt/ros/sweetie_bot/setup.bash
catkin_make
```

8) Для подключения воркспейса используем эту команду `source ~/ros/sweetie_bot/devel/setup.bash`
Можно прописать её в .bashrc 

`echo source ~/ros/sweetie_bot/devel/setup.bash >> ~/.bashrc`

9) Запускать в двух окнах как-то так:

`roslaunch sweetie_bot_deploy load_param.launch`

`roslaunch sweetie_bot_deploy flexbe_control.launch run_flexbe:=true`
