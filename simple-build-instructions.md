 
Инструкция для разработчиков по сборке и установке проекта.

Системные требования и рекомендации:
* Любой дистрибутив GNU/Linux на базе Ubuntu 16.04 (Xenial). Например, Kubuntu 16.04.
* ROS Kinetic Kame

Как ставить.

1. Установить ROS Kinetic Kame (см. инструкцию в разделе "ROS Installation" на вики проекта: https://gitlab.com/sweetie-bot/sweetie_doc/wikis/ros-installation или (ещё лучше) на сайте ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu).

1. (Опциональный) Для использования сторонних библиотек, компилирующихся системой catkin, может быть удобным создание специальной рабочей среды для них, которая должна быть "наслоена" (overlay) на основную. Для этого нужно выполнить следующую последовательность шагов:
`
mkdir -p ~/ros_lib_source/src
source /opt/ros/kinetic/setup.bash
cd ~/ros_lib_source/src
catkin_init_workspace
cd ~/ros_lib_source
catkin_make
source devel/setup.bash
wstool init src
`
    Для использования этого оверлея в рабочей среде catkin, создавать её нужно, используя 
`source ~/ros_lib_source/devel/setup.bash`
вместо
`source /opt/ros/kinetic/setup.bash`

    Далее для добавления нужного пакета используются из директории src команды вида:
`
wstool set myrepo --git git://github.com/<полный путь>/myrepo.git
wstool update
cd ..
catkin_make
`

1. Удалить все ros-kinetic-rtt-* (если стоят). 

1. Установить дополнительные пакеты командой
`
sudo apt-get install ros-kinetic-moveit ros-kinetic-sound-play ros-kinetic-trac-ik-lib ros-kinetic-octomap-msgs ros-kinetic-rosbridge-server ros-kinetic-leap-motion libalglib-dev lua-filesystem
`

1. Скачать с https://yadi.sk/d/4B_tNdTw3P73Md или ftp://xq3.ru/ros файлы ros-kinetic-orocos-toolchain_2.9.0-1_amd64.deb и 
ros-kinetic-rtt-ros-integration_2.9.0-1~1_amd64.deb

1. Установить их командой
`dpkg -i ros-kinetic-orocos-toolchain_2.9.0-1_amd64.deb ros-kinetic-rtt-ros-integration_2.9.0-1~1_amd64.deb`

1. Установить зависимости этих пакетов командой 
`sudo apt-get install -f`

    Замечание: эта команда устанавливает у пакетов, поставленных "вручную", зависимости, если их нет, поэтому может и пакетами, не относящимися к данной инструкции заняться.

1. Скачать (клонировать) основной репозиторий с кодом sweetie_bot:
`git clone https://gitlab.com/sweetiebot/sweetie_bot.git`

1. Установить flexbe_behavior_engine и generic_flexbe_states (см. инструкцию на сайте)
https://github.com/team-vigir/flexbe_behavior_engine.git
https://github.com/FlexBE/generic_flexbe_states.git
    Для удобства можно воспользоваться командами (см. пункт 2)
`
wstool set generic_flexbe_states --git https://github.com/FlexBE/generic_flexbe_states.git
wstool set flexbe_behavior_engine --git git://github.com/team-vigir/flexbe_behavior_engine.git
wstool update
cd ..
catkin_make
`

1. Перейти в основную ветку разработки (если ещё не на ней)
`git checkout devel`

1. Собрать проект командой
`catkin_make`

Замечание: репозиторий code &mdash; устаревший. По некоторым причинам, он ещё есть и иногда обновляется, но если вы не знаете, зачем он, то не используйте его.

АЛЬТЕРНАТИВНЫЙ ПУТЬ (автоматический, поэтому пока не отлажен &mdash; рекомендуется не использовать):

Вызвать скрипт (TODO: указать, где скрипт взять)

`sudo install_dependencies.bash`