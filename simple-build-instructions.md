 
Инструкция для разработчиков по сборке и установке проекта.

Системные требования и рекомендации:
* Любой дистрибутив GNU/Linux на базе Ubuntu 16.04 (Xenial). Например, Kubuntu 16.04.
* ROS Kinetic Kame

Как ставить.

1. Установить ROS Kinetic Kame (см. инструкцию в разделе "ROS Installation" на вики проекта: https://gitlab.com/sweetie-bot/sweetie_doc/wikis/ros-installation или (ещё лучше) на сайте ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu).
На десктопе:
`sudo apt-get install ros-kinetic-desktop-full`
На роботе:
`sudo apt-get install ros-kinetic-robot`

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
`sudo apt remove 'ros-kinetic-rtt-*'`

1. Установить дополнительные пакеты командой

`
sudo apt-get install ros-kinetic-moveit ros-kinetic-sound-play ros-kinetic-trac-ik-lib ros-kinetic-octomap-msgs ros-kinetic-rosbridge-server ros-kinetic-leap-motion libalglib-dev lua-filesystem castxml gccxml libeditline-dev libeditline0 libgmp-dev libgmpxx4ldbl liblua5.1-0 liblua5.1-0-dev libncurses5-dev libomniorb4-1 libomniorb4-dev libomnithread3-dev libomnithread3c2 libreadline-dev libreadline6-dev libtinfo-dev libtool-bin omniidl omniorb omniorb-idl omniorb-nameserver ruby-dev ruby-facets ruby-hoe ruby-nokogiri ruby2.3-dev
`

1. Скачать кастомные пакеты с https://yadi.sk/d/4B_tNdTw3P73Md или ftp://xq3.ru/ros  
`
ros-kinetic-orocos-toolchain_2.9.2-1~sweetiebot1_amd64.deb
ros-kinetic-rtt-ros-integration_2.9.2-1~sweetiebot1_amd64.deb
`
Для робота:
`
ros-kinetic-orocos-toolchain_2.9.2-1~sweetiebot1_amd64.deb
ros-kinetic-rtt-ros-integration_2.9.2-1~sweetiebot1_amd64.deb
`

1. Установить их командой `dpkg -i <имя пакета>`

1. Скачать (клонировать) основной репозиторий с кодом sweetie_bot:
`
mkdir -p ~/ros/sw/src/
cd ~/ros/sw/src/
git clone --recursive git@gitlab.com:sweetie-bot/sweetie_bot.git
`

1. Установить flexbe_behavior_engine и generic_flexbe_states (см. инструкцию на сайте)
`
git clone https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/generic_flexbe_states.git
`
    Или можно воспользоваться командами (см. пункт 2)
`
wstool set generic_flexbe_states --git https://github.com/FlexBE/generic_flexbe_states.git
wstool set flexbe_behavior_engine --git git://github.com/team-vigir/flexbe_behavior_engine.git
wstool update
cd ..
catkin_make
`

1. Перейти в директорию воркспейса
`cd ~/ros/sw/`

1. Собрать проект командой
`catkin_make`

Замечание: репозиторий code &mdash; устаревший. По некоторым причинам, он ещё есть и иногда обновляется, но если вы не знаете, зачем он, то не используйте его.

АЛЬТЕРНАТИВНЫЙ ПУТЬ (автоматический, поэтому пока не отлажен &mdash; рекомендуется не использовать):

Вызвать скрипт (TODO: указать, где скрипт взять)

`sudo install_dependencies.bash`