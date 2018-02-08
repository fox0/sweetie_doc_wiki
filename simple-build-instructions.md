Установка
---------

**Системные требования и рекомендации**

* Любой дистрибутив GNU/Linux на базе, например, Ubuntu 16.04 (Xenial) или Debian 8, 9.
* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation)
* [OROCOS 2.9](https://github.com/orocos-toolchain/orocos_toolchain), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
* Вспомогательные пакеты OROCOS:
    * [rtt-ros-integration 2.9](https://github.com/orocos/rtt_ros_integration), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
    * [kdl_msgs](https://github.com/orocos/kdl_msgs), [rtt_kdl_msgs](https://github.com/orocos/rtt_kdl_msgs)
    * [rttlua_completion](https://github.com/orocos-toolchain/rttlua_completion), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
    * `rtt_tf2_msgs`,`rtt_control_msgs` сгенерировать самостоятельно используя [`rtt_roscom`](https://github.com/orocos/rtt_ros_integration/tree/toolchain-2.9/rtt_roscomm)
* [Rigid Body Bynamics Library](https://rbdl.bitbucket.io/)
* [FlexBe](http://philserver.bplaced.net/fbe/)

Оставшаяся часть файла содержит инструкции по установки этих пакетов и компиляции кода `SweetieBot`.

Инструкция для разработчиков по сборке и установке проекта (**устарела, нуждается в обновлении**)
-------------------------------------------------------------------------------------------------

Актуальные инструкции можно найти здесь: https://gitlab.com/sweetiebot/compile_orocos

Системные требования и рекомендации:
* Любой дистрибутив GNU/Linux на базе Ubuntu 16.04 (Xenial). Например, Kubuntu 16.04.
* ROS Kinetic Kame

Как ставить.

1. Установить ROS Kinetic Kame (см. инструкцию в разделе "ROS Installation" на вики проекта: https://gitlab.com/sweetie-bot/sweetie_doc/wikis/ros-installation или (ещё лучше) на сайте ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu).
На десктопе:
`sudo apt-get install ros-kinetic-desktop-full`
На роботе:
`sudo apt-get install ros-kinetic-robot`

1. Удалить все ros-kinetic-rtt-* (если стоят).
`sudo apt remove 'ros-kinetic-rtt-*'`

1. Установить дополнительные пакеты командой

    ```
sudo apt-get install ros-kinetic-moveit ros-kinetic-sound-play ros-kinetic-trac-ik-lib ros-kinetic-octomap-msgs ros-kinetic-rosbridge-server ros-kinetic-leap-motion ros-kinetic-rospy-message-converter libalglib-dev lua-filesystem castxml gccxml libeditline-dev libeditline0 libgmp-dev libgmpxx4ldbl liblua5.1-0 liblua5.1-0-dev libncurses5-dev libomniorb4-1 libomniorb4-dev libomnithread3-dev libomnithread3c2 libreadline-dev libreadline6-dev libtinfo-dev libtool-bin omniidl omniorb omniorb-idl omniorb-nameserver ruby-dev ruby-facets ruby-hoe ruby-nokogiri ruby2.3-dev libxml-xpath-perl
```

1. Скачать кастомные пакеты с ftp://xq3.ru/ros

    ```
wget ftp://xq3.ru/ros/ros-kinetic-orocos-toolchain_2.9.0-1_amd64.deb
wget ftp://xq3.ru/ros/ros-kinetic-rtt-ros-integration_2.9.0-1_amd64.deb
```

1. Установить их командой 
    ```
sudo dpkg -i ros-kinetic-orocos-toolchain_2.9.0-1_amd64.deb
sudo dpkg -i ros-kinetic-rtt-ros-integration_2.9.0-1_amd64.deb
```

1. Заблокировать обновления этих пакетов

    ```
sudo apt-mark hold ros-kinetic-rtt-ros-integration
sudo apt-mark hold ros-kinetic-orocos-toolchain
```

1. Скачать (клонировать) основной репозиторий с кодом sweetie_bot:

    `mkdir -p ~/ros/sw/src/
cd ~/ros/sw/src/
git clone -b devel --recursive git@gitlab.com:sweetie-bot/sweetie_bot.git
`

1. Установить flexbe_behavior_engine и generic_flexbe_states (см. инструкцию на сайте)

    `git clone https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/generic_flexbe_states.git`

1. Перейти в директорию воркспейса
`cd ~/ros/sw/`

1. Собрать проект командой
`catkin_make`

Примечание 1: репозиторий code &mdash; устаревший. По некоторым причинам, он ещё есть и иногда обновляется, но если вы не знаете, зачем он, то не используйте его.

Примечание 2: Опционально для использования сторонних библиотек, компилирующихся системой catkin, может быть удобным создание специальной рабочей среды для них, которая должна быть "наслоена" (overlay) на основную. Для этого нужно выполнить следующую последовательность шагов:

`mkdir -p ~/ros/overlays/flexbe/src
source /opt/ros/kinetic/setup.bash
cd ~/ros/overlays/flexbe/src
catkin_init_workspace
cd ~/ros/overlays/flexbe/
catkin_make
source devel/setup.bash
wstool init src
`

Для использования этого оверлея в рабочей среде catkin, создавать её нужно, используя 

`source ~/ros/overlays/flexbe/devel/setup.bash`
вместо
`source /opt/ros/kinetic/setup.bash`

Далее для добавления нужного пакета используются из директории src команды вида:

`wstool set myrepo --git git://github.com/<полный путь>/myrepo.git
wstool update
cd ..
catkin_make
`

Пример установки `generic_flexbe_states` и `flexbe_behavior_engine`
```
wstool set generic_flexbe_states --git https://github.com/FlexBE/generic_flexbe_states.git
wstool set flexbe_behavior_engine --git https://github.com/team-vigir/flexbe_behavior_engine.git
wstool update
cd ..
catkin_make
```
