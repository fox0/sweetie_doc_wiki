Установка
=========

## Project dependencies

Internal dependencies:

* [`sweetie_bot_proto2_movements`](https://gitlab.com/sweetie-bot/sweetie_bot_proto2_movements) --- stored movements for Proto2,
* [`sweetie_bot_sounds`](https://gitlab.com/sweetie-bot/sweetie_bot_sounds) --- sound package.

External dependencies:

* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) or later. We need following packages:
    * `ros_base` --- basic ROS installation.
	* MoveIt! packages.
    * `eigen-conversions`, `tf-conversions`, `interactive-markers`.
	* `orocos_kdl` and `track_ik` kinematics packages.
	* [`rospy_message_converter`](https://github.com/baalexander/rospy_message_converter)
	* [`rviz_textured_quads`](https://github.com/lucasw/rviz_textured_quads)
* [OROCOS 2.9](https://github.com/orocos-toolchain/orocos_toolchain), it is recommended slightly modified version from [here](https://github.com/disRecord) with improved lua completion. Also ROS package may be used but it may have some limitation. 
* Additional ROS packages
    * [`rtt-ros-integration` 2.9](https://github.com/orocos/rtt_ros_integration) (You may use ROS packages).
    * [`kdl_msgs`](https://github.com/orocos/kdl_msgs), [rtt_kdl_msgs](https://github.com/orocos/rtt_kdl_msgs), [`rtt_geometry`](https://github.com/orocos/rtt_geometry).
    * [`rttlua_completion`](https://github.com/orocos-toolchain/rttlua_completion), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
    * `rtt_tf2_msgs`,`rtt_control_msgs` typekit packages can be generated with [`rtt_roscom`](https://github.com/orocos/rtt_ros_integration/tree/toolchain-2.9/rtt_roscomm)
* [Rigid Body Bynamics Library 2.5](https://rbdl.bitbucket.io/). Note that 2.6 version is not supported due changed quaternion semantic.
* [ALGLIB library](http://www.alglib.net) you may use package `libalglib-dev`)
* [FlexBe](http://philserver.bplaced.net/fbe/) behavior framework.
	* [`flexbe_behavior_engine`](https://github.com/team-vigir/flexbe_behavior_engine/tree/feature/flexbe_app), specifically `feature/flexbe_app` branch for `felxbe_app` support.
	* [`flexbe_app`](https://github.com/FlexBE/flexbe_app).
    * [`felexbe_general_states`](https://github.com/FlexBE/generic_flexbe_states).
* QT5 development packages (`libqt5-dev`).

## Installation from binary packages

We have repository with binary packages for Ubuntu 16.04, Debian 9 Stretch and Raspbian. 

Add apt keys
```
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 5523BAEEB01FA116
$ wget -O - https://raw.githubusercontent.com/slavanap/ros-build/master/slavanap.key | sudo apt-key add -
```

Add ROS and Sweetie Bot repositories
```
$ sudo -i
# echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# echo "deb http://sweetie.bot/apt $(lsb_release -sc) main" > /etc/apt/sources.list.d/sweetie-bot.list
```
Install binary packages
```
$ sudo apt-get update
$ sudo apt-get install ros-lunar-sweetie-bot ros-lunar-sweetie-bot-base
```
Note that `ros-lunar-sweetie-bot-base` package conflicts with OROCOS toolchain ROS packages.
Sweetie Bot specific software is installed in `/opt/ros/sweetie_bot` directory. 

Install `sweetie_bot_sounds`, `sweetie_bot_proto2_movements` and `sweetie_bot_flexbe_behaviors`: 
```
$ mkdir -p ~/ros/sweetie_bot/src
$ cd ~/ros/sweetie_bot/src
$ git clone git@gitlab.com:sweetie-bot/sweetie_bot_sounds.git
$ git clone git@gitlab.com:sweetie-bot/sweetie_bot_proto2_movements.git
$ git clone git@gitlab.com:sweetie-bot/sweetie_bot_flexbe_behaviors.git
```
Due to software bugs `flexbe_app` and `rviz_textured_quads` should be also installed in workspace:
```
$ git clone https://github.com/lucasw/rviz_textured_quads.git inc/rviz_textured_quads
$ git clone https://github.com/FlexBE/flexbe_app.git
```
Compile workspace
```
$ cd ~/ros/sweetie_bot
$ source /opt/ros/sweetie_bot/setup.bash
$ catkin_make
```

Then in another console setup ROS environment 
```
$ source /opt/ros/sweetie_bot/setup.bash
```
and launch Sweetie Bot control software as described in Usage section.

## Installation from sources

Your may compile Sweetie Bot manually in ROS workspace. This method does not conflicts with installation from binary packages due to ROS overlay mechanism.
Let's assume that all build requirements are satisfied. Or you can install them from binary package `ros-lunar-sweetie-bot-base`.

Create ROS workspace:
```
mkdir -p ~/ros/sweetie_bot/src 
```

Clone dependencies if necessary and generate typekit packages if they not installed. 
If you are using `ros-lunar-sweetie-bot-base` package only FlexBe and `rviz_textured_quads` are needed.
```
cd ~/ros/sweetie_bot/src; mkdir inc; cd inc
git clone https://github.com/lucasw/rviz_textured_quads.git
git clone -b feature/flexbe_app https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/flexbe_app.git
```
Due to the bug (quads are always black) it is recommended to install `rviz_textured_quads` in Sweetie Bot workspace.

Clone SweetieBot sources:
```
cd ~/ros/sweetie_bot/src
git clone -b devel --recursive git@gitlab.com:sweetie-bot/sweetie_bot.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_sounds.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_proto2_movements.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_flexbe_behaviors.git
mkdir msgs; cd msgs;
git clone https://github.com/orocos/rtt_kdl_msgs
git clone https://github.com/orocos/kdl_msgs.git
git clone https://github.com/orocos/rtt_geometry.git
rosrun rtt_roscomm create_rtt_msgs control_msgs
rosrun rtt_roscomm create_rtt_msgs tf2_msgs
```
Compile:
```
source /opt/ros/lunar/setup.bash
cd ~/ros/sweetie_bot
catkin_make
``` 
## Full installation from sources 

On some platforms ROS and OROCOS are not available, so almost all dependencies should be compiled and installed manulally.
Use this [repository](https://github.com/slavanap/ros-build) and this Docker images [here](https://hub.docker.com/r/slavanap/ros-build/tags/).
Also you can find instructions about compiling and crosscompiling OROCOS [here](https://gitlab.com/sweetiebot/compile_orocos).

## Build status

### Base package

Platform        | Status
----------------|--------------
Desktop         | [![Build Status](https://travis-ci.org/slavanap/ros-build.svg?branch=master)](https://travis-ci.org/slavanap/ros-build)
Raspberry Pi 3  | [![Build Status](https://travis-ci.org/slavanap/ros-build.svg?branch=rpi3)](https://travis-ci.org/slavanap/ros-build/branches)

### Main package

[![Build Status](https://gitlab.com/sweetie-bot/sweetie_bot/badges/devel/build.svg)](https://gitlab.com/sweetie-bot/sweetie_bot/pipelines)


<!--FG
Список зависимостей проекта
----------------------------

* Любой дистрибутив GNU/Linux на базе, например, Ubuntu 16.04 (Xenial) или Debian 8, 9.
* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation)
* [OROCOS 2.9](https://github.com/orocos-toolchain/orocos_toolchain), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
* Вспомогательные пакеты OROCOS:
    * [rtt-ros-integration 2.9](https://github.com/orocos/rtt_ros_integration), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
    * [kdl_msgs](https://github.com/orocos/kdl_msgs), [rtt_kdl_msgs](https://github.com/orocos/rtt_kdl_msgs)
    * [rttlua_completion](https://github.com/orocos-toolchain/rttlua_completion), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
    * `rtt_tf2_msgs`,`rtt_control_msgs` сгенерировать самостоятельно используя [`rtt_roscom`](https://github.com/orocos/rtt_ros_integration/tree/toolchain-2.9/rtt_roscomm)
* [Rigid Body Bynamics Library](https://rbdl.bitbucket.io/)
* [FlexBe](http://philserver.bplaced.net/fbe/): [flexbe_behavior_engine](https://github.com/team-vigir/flexbe_behavior_engine.git) c поддержкой [FlexBe APP](https://github.com/FlexBE/flexbe_app) (смотрите описание для деталей устаноки)

В https://gitlab.com/sweetiebot/compile_orocos находится инструкция по самостоятельной сборке OROCOS и его вспомогательных пакетов. Там же описан процесс кросскомпиляции
для бортового компьютера Свити. 


Инструкция для разработчиков по сборке и установке проекта
-------------------------------------------------------------------------------------------------

Данная инструкция описывает процесс развертывания рабочего места разработчика. Она позволяет собрать часть управляющего ПО предназначенного для работы на машине оператора. 
Этот же набор пакетов может быть использован для моделирования робота.  Инструкция не содержит подробную информацию о сборке части системы исполняемой на роботе и о настройке его бортового компьютера.

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
Если хотите скомпилировать эти пакеты самостоятельно или собрать их для другого дистрибутива, то отрывки, похожие на инструкцию можно найти здесь: https://gitlab.com/sweetiebot/compile_orocos

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
git clone -b devel --  recursive git@gitlab.com:sweetie-bot/sweetie_bot.git
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
catkin_make -->
```
