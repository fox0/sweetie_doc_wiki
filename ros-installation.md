# Установка ROS
Текущая используемая версия в проекте &mdash; ROS Kinetic Kame.
Подробно установка описана [тут](http://wiki.ros.org/kinetic/Installation). Ниже приведена краткая инструкция по установке kinetic на Ubuntu 16.04 LTS.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall
```

# Конфигурирование окружения ROS
По умолчанию ROS устанавливается в директорию `/opt/ros/`
Следующая команда загрузит все переменные среды необходимые для запуска и использования ROS.

```
source /opt/ros/kinetic/setup.bash
```
Рекомендуется добавить её в свой .bashrc чтобы она запускалась для каждого вновь запущенного терминала:

```
echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
```
# Создание рабочей директории (workspace)

```
mkdir -p ~/ros/my_ws/src
cd ~/ros/my_ws
catkin_make
```
В результате будут создана пустая рабочая директория.

У рабочей директории тоже есть `setup.bash`.  

Его также рекомендуется добавить его в свой .bashrc:

```
source $HOME/ros/my_ws/devel/setup.bash
echo 'source $HOME/ros/my_ws/devel/setup.bash' >> ~/.bashrc
```

Подробно создание рабочей директории описано [тут](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

### Смотри далее:
1. [Создание пакетов использующих «публикацию/подписку»](ros-create-pub-sub)
1. [Создание и вызов сервисов](ros-create-service)
1. [Параметр сервер (Parameter Server)](ros-parameters)