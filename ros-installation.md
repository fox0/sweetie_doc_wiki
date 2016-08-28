# Установка ROS
Текущая версия [ROS Kinetic](http://wiki.ros.org/ROS/Installation) пока не поддерживает все нужные нам функции, поэтому решено было пока остаться на [ROS Indigo](http://wiki.ros.org/indigo).
Подробна установка описана [тут](http://wiki.ros.org/indigo/Installation). Ниже приведена краткая инструкция по установке Indigo на Ubuntu 14.04 LTS.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall
```

# Конфигурирование окружения ROS
По умолчанию ROS устанавливается в директорию `/opt/ros/`
Следующая команда загрузит все переменные среды необходимые для запуска и использования ROS.

```
source /opt/ros/indigo/setup.bash
```
Рекомендуется добавить её в свой .bashrc чтобы она запускалась для каждого вновь запущенного терминала:

```
echo 'source /opt/ros/indigo/setup.bash' >> ~/.bashrc
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

Подробно установка описана [тут](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

### Смотри далее:
1. [Создание пакетов использующих «публикацию/подписку»](ros-create-pub-sub)
1. [Создание и вызов сервисов](ros-create-service)
1. [Параметр сервер (Parameter Server)](ros-parameters)