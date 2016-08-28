### Пример 2. Операции
Если нужно не только передать какие данные, но и получить в ответ другие данные используются операции — аналог сервисов из ROS.
Создаем два пакета-компонента. Первый будет отправлять данные второму.
```
$ cd ~/catkin_ws/src/
$ mkdir -p orocos_tests/example2
$ cd orocos_tests/example2
$ orocreate-catkin-pkg summer_client component
Using templates at /opt/ros/indigo/share/ocl/templates...
Package summer_client created in directory /home/ignat/catkin_ws/src/orocos_tests/example2/summer_client
$ orocreate-catkin-pkg summer_server component
Using templates at /opt/ros/indigo/share/ocl/templates...
Package summer_server created in directory /home/ignat/catkin_ws/src/orocos_tests/example2/summer_server
```

`summer_client/src/summer_client-component.cpp`
```cpp
#include "summer_client-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Summer_client::Summer_client(std::string const& name) :
  TaskContext(name),
  summer_caller("sum_two_doubles_caller")
{
  this->requires()->addOperationCaller(summer_caller);
  std::cout << "Summer_client constructed !" <<std::endl;
}

bool Summer_client::configureHook(){
  std::cout << "Summer_client configured !" <<std::endl;
  this->setPeriod(1);
  return summer_caller.ready();
}

bool Summer_client::startHook(){
  std::cout << "Summer_client started !" <<std::endl;
  return true;
}

void Summer_client::updateHook(){
  std::cout << "Summer_client executes updateHook !" <<std::endl;
  double a = rand() % 100;
  double b = rand() % 100;
  log(Info)<<"summer_caller(" << a << ", " << b << ") = " << summer_caller.call(a,b) <<endlog();
}

void Summer_client::stopHook() {
  std::cout << "Summer_client executes stopping !" <<std::endl;
}

void Summer_client::cleanupHook() {
  std::cout << "Summer_client cleaning up !" <<std::endl;
}

ORO_CREATE_COMPONENT(Summer_client)
```

`summer_client/src/summer_client-component.hpp`
```cpp
#ifndef OROCOS_SUMMER_CLIENT_COMPONENT_HPP
#define OROCOS_SUMMER_CLIENT_COMPONENT_HPP

#include <rtt/RTT.hpp>

using namespace RTT;

class Summer_client : public RTT::TaskContext{
  protected:
    OperationCaller<double(double,double)> summer_caller;
  public:
    Summer_client(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
```

`summer_server/src/summer_server-component.cpp`
```cpp
#include "summer_server-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Summer_server::Summer_server(std::string const& name) : TaskContext(name){
  addOperation( "sum_two_doubles", &Summer_server::summer, this, OwnThread)
                                                        .doc("Sum two doubles")
                                                        .arg("Value A", "First value")
                                                        .arg("Value B", "Second value");

  std::cout << "Summer_server constructed !" <<std::endl;
}

double Summer_server::summer(double a, double b) {
  log(Info)<<"Summer_server::summer()"<<endlog();
  return a+b;
}

bool Summer_server::configureHook(){
  std::cout << "Summer_server configured !" <<std::endl;
  return true;
}

bool Summer_server::startHook(){
  std::cout << "Summer_server started !" <<std::endl;
  return true;
}

void Summer_server::updateHook(){
  std::cout << "Summer_server executes updateHook !" <<std::endl;
}

void Summer_server::stopHook() {
  std::cout << "Summer_server executes stopping !" <<std::endl;
}

void Summer_server::cleanupHook() {
  std::cout << "Summer_server cleaning up !" <<std::endl;
}

ORO_CREATE_COMPONENT(Summer_server)
```

`summer_server/src/summer_server-component.hpp`
```cpp
#ifndef OROCOS_SUMMER_SERVER_COMPONENT_HPP
#define OROCOS_SUMMER_SERVER_COMPONENT_HPP

#include <rtt/RTT.hpp>

using namespace RTT;

class Summer_server : public RTT::TaskContext{
  public:
    Summer_server(std::string const& name);
    double summer(double a, double b);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
```
Создадим скрипты запуска и конфигурационные файлы. **Примечание:** Их можно создать в любом компоненте, или же создать отдельный компонент для конфигов и скриптов запуска. В данном примере мы создадим их в пакете summer_server.
```
mkdir -p summer_server/config summer_server/launch summer_server/scripts
```
`summer_server/config/start.ops`
```sh
# Импортируем оба компонента
import("summer_server");
import("summer_client");

# Данная функция выведет список импортированных компонентов
displayComponentTypes();

# Создадим экземпляры классов каждого компонента
loadComponent("summer_server", "Summer_server");
loadComponent("summer_client", "Summer_client");

# Соединим выходной и входной порты
connectPeers("summer_server", "summer_client")
connectOperations("summer_client.sum_two_doubles_caller", "summer_server.sum_two_doubles");

# Конфигурация
summer_server.configure()
summer_client.configure()

# Запуск
summer_server.start()
summer_client.start()
```
`summer_server/launch/start.launch`
```xml
<launch>
  <node name="send_recv" pkg="rtt_ros" type="deployer" args="-s $(find summer_server)/config/start.ops --" output="screen">
  </node>
</launch>
```
`summer_server/scripts/start.sh`
```sh
#!/bin/sh
BASEDIR=$(dirname "$0")
deployer-gnulinux -s $BASEDIR/../config/start.ops -linfo
```
```
$ chmod a+x summer_server/scripts/start.sh
```
![](http://i.imgur.com/gbirNZZ.png)

Компилируем:
```
$ cd ~/catkin_make
$ catkin_make
```
Проверяем:
```
$ rosrun summer_server start.sh
```
или
```
$ roslaunch summer_server start.launch
```
В последнем случае будет запущено ядро ROS, хотя сами компоненты никак его не используют.
Помимо прочего вывод должен содержать повторяющиеся строки:
```
Summer_client executes updateHook !
1.080 [ Info   ][Logger] Summer_server::summer()
Summer_server executes updateHook !
1.080 [ Info   ][Logger] summer_caller(83, 86) = 169
Summer_client executes updateHook !
2.080 [ Info   ][Logger] Summer_server::summer()
Summer_server executes updateHook !
2.080 [ Info   ][Logger] summer_caller(77, 15) = 92
...
```