### Пример 1. Передача данных через порт
Для начала пример «чистых» OROCOS пакетов без интеграции с ROS.

По аналогии с ROS, для упрощения создания пакетов существует команда orocreate-catkin-pkg
```
orocreate-catkin-pkg <имя пакета> [тип пакета]
```
Создаем два пакета-компонента. Первый будет отправлять данные второму.
```
$ cd ~/catkin_ws/src/
$ mkdir -p orocos_tests/example1
$ cd orocos_tests/example1
$ orocreate-catkin-pkg sender component
Using templates at /opt/ros/indigo/share/ocl/templates...
Package sender created in directory /home/user/catkin_ws/src/orocos_tests/example1/sender
$ orocreate-catkin-pkg receiver component
Using templates at /opt/ros/indigo/share/ocl/templates...
Package receiver created in directory /home/user/catkin_ws/src/orocos_tests/example1/receiver
```
Команда orocreate-catkin-pkg уже создала cpp и hpp файлы, подредактируем их немного.

`sender/src/sender-component.cpp`
```cpp
#include "sender-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Sender::Sender(std::string const& name) : TaskContext(name){
  std::cout << "Sender constructed !" <<std::endl;
  // Инициализируем выходной порт
  this->ports()->addPort( "outPort", outPort ).doc( "Output Port, here write our data to." );
}

bool Sender::configureHook(){
  std::cout << "Sender configured !" <<std::endl;
  // Сюда надо помещать все что относится к конфигурации компонента
  // чтение параметров, инициализация, выделение ресурсов и т.д.
  // В данном случае задаем тут периодический таска:
  return this->setPeriod(0.1); // updateHook будет выполняться с периодом 10Hz.
}

bool Sender::startHook(){
  std::cout << "Sender started !" <<std::endl;
  // По умолчанию компопнет считается остановленым.
  // Сюда необходимо поместить код который будет
  // переводить компонент в активное состояние.
  return true;
}

void Sender::updateHook(){
  std::cout << "Sender executes updateHook !" <<std::endl;
  // Тут происходит основная работа компонента.
  // Отправляем данные на выходнйо порт
  outPort.write( 2.0 );
}

void Sender::stopHook() {
  std::cout << "Sender executes stopping !" <<std::endl;
  // Для кода остановки компонента.
}

void Sender::cleanupHook() {
  std::cout << "Sender cleaning up !" <<std::endl;
  // Освобождение ресурсов перед завершением.
}

ORO_CREATE_COMPONENT(Sender)
```

`sender/src/sender-component.hpp`

```cpp
#ifndef OROCOS_SENDER_COMPONENT_HPP
#define OROCOS_SENDER_COMPONENT_HPP

#include <rtt/RTT.hpp>

using namespace RTT;

class Sender : public RTT::TaskContext{
    // Атрибут класса хранящий объект выходного порта
    // В скобках указан тип передаваемого сообщения
    OutputPort<double> outPort;

  public:
    Sender(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
```

`receiver/src/receiver-component.cpp`
```cpp
#include "receiver-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Receiver::Receiver(std::string const& name) : TaskContext(name){
  std::cout << "Receiver constructed !" <<std::endl;
  // Если инициализировать входной порт при помощи метода addEventPort,
  // то каждый раз когда на порт будут приходить данные, будет срабатывать updateHook.
  this->ports()->addEventPort( "inPort", inPort ).doc( "Input Port that raises an event." );
}

bool Receiver::configureHook(){
  std::cout << "Receiver configured !" <<std::endl;
  // Сюда надо помещать все что относится к конфигурации компонента
  // чтение параметров, инициализация, выделение ресурсов и т.д.
  return true;
}

bool Receiver::startHook(){
  std::cout << "Receiver started !" <<std::endl;
  // По умолчанию компопнет считается остановленым.
  // Сюда необходимо поместить код который будет
  // переводить компонент в активное состояние.
  return true;
}

void Receiver::updateHook(){
  // Тут происходит основная работа компонента.
  // В данном случае читаем данные.
  double result;
  switch ( inPort.read(result) )
  {
    case NoData:  std::cout << "NO data received !" <<std::endl;
                  break;
    case OldData: std::cout << "OLD data received !" <<std::endl;
                  break;
    case NewData: std::cout << "New data received : " << result <<std::endl;
                  break;
  }
}

void Receiver::stopHook() {
  std::cout << "Receiver executes stopping !" <<std::endl;
  // Для кода остановки компонента.
}

void Receiver::cleanupHook() {
  std::cout << "Receiver cleaning up !" <<std::endl;
  // Освобождение ресурсов перед завершением.
}

ORO_CREATE_COMPONENT(Receiver)
```

`receiver/src/receiver-component.hpp`
```cpp
#ifndef OROCOS_RECEIVER_COMPONENT_HPP
#define OROCOS_RECEIVER_COMPONENT_HPP

#include <rtt/RTT.hpp>

using namespace RTT;

class Receiver : public RTT::TaskContext{
    // Атрибут класса хранящий объект входного порта
    // В скобках указан тип передаваемого сообщения
    InputPort<double> inPort;

  public:
    Receiver(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
```

Создадим скрипты запуска и конфигурационные файлы. **Примечание:** Их можно создать в любом пакете, или же создать отдельный пакет для конфигов и скриптов запуска. В данном примере мы создадим их в пакете receiver.
```
$ mkdir -p receiver/scripts receiver/config receiver/launch
```
`receiver/config/start.ops`
```sh
# Импортируем оба компонента
import("receiver");
import("sender");

# Данная функция выведет список импортированных компонентов
displayComponentTypes();

# Создадим экземпляры классов каждого компонента
loadComponent( "my_receiver", "Receiver" );
loadComponent( "my_sender", "Sender" );

# Соединим выходной и входной порты
connect( "my_sender.outPort", "my_receiver.inPort", ConnPolicy() );

# Конфигурация
my_receiver.configure();
my_sender.configure();

# Запуск
my_receiver.start();
my_sender.start();
```

`receiver/launch/start.launch`
```xml
<launch>
  <node name="send_recv" pkg="rtt_ros" type="deployer" args="-s $(find receiver)/config/start.ops --" output="screen">
  </node>
</launch>
```

`receiver/scripts/start.sh`
```sh
#!/bin/sh
BASEDIR=$(dirname "$0")
deployer-gnulinux -s $BASEDIR/../config/start.ops -linfo
```
```
$ chmod a+x receiver/scripts/start.sh
```
![](https://i.imgur.com/ghRwfmK.png)

Проверяем:
```
$ receiver/scripts/start.sh
```
или
```
$ rosrun receiver start.sh
```
или
```
$ roslaunch receiver start.launch
```
В последнем случае будет запущено ядро ROS, хотя сами компоненты никак его не используют.

Помимо прочего вывод должен содержать повторяющиеся строки:
```
Sender executes updateHook !
New data received : 2
Sender executes updateHook !
New data received : 2
...
```