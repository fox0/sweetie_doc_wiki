![](http://wiki.ros.org/custom/images/ros_org.png)

# Что такое ROS?
ROS (Robot Operating System) — Операционная система для роботов — это фреймворк для программирования роботов, предоставляющий функциональность для распределённой работы. ROS обеспечивает разработчиков библиотеками и инструментами для создания приложений робототехники. ROS обеспечивает аппаратную абстракцию, предлагает драйверы устройств, библиотеки, визуализаторы, обмен сообщениями, менеджеры пакетов и многое другое.

Полную документацию и [туториалы](http://wiki.ros.org/ROS/Tutorials) можно найти в официальной [wiki](http://wiki.ros.org/). Ниже приведена краткая концепция ROS и описаны основные понятия. В следующих разделах будет рассказано как создавать, компилировать пакеты ROS и запускать их на выполнение.

# Концепция ROS
ROS предоставляет удобные инструменты для построения распределённой системы, которая разбита на несколько процессов, занимающихся каждый своим делом и общающихся между собой при помощи сообщений (message) по сети. В центре системы находится мастер (roscore, ядро), который выступает в роли прозрачного маршрутизатора, позволяя процессам-нодам (Node) найти друг друга. Непосредственный обмен данными происходит без участия мастера. 

Весь необходимый код для приема-передачи сообщений создается автоматически на этапе сборки. Надо только создать специальные файлы с расширением «.msg» и «.srv», в которых будут описаны типы передаваемых данных и включить генерацию кода.

Ноды могут общаться друг с другом при помощи двух способов — **«публикация/подписка» (publish/subscribe)** и **«запрос/ответ» (request/reply)**.

При **publish/subscribe** между нодами устанавливается однонаправленный канал обмена данными. Ноды, которые генерируют информацию должны создать топик (Topic) и опубликовать в нем сообщения.
Ноды, которым нужна данная информация, должны подписаться на эти топики, после чего они будут получать все новые сообщения. 

При помощи этого способа можно легко организовать обмен данным не только «один к одному» или «один ко многим», но и «многие к одному», «многие ко многим» потому-что публикующих нодов может быть несколько. Этот способ хорошо подходит для случая, например когда съем данных с датчика происходит в одном процессе, а обработка полученных данных в другом/других процессах. Или если необходимо организовать передачу данных от распределенных или одинаковых по логике датчиков в один центр обработки. 

Для всех сообщений (message) создаются файлы «.msg», описывающие формат сообщения и типы передаваемых данных.
```
int8 CONST_NAME=1
int32 x
int32 y
uint32 foo
string bar
time my_time
```

Если необходимо передавать информацию только по запросу, то используются **сервисы (services)**. В этом случае клиентская нода пересылает **сообщение запрос (request message)** ноду сервису, а тот отвечает на запрос, посылая **сообщение ответ (reply message)**. Оба сообщения могут содержать какие-то данные. Для каждой пары сообщений должны быть созданы файлы «.srv», формат которых похож на формат файлов «.msg». Только запрос и ответ разделяются тройным тире.
```
#request constants
int8 FOO=1
int8 BAR=2
#request fields
int8 foobar
another_pkg/AnotherMessage msg
---
#response constants
uint32 SECRET=123456
#response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```

В некоторых случаях, если требуется выполнить длительную операцию, используются экшн-серверы из пакета actionlib. **Action-сервер** это нода, которому можно передать на вход **цель (goal)** и следить за ходом выполнения этой цели, получая сообщения состояния, а по завершении он вернет конечное состояние. Выполнение в любой момент можно прервать передав экшн-серверу другую цель или отмену задания.

Формат сообщений, передающихся при этом, описываются в файле с расширением .action, разделенного на три части тройным тире.
```
# Определение цели
uint32 dishwasher_id  # Определение какого робота моющего тарелки мы хотим использовать.
---
# Результат (количество тарелок)
uint32 total_dishes_cleaned 
---
# Сообщения состояния: проценты выполнения
float32 percent_complete
```

## Граф имен ресурсов (Graph Resource Names)
Все ресурсы в ROS (ноды, параметры, топики и сервисы) формируют древовидную структуру (граф), которую принято именовать по аналогии с файловой системой. Каждый ресурс определяется пространством имен, которое он может делить с другими ресурсами. Ресурсы могут создавать другие ресурсы в своём пространстве имен, и получать доступ к ресурсам в своём или во вложенных пространствах имен.

* / (глобальное пространство имен)
* /foo
* /stanford/robot/name
* /wg/node1

Всего различают 4 типа именования base, relative, global, и private.
base

* relative/name
* /global/name
* ~private/name

По умолчанию имена разрешаются относительно пространства имен ноды. Например нода /wg/node1 имеет пространство имен /wg, поэтому имя node2 будет разрешено как /wg/node2.
Глобальное именование использовать не рекомендуется из соображений переносимости. Если при добавлении новых нодов ресурсы с одинаковыми именами начнут пересекаться, то можно легко перенести часть проекта в другое пространство имен.
Подробней об именовании [тут](http://wiki.ros.org/Names).

## Пакеты ROS (ROS Package)
Пакеты служат вместилищем кода проекта. Каждый пакет может содержать один или несколько исполняемых файлов (процессов ROS, нодов) или библиотек. Все пакеты обязаны удовлетворять следующим условиям:
* Содержать файл package.xml — конфигурационный файл ROS;
* Содержать файл CMakeLists.txt — скрипт сборщика CMake;
* В каждой директории может содержатся не больше одного пакета.
Набор файлов самого простого (пустого) пакета:

```
my_package
         └── CMakeLists.txt
         └── package.xml
```

## Рабочая директория (ROS workspace, catkin workspace)
Для того чтобы иметь возможность компилировать и запускать пакеты их надо поместить в рабочую директорию. Она имеет свою структуру:

```
ws
├── build
├── devel
└── src
    └── CMakeLists.txt
    └── project_name
        └── package_name1
        └── package_name2
        └── subfolder
            └── package_name3
            └── package_name4
```
Директории build, devel и верхний файл  CMakeLists.txt создаются автоматически при первой компиляции. Подробнее как создать workspace и пакеты описано [тут](ros-create-packages).

## Система сборки catkin
Для сборки пакетов в ROS используется система сборки под названием catkin, которая основана на CMake — кроссплатформенной системе автоматизации сборки программного обеспечения из исходного кода. Catkin пришла на замену устаревшей rosbuild. Catkin включает в себя макросы CMake и скрипты python, для расширения функциональности стандартного CMake.

### Файл package.xml
В этом файле содержится информация о пакете, имя, версия, описание, и т.д.
Пример:
```xml
<package>
  <name>foo_core</name>
  <version>1.2.4</version>
  <description>
  This package provides foo capability.
  </description>
  <maintainer email="ivana@willowgarage.com">Ivana Bildbotz</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <run_depend>message_runtime</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>

  <test_depend>python-mock</test_depend>
</package>
```
Помимо обязательных тегов файл package.xml может содержать список пакетов от которых зависит данный пакет:
* buildtool_depend - зависимости от утилит сборки (Build Tool Dependencies) — определяет системные утилиты от которых зависит сборка пакета. Обычно для сборки необходим только catkin. В случае кросс-компиляции тут могут указываться утилиты необходимые для компиляции для нужной архитектуры.
* build_depend - Зависимости времени сборки (Build Dependencies) — определяет какие пакеты нужны для сборки этого пакета. Используется только если какие-то файлы из других пакетов используются во время сборки. Например заголовочные файлы из других пакетов, линковка библиотек или другие необходимые ресурсы которые нужны во время сборки (особенно если эти пакеты добавлены в find_package() в CMake). В случае кросс-компиляции тут указываются зависимости для целевой платформы.
* run_depend - Зависимости времени выполнения (Run Dependencies) — определяет какие пакеты нужны для запуска этого пакета, или библиотек в составе этого пакета. Используется в случае если вы зависите от разделяемых библиотек или транзитивно подключаете их заголовки (главным образом когда пакеты обозначены как (CATKIN_)DEPENDS в catkin_package() в CMakeList.txt).

Полная документация по файлу package.xml [тут](http://wiki.ros.org/catkin/package.xml).

Более подробно про зависимости можно почитать [тут](http://wiki.ros.org/catkin/conceptual_overview#Dependency_Management).

### Файл CMakeLists.txt
Скрипт системы сборки CMake. Пакеты ROS могут содержать один или несколько файлов CMakeLists.txt. Формат файла стандартен для системы CMake, но содержит расширения ROS.

Ниже приведены функций, которые необходимо вызвать для правильной сборки пакета. Первые две функции являются обязательными даже для пустого пакета.  **Порядок следования важен!**
* Версия CMake (cmake_minimum_required)
* Имя пакета (project())
* В параметрах этой функции указываются зависимости (CMake или Catkin пакеты) нужные для сборки (find_package())
* Добавление файлов сообщений(Message)/сервисов(Service)/событий(Action)(add_message_files(), add_service_files(), add_action_files())
* Функция генератор сообщений(Message)/сервисов(Service)/событий(Action) (generate_messages())
* Тут указывается прочая информация для сборки (catkin_package())
* В этих функциях определяется какие библиотеки/исполняемые файлы собирать и с чем их линковать (add_library()/add_executable()/target_link_libraries())
* Правила установки (install())

Более подробно про  файл CMakeLists.txt можно почитать [тут](http://wiki.ros.org/catkin/CMakeLists.txt).

Частично переведенная официальная документация [тут](http://wiki.ros.org/ru/catkin/CMakeLists.txt).

Пример:
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(robot_brain)
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  GetPositionIK.srv
  GetPositionFK.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_brain
   CATKIN_DEPENDS  message_runtime  
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_robot_brain src/my_robot_brain.cpp)

add_dependencies(my_robot_brain
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(kinematic_solver
  ${catkin_LIBRARIES}
)
```

### Смотри далее:  [Установка](ros-installation)