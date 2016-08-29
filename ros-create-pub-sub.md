# Создание пакетов использующих «публикацию/подписку»
При publish/subscribe между нодами устанавливается однонаправленный канал обмена данными.
Создадим два пакета `twilight` и `celestia`, в каждом будет по исполняемому файлу с нодами `twilight_sparkle` и `princess_celestia` соответственно. Формат передаваемого сообщения будет описан в пакете `letter_msgs`.
`twilight_sparkle` будет публиковать сообщения в топик `spike`,  а `princess_celestia` будет подписываться на этот топик и получать сообщения.

```
mkdir ~/ros/my_ws/src/pub_sub
cd ~/ros/my_ws/src/pub_sub
catkin_create_pkg letter_msgs std_msgs roscpp message_generation message_runtime
catkin_create_pkg twilight roscpp
catkin_create_pkg celestia roscpp
mkdir letter_msgs/msg
```

Этот файл описывает формат сообщений, которыми будут обмениваться наши ноды.

`letter_msgs/msg/Letter.msg`
```
string text
int64 num
```
`letter_msgs/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(letter_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
)

add_message_files(
  FILES
  Letter.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES letter_msgs
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

`letter_msgs/package.xml`
```xml
<?xml version="1.0"?>
<package>
  <name>letter_msgs</name>
  <version>0.0.0</version>
  <description>The letter_msgs package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>std_msgs</run_depend>

  <export>
  </export>
</package>
```

`twilight/src/twilight.cpp`
```cpp
#include "ros/ros.h"
// этот файл создастся автоматически из файла Letter.msg
#include "letter_msgs/Letter.h"

int main(int argc, char **argv)
{
  /**
   * Функция ros::init() инициализирует подсистему ROS.
   * Первыми двумя параметрами передаются параметры командной строки т.к. ноды ROS
   * используют свои параметры (см. http://wiki.ros.org/Nodes)
   * третьим параметром задается имя ноды, четвертым могут быть переданы опции инициализации.
   */
  ros::init(argc, argv, "twilight_sparkle");

  /**
   * При помощи NodeHandle осуществляется коммуникация с подсистемой ROS.
   * Первый созданный NodeHandle полностью инициализирует ноду,
   * а последний уничтоженный NodeHandle завершает ноду.
   */
  ros::NodeHandle n;

  /**
   * При помощи функции advertise() (англ. извещать, информировать, оповещать; уведомлять;)
   * можно сказать ROS, что мы хотим опубликовать сообщения в топик с определенным именем.
   * Функция возвращает объект Publisher, который позволяет публиковать сообщения в топик,
   * вызвая функцию publish(). Топик автоматически закроется как только все копии объекта
   * Publisher будут уничтожены. Вторым параметром передается размер буфера.
   */
  ros::Publisher spike_pub = n.advertise<letter_msgs::Letter>("spike", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    /**
     * Это объект сообщения, мы заполняем его данными и публикуем.
     */
    letter_msgs::Letter letter;

    letter.text = "Dear Princess Celestia";
    letter.num = count;

    /**
     * Функция publish() посылает сообщения. Параметром передается объект сообщения.
     * Тип сообщения должен совпадать с тем, который был указан при вызове advertise().
     */
    spike_pub.publish(letter);

    ROS_INFO("Spike sends %lu letters to princess", letter.num);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
```

`twilight/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(twilight)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  letter_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES twilight
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(twilight_node src/twilight.cpp)

add_dependencies(twilight_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(twilight_node
  ${catkin_LIBRARIES}
)
```
 
`twilight/package.xml`
```xml
<?xml version="1.0"?>
<package>
  <name>twilight</name>
  <version>0.0.0</version>
  <description>The twilight package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>letter_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>letter_msgs</run_depend>

  <export>
  </export>
</package>
```

`celestia/src/celestia.cpp`
```cpp
#include "ros/ros.h"
// этот файл создастся автоматически из файла Letter.msg
#include "letter_msgs/Letter.h"

void chatterCallback(const letter_msgs::Letter::ConstPtr& letter)
{
  ROS_INFO("A letter from Twilight: [%s] %lu", letter->text.c_str(), letter->num);
}

int main(int argc, char **argv)
{
  /**
   * Функция ros::init() инициализирует подсистему ROS.
   * Первыми двумя параметрами передаются параметры командной строки т.к. ноды ROS
   * используют свои параметры (см. http://wiki.ros.org/Nodes)
   * третим параметром задается имя ноды, четвертым могут быть переданы опции инициализации.
   */
  ros::init(argc, argv, "princess_celestia");

  /**
   * При помощи NodeHandle осуществляется коммуникация с подсистемой ROS.
   * Первый созданный NodeHandle полностью инициализирует ноду,
   * а последний уничтоженный NodeHandle завершает ноду.
   */
  ros::NodeHandle n;

  /**
   * При помощи функции subscribe() можно сказать ROS, что вы хотите получать сообщения
   * из определенного топика. Сообщения будут переданы в callback функцию chatterCallback.
   * Функция возвращает объект Subscriber, который нужно хранить пока вы не захотите отписаться.
   * Вторым параметром передается размер буфера.
  */
  ros::Subscriber spike_sub = n.subscribe("spike", 1000, chatterCallback);

  /**
   * Функция ros::spin() не дает программе завершиться, она инициализирует бесконечный цикл,
   * при этом все callback функции будут вызваны в этом же (главном) потоке.
   * Функция завершится при нажатии Ctrl-C, или если нода будет завершена из вне мастером.
   */
  ros::spin();
  return 0;
}
```

`celestia/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(celestia)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  letter_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES celestia
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(celestia_node src/celestia.cpp)

add_dependencies(celestia_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(celestia_node
  ${catkin_LIBRARIES}
)

```

`celestia/package.xml`
```xml
<?xml version="1.0"?>
<package>
  <name>celestia</name>
  <version>0.0.0</version>
  <description>The celestia package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>letter_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>letter_msgs</run_depend>

  <export>
  </export>
</package>
```

```
src
└── pub_sub
    ├── celestia
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── celestia
    │   ├── package.xml
    │   └── src
    │       └── celestia.cpp
    ├── letter_msgs
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── letter_msgs
    │   ├── msg
    │   │   └── Letter.msg
    │   ├── package.xml
    │   └── src
    └── twilight
        ├── CMakeLists.txt
        ├── include
        │   └── twilight
        ├── package.xml
        └── src
            └── twilight.cpp
```

Теперь можно скомпилировать воркспейс.

```
cd ~/ros/my_ws
catkin_make
```
И проверить. Запускаем каждую команду в отдельном терминале:

```
roscore
```
```
rosrun celestia celestia_node
```
```
rosrun twilight twilight_node
```
![pub-sub](/uploads/d9e1074e1e5b8d93435a16365a57cc82/pub-sub.png)

### Смотри также
* [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
* [Создание и вызов сервисов](ros-create-service)
* [Параметр сервер (Parameter Server)](ros-parameters)