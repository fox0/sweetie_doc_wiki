# Создание и вызов сервисов
Если необходимо передавать информацию только по запросу, то используются сервисы (services). В этом случае клиентская нода пересылает сообщение запрос (request message), а нода сервис отвечает на запрос, посылая сообщение ответ (reply message).

Наш сервис будет принимать на вход два числа и возвращать их сумму.

```
mkdir ~/ros/my_ws/src/service
cd ~/ros/my_ws/src/service
catkin_create_pkg summator_msgs std_msgs roscpp message_generation message_runtime
catkin_create_pkg summator_service roscpp
catkin_create_pkg summator_client roscpp
mkdir summator_msgs/srv
```

`summator_msgs/srv/Add2Ints.srv`
```
int64 a
int64 b
---
int64 sum
```
Этот файл описывает формат сообщений, которыми будут обмениваться наши ноды.
Он разделен на две части — запрос и ответ.

`summator_msgs/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(summator_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
)

add_service_files(
  FILES
  Add2Ints.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES summator_msgs
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

`summator_msgs/package.xml`
```xml
<?xml version="1.0"?>
<package>
  <name>summator_msgs</name>
  <version>0.0.0</version>
  <description>The summator_msgs package</description>
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

`summator_service/src/summator_service.cpp`
```cpp
#include "ros/ros.h"
// Этот файл создастся автоматически во время сборки из файла Add2Ints.srv
#include "summator_msgs/Add2Ints.h"

bool add(summator_msgs::Add2Ints::Request  &req,
         summator_msgs::Add2Ints::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("Returning [%ld + %ld = %ld]", (long int)req.a, (long int)req.b, (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```

`summator_service/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(summator_service)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  summator_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES summator_service
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(summator_service_node src/summator_service.cpp)

add_dependencies(summator_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(summator_service_node
  ${catkin_LIBRARIES}
)
```

`summator_service/package.xml`
```xml
<?xml version="1.0"?>
<package>
  <name>summator_service</name>
  <version>0.0.0</version>
  <description>The summator_service package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>summator_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>summator_msgs</run_depend>

  <export>
  </export>
</package>
```

`summator_client/src/summator_client.cpp`
```cpp
#include "ros/ros.h"
// Этот файл создастся автоматически во время сборки из файла Add2Ints.srv
#include <summator_msgs/Add2Ints.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: summator_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<summator_msgs::Add2Ints>("add_two_ints");
  summator_msgs::Add2Ints srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  ROS_INFO("Requesting %ld+%ld", (long int)srv.request.a, (long int)srv.request.b);

  if (client.call(srv))
  {
    ROS_INFO("%ld + %ld = %ld", (long int)srv.request.a, (long int)srv.request.b, (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```

`summator_client/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(summator_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  summator_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES summator_client
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(summator_client_node src/summator_client.cpp)

add_dependencies(summator_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(summator_client_node
  ${catkin_LIBRARIES}
)
```

`summator_client/package.xml`
```xml
<?xml version="1.0"?>
<package>
  <name>summator_client</name>
  <version>0.0.0</version>
  <description>The summator_client package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>summator_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>summator_msgs</run_depend>

  <export>
  </export>
</package>
```

```
.
├── summator_client
│   ├── CMakeLists.txt
│   ├── include
│   │   └── summator_client
│   ├── package.xml
│   └── src
│       └── summator_client.cpp
├── summator_msgs
│   ├── CMakeLists.txt
│   ├── include
│   │   └── summator_msgs
│   ├── package.xml
│   ├── src
│   └── srv
│       └── Add2Ints.srv
└── summator_service
    ├── CMakeLists.txt
    ├── include
    │   └── summator_service
    ├── package.xml
    └── src
        └── summator_service.cpp
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
# rosrun summator_service summator_service_node
[ INFO] [1472376080.566648113]: Ready to add two ints.
```
```
# rosrun summator_client summator_client_node 1 1
[ INFO] [1472376098.419337334]: Requesting 1+1
[ INFO] [1472376098.420931418]: 1 + 1 = 2
```
