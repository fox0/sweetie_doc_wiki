## Добавим интеграцию для второго примера. 

Но на этот раз создадим пакет summer_msgs, в котором будет храниться `.srv` файл с типом сообщения сервиса.

Также, для этого пакета необходимо создать rtt_ пакет, который будет подгружать этот тип в OROCOS компоненты.
Это Можно сделать с помощью специального скрипта из пакета rtt_roscomm:  `create_rtt_msgs`
```
$ catkin_create_pkg summer_msgs message_generation message_runtime std_msgs
Created file summer_msgs/CMakeLists.txt
Created file summer_msgs/package.xml
Successfully created files in /home/user/catkin_ws/src/orocos_tests/sweetie_bot_core/summer_msgs. Please adjust the values in package.xml.
$ rosrun rtt_roscomm create_rtt_msgs summer_msgs
Using templates at /opt/ros/indigo/share/rtt_roscomm/rtt_roscomm_pkg_template...
removed 'rtt_summer_msgs/CATKIN_IGNORE'
Successfully created rtt_summer_msgs.
$ mkdir summer_msgs/srv
```
`summer_msgs/srv/MySum.srv`
```
float64 a
float64 b
---
float64 c
```
В файле `summer_msgs/CMakeLists.txt` добавляем/раскомментариваем следующие строки:
```
add_service_files(
  FILES
  MySum.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```
`summer_client/package.xml`
```diff
@@ -13,9 +13,18 @@
 
   <build_depend>rtt</build_depend>
   <build_depend>orogen</build_depend>
+  <build_depend>rtt_ros</build_depend>
+  <build_depend>rtt_summer_msgs</build_depend>
+
   <run_depend>rtt</run_depend>
+  <run_depend>rtt_ros</run_depend>
+  <run_depend>rtt_roscomm</run_depend>
 
   <export>
-
+    <rtt_ros>
+      <plugin_depend>rtt_rosnode</plugin_depend>
+      <plugin_depend>rtt_roscomm</plugin_depend>
+      <plugin_depend>rtt_summer_msgs</plugin_depend>
+    </rtt_ros>
   </export>
 </package>
```
`summer_client/src/summer_client-component.cpp`
```diff
@@ -23,9 +23,14 @@ bool Summer_client::startHook(){
 
 void Summer_client::updateHook(){
   std::cout << "Summer_client executes updateHook !" <<std::endl;
-  double a = rand() % 100;
-  double b = rand() % 100;
-  log(Info)<<"summer_caller(" << a << ", " << b << ") = " << summer_caller.call(a,b) <<endlog();
+  summer_msgs::MySum::Request req;
+  summer_msgs::MySum::Response resp; 
+  req.a = rand() % 100;
+  req.b = rand() % 100;
+  if( summer_caller.call(req, resp) )
+    log(Info)<<"summer_caller(" << req.a << ", " << req.b << ") = " << resp.c <<endlog();
+  else
+    log(Info)<<"summer_caller(" << req.a << ", " << req.b << ") = false! " << resp.c <<endlog();
 }
```
`summer_client/src/summer_client-component.hpp`
```diff
@@ -2,12 +2,15 @@
 #define OROCOS_SUMMER_CLIENT_COMPONENT_HPP
 
 #include <rtt/RTT.hpp>
+#include <ros/ros.h>
+#include <std_msgs/Float64.h>
+#include <summer_msgs/MySum.h>
 
 using namespace RTT;
 
 class Summer_client : public RTT::TaskContext{
   protected:
-    OperationCaller<double(double,double)> summer_caller;
+    OperationCaller<bool(summer_msgs::MySum::Request&, summer_msgs::MySum::Response&)> summer_caller;
   public:
     Summer_client(std::string const& name);
     bool configureHook();
```
`summer_server/package.xml`
```diff
@@ -13,9 +13,18 @@
 
   <build_depend>rtt</build_depend>
   <build_depend>orogen</build_depend>
+  <build_depend>rtt_ros</build_depend>
+  <build_depend>rtt_summer_msgs</build_depend>
+
   <run_depend>rtt</run_depend>
+  <run_depend>rtt_ros</run_depend>
+  <run_depend>rtt_roscomm</run_depend>
 
   <export>
-
+    <rtt_ros>
+      <plugin_depend>rtt_rosnode</plugin_depend>
+      <plugin_depend>rtt_roscomm</plugin_depend>
+      <plugin_depend>rtt_summer_msgs</plugin_depend>
+    </rtt_ros>
   </export>
 </package>
```
`summer_server/src/summer_server-component.cpp`
```diff
@@ -11,9 +11,10 @@ Summer_server::Summer_server(std::string const& name) : TaskContext(name){
   std::cout << "Summer_server constructed !" <<std::endl;
 }
 
-double Summer_server::summer(double a, double b) {
+bool Summer_server::summer(summer_msgs::MySum::Request& req, summer_msgs::MySum::Response& resp){
   log(Info)<<"Summer_server::summer()"<<endlog();
-  return a+b;
+  resp.c = req.a + req.b;
+  return true;
 }
```
`summer_server/src/summer_server-component.hpp`
```diff
@@ -2,13 +2,16 @@
 #define OROCOS_SUMMER_SERVER_COMPONENT_HPP
 
 #include <rtt/RTT.hpp>
+#include <ros/ros.h>
+#include <std_msgs/Float64.h>
+#include <summer_msgs/MySum.h>
 
 using namespace RTT;
 
 class Summer_server : public RTT::TaskContext{
   public:
     Summer_server(std::string const& name);
-    double summer(double a, double b);
+    bool summer(summer_msgs::MySum::Request& req, summer_msgs::MySum::Response& resp);
     bool configureHook();
     bool startHook();
     void updateHook();
```
`summer_server/config/start.ops`
```sh
# Импортируем оба компонента
import("summer_server");
import("summer_client");

# Импортируем компоненты интеграции
import("rtt_ros")

ros.import("summer_server");
ros.import("summer_client");

# Данная функция выведет список импортированных компонентов
displayComponentTypes();

# Создадим экземпляры классов каждого компонента
loadComponent("summer_server", "Summer_server");
loadComponent("summer_client", "Summer_client");

# Соединим сервер с клиентом
connectPeers("summer_server", "summer_client")
connectOperations("summer_client.sum_two_doubles_caller", "summer_server.sum_two_doubles");

# Подключение операции к сервису ROS
loadService("summer_server","rosservice");
summer_server.rosservice.connect( "sum_two_doubles", "/summer", "summer_msgs/MySum");

# Конфигурация
summer_server.configure();
summer_client.configure();

# Запуск
summer_server.start();
summer_client.start();
```
![](http://i.imgur.com/7CpeS9Z.png)

Компилируем и запускаем `roscore`
```
$ cd ~/catkin_ws
$ catkin_make
$ roscore
```
```
$ rosrun summer_server start.sh
```
Вызов сервиса средствами ROS:
```
rosservice call /summer "{a: 1.1, b: 2.2}"
c: 3.3
```