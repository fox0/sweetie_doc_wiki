## Добавим первому примеру интеграцию с ROS.
Изменим тип передаваемого сообщения на тип сообщения ROS `std_msgs::Float64`

`sender/src/sender-component.hpp`
```diff
@@ -2,13 +2,15 @@
 #define OROCOS_SENDER_COMPONENT_HPP
 
 #include <rtt/RTT.hpp>
+#include <ros/ros.h>
+#include <std_msgs/Float64.h>
 
 using namespace RTT;
 
 class Sender : public RTT::TaskContext{
     // Атрибут класса хранящий объект выходного порта
     // В скобках указан тип передаваемого сообщения
-    OutputPort<double> outPort;
+    OutputPort<std_msgs::Float64> outPort;
 
   public:
     Sender(std::string const& name);
```
`sender/src/sender-component.cpp`
```diff
@@ -28,7 +28,9 @@ void Sender::updateHook(){
   std::cout << "Sender executes updateHook !" <<std::endl;
   // Тут происходит основная работа компонента.
   // Отправляем данные на выходнйо порт
-  outPort.write( 2.0 );
+  std_msgs::Float64 to_send;
+  to_send.data = 2.2;
+  outPort.write( to_send );
 }
 
 void Sender::stopHook() {
```
`receiver/src/receiver-component.hpp`
```diff
@@ -2,13 +2,15 @@
 #define OROCOS_RECEIVER_COMPONENT_HPP
 
 #include <rtt/RTT.hpp>
+#include <ros/ros.h>
+#include <std_msgs/Float64.h>
 
 using namespace RTT;
 
 class Receiver : public RTT::TaskContext{
     // Атрибут класса хранящий объект входного порта
     // В скобках указан тип передаваемого сообщения
-    InputPort<double> inPort;
+    InputPort<std_msgs::Float64> inPort;
 
   public:
     Receiver(std::string const& name);
```
`receiver/src/receiver-component.cpp`
```diff
@@ -27,7 +27,7 @@ bool Receiver::startHook(){
 void Receiver::updateHook(){
   // Тут происходит основная работа компонента.
   // В данном случае читаем данные.
-  double result;
+  std_msgs::Float64 result;
   switch ( inPort.read(result) )
   {
     case NoData:  std::cout << "NO data received !" <<std::endl;
```
В конфигурационный OPS файл, в команде `stream` задается имя топика с которым будет соединён порт.

`receiver/config/start.ops`
```sh
# Импортируем оба компонента
import("receiver");
import("sender");

# Импортируем компоненты интеграции
import("rtt_ros")

ros.import("receiver");
ros.import("sender");

# Данная функция выведет список импортированных компонентов
displayComponentTypes();

# Создадим экземпляры классов каждого компонента
loadComponent( "my_receiver", "Receiver" );
loadComponent( "my_sender", "Sender" );

# Соединим выходной и входной порты
connect( "my_sender.outPort", "my_receiver.inPort", ConnPolicy() );

stream("my_receiver.inPort", ros.topic("in_topic"))
stream("my_sender.outPort", ros.topic("out_topic"))

# Конфигурация
my_receiver.configure();
my_sender.configure();

# Запуск
my_receiver.start();
my_sender.start();
```
В конфигурационный файл пакетов добавятся зависимости от `rtt_ros`, `rtt_roscomm`, `rtt_std_msgs` и `std_msgs`.

`receiver/package.xml`
```diff
@@ -13,9 +13,20 @@
 
   <build_depend>rtt</build_depend>
   <build_depend>orogen</build_depend>
+  <build_depend>rtt_ros</build_depend>
+  <build_depend>std_msgs</build_depend>
+
   <run_depend>rtt</run_depend>
+  <run_depend>rtt_ros</run_depend>
+  <run_depend>rtt_roscomm</run_depend>
+  <run_depend>rtt_std_msgs</run_depend>
+  <run_depend>std_msgs</run_depend>
 
   <export>
-
+    <rtt_ros>
+      <plugin_depend>rtt_rosnode</plugin_depend>
+      <plugin_depend>rtt_roscomm</plugin_depend>
+      <plugin_depend>rtt_std_msgs</plugin_depend>
+    </rtt_ros>
   </export>
 </package>
```
`sender/package.xml`
```diff
@@ -13,9 +13,20 @@
 
   <build_depend>rtt</build_depend>
   <build_depend>orogen</build_depend>
+  <build_depend>rtt_ros</build_depend>
+  <build_depend>std_msgs</build_depend>
+
   <run_depend>rtt</run_depend>
+  <run_depend>rtt_ros</run_depend>
+  <run_depend>rtt_roscomm</run_depend>
+  <run_depend>rtt_std_msgs</run_depend>
+  <run_depend>std_msgs</run_depend>
 
   <export>
-
+    <rtt_ros>
+      <plugin_depend>rtt_rosnode</plugin_depend>
+      <plugin_depend>rtt_roscomm</plugin_depend>
+      <plugin_depend>rtt_std_msgs</plugin_depend>
+    </rtt_ros>
   </export>
 </package>
```
Теперь для работы пакетам будет требоваться наличие запущенного `roscore` .

Проверка:
```
roslaunch receiver start.launch
```
```
rostopic echo /out_topic
```
```
rostopic pub -r 10 /in_topic std_msgs/Float64 4.4
```
```
Sender executes updateHook !
New data received : data: 2.2

New data received : data: 4.4

Sender executes updateHook !
New data received : data: 2.2

New data received : data: 4.4
...
```