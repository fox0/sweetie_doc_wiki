## Параметр сервер (Parameter Server)
Параметрами в ROS называют конфигурационную информацию хранящуюся на параметр сервере и необходимую для работы нодов. По функциям параметр сервер сходен с конфигурационными файлами (.conf, .ini) или реестром виндоус. В рамках одной инстанции ROS все ноды могут получать прозрачный, централизованный доступ к параметрам (создание, изменение, удаление).

Задать значение параметра можно несколькими способами:
* В launch файлах через теги «param» и «rosparam»;
* Через параметры командной строки при запуске нода;
* В процессе выполнения самих нодов (в коде);
* Используя утилиту командной строки «rosparam».

Параметр сервер не рассчитан на большую нагрузку, слишком частое считывание/изменение параметров может привести к перегрузке сервера и ядра ROS. Для часто меняющихся параметров лучше использовать обычную публикацию/подписку или специальный пакет [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure).

### Задание значений параметров через launch файлы
Launch файлы это что-то вроде центрального конфигурационного файла всей системы. Во первых в них перечисляются все запускаемые ноды, а во вторых они является самым типичным местом для задания значений параметров, для этого в них предусмотрено два тега [«param»](http://wiki.ros.org/roslaunch/XML/param) и [«rosparam»](http://wiki.ros.org/roslaunch/XML/rosparam). Первый используется для задания одного единственного параметра, а с помощью «rosparam» можно задавать несколько параметров одновременно, также, только с помощью этого тэга можно задавать параметры с комплексными типами list и dictionary.

Подробней про формат roslaunch файлов можно почитать [тут](http://wiki.ros.org/roslaunch/XML).

Пример launch файла с параметрами и ниже код как их читать.

`example.launch`
```xml
<launch>
  <param name="someinteger1" value="1" type="int" />
  <param name="someinteger2" value="2" />

  <param name="somestring1" value="bar2" />
  <!-- force to string instead of integer -->
  <param name="somestring2" value="10" type="str" />

  <param name="somefloat1" value="3.14159" type="double" />
  <param name="somefloat2" value="3.0" />

  <param name="/global_name" value="This is global parameter" />
  <param name="relative_name" value="This is relative parameter" />

  <!-- you can set parameters in child namespaces -->
  <param name="wg/wgchildparam" value="a child namespace parameter" />
  <group ns="wg2">
    <param name="wg2childparam1" value="a child namespace parameter" />
    <param name="wg2childparam2" value="a child namespace parameter" />
  </group>

  <!-- use rosparam for more complex types -->

  <!-- Lists YAML syntax -->
  <rosparam param="my_double_list">
      - 1.1
      - 2.2
      - 3.3
      - 4.4
  </rosparam>
  <!-- Lists (optional comma separated inline JSON syntax) -->
  <rosparam param="my_double_list1">[1.1, 2.2, 3.3, 4.4]</rosparam>

  <!-- Associative arrays -->
  <rosparam param="my_dict1">
      a: foo
      b: bar
      c: baz
  </rosparam>
  <!-- Associative arrays (optional comma separated inline JSON syntax) -->
  <rosparam param="my_dict2">{a: 100, b: 200, c: 300}</rosparam>

  <!-- Associative arrays (string keys) -->
  <rosparam param="my_dict3">
      '1': false
      '2': true
      '3': false
  </rosparam>

  <!-- mixed JSON/YAML syntax -->
  <rosparam param="limbs">
    leg1: ports: [0, 1, 2, 3, 4]
          reverse: [true,false,true,false,true]

    leg2: { ports: [5, 6, 7, 8, 9], reverse: [true,false,true,false,true] }

    leg3: ports: [10,11,12,13,14]
          reverse: [true,false,true,false,true]

    leg4: ports: [15,16,17,18,19]
          reverse: [true,false,true,false,true]

    neck: ports: [20,21,22]
          reverse: [true,false,true]

    tail: ports: [23]
          reverse: [false]
  </rosparam>

  <!-- use YAML files -->
  <group ns="yaml">
    <rosparam command="load" file="$(find test)/launch/example.yaml" />
  </group>

  <!-- upload the contents of a file as a param -->
  <param name="configfile" textfile="$(find test)/launch/example.launch" />
  <!-- upload the contents of a file as base64 binary as a param -->
  <param name="binaryfile" binfile="/bin/bash" />
  <!-- upload the output of a command as a param. -->
  <param name="cmd_output" command="uname -rsmo" />

  <node name="test_node" pkg="test" type="test_node" output="screen">
      <!-- set a private parameters for the node -->
      <param name="private_param1" value="a value" />
      <rosparam>
         private_param2: "a value"
         private_param3: "a value"
      </rosparam>
      <!-- you can set environment variables for a node -->
      <env name="ENV_EXAMPLE" value="some value" />
  </node>
</launch>
```

`example.yaml`
```yaml
string1: bar
string2: !!str 10
preformattedtext: |
  This is the first line
  This is the second line
  Line breaks are preserved
  Indentation is stripped
list1:
 - head
 - shoulders
 - knees
 - toes
list2: [1, 1, 2, 3, 5, 8]
dict1: { head: 1, shoulders: 2, knees: 3, toes: 4}
integer1: 1
integer2: 2
float1: 3.14159
float2: 1.2345e+3
robots:
  childparam: a child namespace parameter
  child:
     grandchildparam: a grandchild namespace param
```

В примерах ниже я кое-где для сравнения привожу закомментированный код C++11.
```cpp
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "param_reader");
  ros::NodeHandle nh;

  int someinteger1, someinteger2;
  nh.getParam("someinteger1", someinteger1);
  // read int and set default value if it's not set
  nh.param("someinteger2", someinteger2, 0);
 
  std::string somestring1, somestring2;
  // getParam() returns a bool, which provides the ability to check
  // if retrieving the parameter succeeded or not
  if (nh.getParam("somestring1", somestring1))
  {
    ROS_INFO("Got param somestring1: %s", somestring1.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'somestring1'");
  }
  // Sometimes the compiler requires a hint for the string type.
  nh.param<std::string>("somestring2", somestring2, "default_value");
 
  // Floats and Doubles
  double somefloat1, tmp;
  nh.getParam("somefloat1", somefloat1);
  nh.param("somefloat2", tmp, 0.5);
  float somefloat2 = tmp;

  // Namespaces
  std::string global_name, relative_name, wg_wgchildparam, wg2_wg2childparam1;
  nh.getParam("/run_id", global_name);
  nh.getParam("relative_name", relative_name);
  nh.getParam("wg/wgchildparam", wg_wgchildparam);
  nh.getParam("wg2/wg2childparam1", wg2_wg2childparam1);

  // Sum a list of doubles from the parameter server
  std::vector<double> my_double_list;
  double sum = 0;
  nh.getParam("my_double_list", my_double_list);
  for(unsigned i=0; i < my_double_list.size(); i++) {
    sum += my_double_list[i];
  }
  ROS_INFO("my_double_list sum=%.3f",sum);

  // Construct a map of strings
  std::map<std::string,std::string> my_dict1;
  // Read dict to map
  nh.getParam("my_dict1", my_dict1);
  // Add element to map and write it back to parameter server 
  my_dict1["d"] = "qux";
  nh.setParam("my_dict1", my_dict1);
  // Show it on screen
  typedef std::map<std::string,std::string>::iterator it_type1;
  for(it_type1 iterator = my_dict1.begin(); iterator != my_dict1.end(); iterator++) {
    // iterator->first = key
    // iterator->second = value
    ROS_INFO("my_dict1[%s] => %s", iterator->first.c_str(), iterator->second.c_str());
  }
  /* // C++11 syntax 
  for(auto const &it1 : my_dict1) {
    ROS_INFO("my_dict1[%s] => %s", it1.first.c_str(), it1.second.c_str());
  } // */

  std::map<std::string,int> my_dict2;
  nh.getParam("my_dict2", my_dict2);
  typedef std::map<std::string,int>::iterator it_type2;
  for(it_type2 iterator = my_dict2.begin(); iterator != my_dict2.end(); iterator++) {
    // iterator->first = key
    // iterator->second = value
    ROS_INFO("my_dict2[%s] => %d", iterator->first.c_str(), iterator->second);
  }
  /* // C++11 syntax
  for(auto const &it2 : my_dict2) {
    ROS_INFO("my_dict2[%s] => %d", it2.first.c_str(), it2.second);
  } // */

  std::map<std::string,bool> my_dict3;
  nh.getParam("my_dict3", my_dict3);
  typedef std::map<std::string,bool>::iterator it_type3;
  for(it_type3 iterator = my_dict3.begin(); iterator != my_dict3.end(); iterator++) {
    // iterator->first = key
    // iterator->second = value
    ROS_INFO("my_dict3[%s] => %s", iterator->first.c_str(), iterator->second ? "true" : "false");
  }
  /* // C++11 syntax
  for(auto const &it3 : my_dict3) {
    ROS_INFO("my_dict3[%s] => %s", it3.first.c_str(), it3.second ? "true" : "false");
  } // */

  XmlRpc::XmlRpcValue topicList;
  if (nh.getParam("limbs", topicList))
  {
   std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
   for (i = topicList.begin(); i != topicList.end(); i++)
   {
     std::string topic_name;
     std::stringstream port_list, reverse_list;

     topic_name = i->first;
     if( i->second["ports"].getType() == XmlRpc::XmlRpcValue::TypeArray ){
       std::vector<int> ports;
       nh.getParam("limbs/" + topic_name + "/ports", ports);
       std::copy(ports.begin(), ports.end(), std::ostream_iterator<int>(port_list, ", "));
     }

     if( i->second["reverse"].getType() == XmlRpc::XmlRpcValue::TypeArray ){
       std::vector<bool> reverse;
       nh.getParam("limbs/" + topic_name + "/reverse", reverse);
       for(int i=0; i<reverse.size(); ++i){
         reverse_list << ( reverse[i] ? "true, " : "false, " );
       }
     }

     ROS_INFO("limbs/%s/ports/[%s]",topic_name.c_str(), port_list.str().c_str() );
     ROS_INFO("limbs/%s/reverse/[%s]",topic_name.c_str(), reverse_list.str().c_str() );
   }
  }

  std::string configfile;
  nh.getParam("configfile", configfile);
  ROS_INFO("configfile.size = %lu", configfile.size());

  XmlRpc::XmlRpcValue binaryfile;
  nh.getParam("binaryfile", binaryfile);
  std::vector<char> binaryfile_v = static_cast<XmlRpc::XmlRpcValue::BinaryData&>( binaryfile );
  std::string binaryfile_s( binaryfile_v.begin(), binaryfile_v.end() );
  ROS_INFO("binaryfile.size = %lu", binaryfile_s.size() );

  std::string cmd_output;
  nh.getParam("cmd_output", cmd_output);
  ROS_INFO("cmd_output = %s", cmd_output.c_str());

  ros::spin();

  return 0;
}
```

Ссылки по теме
* [Parameter Server](http://wiki.ros.org/Parameter%20Server)
* [rosparam](http://wiki.ros.org/rosparam)
* [roscpp Parameter Server](http://wiki.ros.org/roscpp/Overview/Parameter%20Server)
* [Parameters Tutorials](http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters)
* [Parameterization](http://wiki.ros.org/ROS/Patterns/Parameterization)
* [Publisher and Subscriber with Parameters and Dynamic Reconfigure](http://wiki.ros.org/roscpp_tutorials/Tutorials/Publisher%20and%20Subscriber%20with%20Parameters%20and%20Dynamic%20Reconfigure)
