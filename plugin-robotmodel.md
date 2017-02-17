Плагин кинодинамической модели `RobotModelURDF`
=======================


Предназначен для унификации загрузки модели робота, доступа к кинематическим цепочкам и средствам именования и нумерации звеньев. 
Эту функциональность используют: 

* модули кинематики, 
* динамика, 
* `agregator_gait` (версия на базе `JointLimbState`), 
* задатчики (собственная кинематика или преобразование `JointState` -> `JointLimbState`) 

1. `urdf` (`string`) --- описание робота 
1. `kinematics_chains` (`PropertyBag`) --- структура с декларациями ресурсов-кинематических цепочек.
```
      |- chain1_name
      |  |- first_link (string)
      |  |- last_link (string)
      |  \- parameter1 (игнорируется)
      \- chain2_name
```
### Операции (предоставляет)

1. `configure()/cleanup()` (`OwnThread`) --- загрузка urdf, формирование цепочек (`chains`), очистка.

2. `strings listChains()` (`ClientThread`) --- список цепочек.

3. `strings listJoints(const string& name)`  (`ClientThread`) --- список звеньев.

3. `string getJointChain(const string& name)`  (`ClientThread`) --- возвращает имя цепочки-владельца звена, пустую строку, если звена нет.

3. `strings getJointsChains(strings name)`  (`ClientThread`) --- возвращает список имен цепочек владельцев звеньев.

4.  `strings listAllJoints()` (`ClientThread`) --- список всех звеньев.

5.  `int getJointIndex(const string& name)` (`ClientThread`) --- возвращает позицию звена в упорядоченной в соответствие с принятыми соглашения позе робота, -1 в случае ошибки.

6. `bool extractChain(const string& name, const sensor_msgs::JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)` (`ClientThread`) --- выделение заданной кинематической цепочки.

7. `bool packChain(const string& name, JntArray& position, JntArray& speed, JntArray& efforts, JointState& in)` (`ClientThread`) 
     --- обратное действие, возврат true в случае успеха (неуспех: нет цепочки, компонент не готов).

Из интерфейса исключен `JointLimbState` по следующим причинам: 
* указанными методами `JointLimbState` может быть легко получен итерациям по именам цепочек, 
* компоненты кинематики нуждаются именно в `JntArray`, а не `JointLimbState`,
* вариант с `JointLimbState` не способен исключить динамическое выделение памяти.

**Замечание**: можно реализовать на операциях интерфейс добавления цепочек.
1. `addChain(string name)`, `removeChain(string name)` добавление/удаление динамических параметров для ручного использования (вспомогательный класс).  Создает необходимый набор дочерних параметров цепочки (`first_link`,...). Изменяет только опции, для перенастройки требуется вызов `configure`.


### Методы (внутренний интерфейс сервиса)

2. `KDL::Chain& getCahin(const string& name)`  --- доступ к цепочкам, не может быть реализован как операция.

1. ` bool mapChain(const string& name, JointState& joint_state, JntArray& position, JntArray& velocity, JntArray& effort)` --- для 
отсортированного по цепочкам сообщения `joint_state` отображает в `JntArray` участки `vector<double>` из `JointState`. При ошибке возвращает false.

2. `PropertyBag& getChainProperties(const string& name)` --- упрощение доступа к дополнительными параметрам цепочки.

### Детали реализации

Для ускорения работы предварительно (при конфигурации) выделяются зарегистрированные кинематические цепочки. 
Потребуется ввести атомарную переменную флаг, чтобы исключить запуск методов до завершения конфигурации.

Вероятно, для удобства параметры плагина можно добавить непосредственно в базовый `Service` компонента

