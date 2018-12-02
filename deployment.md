Развертывание системы
=====================

(Новая версия. Старая доступна [тут](deployment-old)).

Конфигурация развернутой системы описывается тремя элементами:

1. Загруженные компоненты.
1. Параметрами всех компонент.
2. Отношениями между компонентами (соединения портов и операций). 

Любых вариантов скрытых параметров следует избегать.

Общие идеи работы системы 
-------------------------

На уровне ROS система представлена набором `roslaunch` скриптов. На уровне OROCOS --- `lua` скрипты развертывания. 
Каждый из скриптов 
* представляет собой законченный модуль, добавляющий в систему законченную функциональность с определённым интерфейсом (порты, топики, сервисы),
* может зависеть от других модулей,
* инвариантен по отношению пространств имен, где запускается (необходимо для поддержки виртуального и реального робота).

Подсистема конфигурации делится на две части:
* не зависящие от робота элементы (пакет `sweetie_bot_deploy`)
* параметры и скрипты относящиеся к конкретному роботу (пакеты `<robot_name>_deploy`).

Способ хранения статических и медленном меняющие данных (параметры, траекторий) должен гарантировать их целостность во всей системе при использовании нескольких машин. 
Для этих целей избран сервер параметров ROS, на котором размещается вся конфигурация системы.
* Параметры компонент ROS и OROCOS хранятся `yaml` файлах, доступ к ним осуществляется через Parameter Server ROS. 
* Для типов OROCOS, несовместимых с ROS используются  `.cpf` файлы (разновидность xml). Они загружаются на сервер параметров в виде строк. 
* Сообщения ROS (траектории движения, в позы), хранятся в '.json', загружаются на сервер параметров в виде бинарных параметров в сериализованном виде. 

Предполагается наличие в развернутой системе двух машин:
* Компьютер робота (Rashberry Pi или Beagle Board) исполняет подсистему управления движением, код анимации глаз и звука. 
* Управляющий компьютер (компьютер оператора, ПК) запускает виртуального робота (часть подсистемы движения), графические системы мониторинга и управления движением (rviz, интерфейс flexbe).
     На эту же машину могут быть вынесен высший уровен управления, сложные операции по обаботке изображений, SLAM и т.п. 

Пространства имен
-----------------

Правила именования изложены в [общих соглашениях](common.md). Ниже приводится схема пространств имен ROS с загруженной подсистемой OROCOS

    /
    |- param: robot_description --- URDF model
    |- param: robot_description_semantic --- MoveIt configuration
    |
    |- CONF_FILE --- OROCOS configuration files parameters (.cpf, .log4cpp). 
    |- JOINT_TRAJECTORY --- parameters with joint trajectories.
    |
    |- MOTION --- OROCOS rt control node and configuration. Note that ROS see all OROCOS component as a one node with 'motion' name.
    |    |             This node is lauched on the robot on-board computer.
    |    |
    |    |- ROBOT_MODEL --- OROCOS robot semantic model (kinematics chains, contacts). It is loaded from `kinematic_chains.cpf`.
    |    |
    |    |- CONTROLLER --- motion controllers
    |    |    |- <controller_name> --- controller parameters and its topics.
    |    |    ...
    |    |- HERKULEX --- servo control groups
    |    |    |- LEG12 
    |    |    |    |- array --- provides configuration and debug interface for servos.
    |    |    |    |- driver
    |    |    |    \- sched 
    |    |    ...
    |    |- aggregator_ref --- provides robot reference pose in joint space 
    |    |- kinematics_fwd --- provides robot limbs positons in cartesian space
    |    |- odometry_ref --- base_link odometry
    |    |- dynamics_inv --- robot balace and servo efforts calculation
    |    |- aggregator_real --- provides robot real pose in joint space 
    |    ...
    |
    |- topic: joint_states, tf
    |- topic: control --- high level TextCommands for eyes emotions change and voice control.
    |- topic: move_group, trajectory_execution, pick, place --- MoveIt! topics.
    |
    |- robot_state_publisher --- /tf publication
    |- move_group --- MoveIt! control node, coresponding topics and parameters are loaded in / namespace.
    |- behavior --- FlexBe behavior engine
    |- eye_left, eye_right --- hardware dependent eyes node. They are lauched on the robot on-board computer.
    |
    |- VOICE 
    |   |- voice_node
    |   \- sound_play
    |
    |- FLEXBE --- flexbe operator interface nodes.
    |
    \- HMI --- visualization and opertor interfce
        |- rviz 
        \- pose_markers

В целях унификации модули должны быть работоспособны независимо от пространства имен, т.е. использовать относительные имена.
Однако на практике наблюдается ряд проблем с move_group, rviz не позволяющие запускать их вне корневого пространства имен.

#### Параметры ROS, топики ROS

Для каждого компонента следует загружать конфигурацию из сервера параметров, используя вызов `config.get_peer_rosparam()`.
Он обращается к Parameter Server, используя пространство имен вида (см. [общие соглашения](common.md)):

    <namespace>/<nodename>/<component_name>
    <namespace>/<nodename>/<subsystem>/<component_name>

Топики ROS, связанные с компонентами OROCOS, располагаются в том же пространстве имен. Их имена совпадают с именами портов.

#### Идентификаторы систем координат

ROS требует уникальности идентификаторов систем координат в рамках одного roscore. Топик `\tf` полагается глобальным, `rviz` не поддерживает использование нескольких `\tf`.

1. Неподвижной считается система координат `odom_combined`.
2. В `/tf` публикуются расчетная и желаемая поза робота:
    - `odom_combined -> base_link` --- публикуется одометрией. 
	- `base_link` -> `link*` --- желаемая поза робота (команды, что передаются на приводы, рассчитывается по выходу `aggregator_ref`).
	- `base_link` -> `real/link*` ---  поза робота по датчикам (результат опроса приводов, рассчитывается по выходу `aggregator_real`).

#### Интеграция OROCOS и ROS 

[`rtt_ros_integration` package](https://github.com/orocos/rtt_ros_integration) позволяет отображать порты OROCOS на топики ROS, также он предоставляет компонентам OROCOS доступ к 
серверу параметров, позволяет реализовывать на OROCOS серверы actionlib. ROS воспринимает подсистему OROCOS как одну ноду с именем `motion`. Компоненты OROCOS представляют 
свои интерфейсы в пространствах имен в соответствие со схемой выше. Например, `controller/stance` предоставляет топики сервера actionlib в пространстве имен `motion/controller/stance` 
слушает топик `motion/controller/in_base_ref`, его параметры извлекаются из пространства имен `motion/controller/stance`.

Некоторые параметры имеют особое значение:
 * `period` (`double`) всегда устанавливается в длительность цикла управления, которая определена в конфигурации компонента `timer`.
 * `services` (`strings`) содержит имена сервисов OROCOS, которые требуется загрузить в компонент.
 * `priority` (`int`) устанавливает приоритет RT Linux.

Соответственно для работы с топиками, нодами и параметрами можно использовать стандартные средства ROS. Взаимодействовать с компонентами OROCOS можно через 
консоль [`rttlua`](http://www.orocos.org/wiki/orocos/toolchain/luacookbook), которая становится достпной с запуском подсистемы OROCOS.
Интерфейсы компонентов OROCOS самодокументированы, например, посмотреть интерфейс компонента `controller/stance` можно набрав `= controller.stance`. 

Система развертывания
-------------------

Система развертывания представлена набором пакетов в каталоге `config`:
* `sweetie_bot_deploy` --- базовая подсистема развертывания. 
* `sweetie_bot_movit_config`  --- конфигурация `moveit!`.
* `sweetie_bot_test` --- средства тестирования подсистем, требующих развертки значительной части СУ робота.
* `sweetie_bot_proto2_deploy` --- специфические элементы системы развертывания для SweetieBot Proto2.

### Базовая подсистема развертки

`sweetie_bot_deploy` предоставляет средства для запуска системы. Это поимущественное независимые от робота `.launch` файлы, скрипты `.lua`

#### Скрипты `launch`

Скрипты `launch` размещаются в каталога, в зависимости от принадлежности к той или иной подсистемы.

Принципиально они могут быть двух типов:

1. **Модули**, загружающие некую хорошо определенную часть системы. Обладают свойством универсальности, что позволяет их задействовать в любой части системы.
    Их можно разделить на две группы: 
    * Модули робота, запускаемые соответственно на бортовом компьютере робота или на машине оператора (для моделирования работы). 
    * Модули машины оператора, которые запускаются на машине оператора в любой ситуации.

2. **Скрипты высокого уровня.** Они загружают нужный набор параметров и модулей, включая запуск подсистемы OROCOS на роботе или компьютере оператора.
     
     Особенностью этих скриптов является следующие параметры:

     * `run_real` (boolean, default: false) означает, что будут запущены компоненты исполнительной системы робота (управление приводами, экраны-глаза).
     * `robot` (boolean, default: true) указывает, что будут запущены компоненты, которые должны исполнятся на бортовом компьютере робота.
     * `host` (boolean, default: true) указывает, что будут запущены компоненты, кторые должны исполнятся на машине оператора.
	 * `robot_ns` (string, default: /) пространство имен, где запускаются подсистемы робота. 
	 * `robot_name` (string, default: sweetie_bot_proto2) префиксы пакетов с конфигурацией робота, к нему добавляется постфиксы `_deploy` (парамеры и скрипты), `_moveit_config` (конфигурация MoveIT!), `_description` (URDF модель). Потенциально могут быть и другие пакеты, например `_movements` (записанные движения).
	 * `robot_profile` (string) указывают, откуда брать специфичные для робота параметры и скрипты конфигурации в пакете `<robot_name>_deploy`. 

	 Параметры `robot_name` и `robot_profile` позволяет иметь несколько пакетов роботов с различными вариантами конфигурации в них.
	 Это достигается следующим образом:

     * Каталог `<robot_name>_deploy/<robot_profile>` подключается как первый overlay при вызове `config.lua` (см. подробности позже позже).  Т.е. оттуда по умолчанию берутся скрипты `.lua` и файлы с параметрами.
	 * Этот каталог содержит файлы `load_param.launch` (загрузка параметров), `robot_module.launch` (специфичные для данной конфигурации робота модули ROS).
  

Модули реализующие функциональность робота должны быть инвариантны по отношению к базовому пространству имен. Любой `*.launch` файл начинается с поясняющего комментария, описывающего его тип (MODULE --- модуль, DEPLOYMENT --- скрипт развертки) и назначение.

Наиболее важные `.launch` файлы.

* `load_param.launch` --- загрузить параметры. Он просто вызывает `load_param.launch` из соответствующего пакета описания робота и профиля.
* `joint_space_control.launch` --- robot deployment script. It starts basic motion control configuration. 
* `flexbe_control.launch` --- robot deployment script. It starts basic motion control configuration and high level control nodes (eyes and voice).
* `flexbe.launch` --- FlexBe core and user interface module. Starts GUI to control robot high-level behavior.
* `joint_trajectory_editor.launch` --- start GUI tool to create new movements.

`joint_space_control.launch` и `flexbe_control.launch` запускают консоль `rttlua`

### Подсистема развертывания OROCOS

Центральным его элементом является скрипт `common/config.lua`, предназначенный для запуска нужной конфигурации OROCOS.
     
         rttlua config.lua <module1>.lua <module2>.lua> ... <overlay1> <overlay2> ... __ns:=<namespace> __name:=<name>

Скрипт выполняет следующие действия:

1. Разрешает параметры `<overlayX>` в имена каталогов. Если путь относительный, то к нему добавляется префикс пакета `sweetie_bot_deploy`, иначе используется переданный путь.
    К полученному добавляются `<sweetie_bot_deploy>/common` и `<sweetie_bot_deploy>`

2. Добавляет полученный список каталогов к `LUA_PATH`, `<overlay1>` идет будет первым, затем `<overlay2>` и т.д.

3. Загружает необходимое окружение RTT: модули `rosnode`, `rosparam`, объявляет глобальные переменные `depl` (Deployer), `ros` (сервис ROS).

4. Имя ноды `<name>` и `<namespace>` используется для назначения параметра `default_root_category` сервиса `log4cpp`.

5. Предоставляет функции 
    * `config.file`, которая ищет файл сначала на сервере параметров ROS в пространстве имен `conf_file/`, а затем последовательно в `<overlay1>`, `<overlay2>` и т.д.
	    Такой механизм позволяет легко получить доступ к любому `.cpf` файлу конфигурации.
	* `config.set_property`, `config.get_property` упрощают назначение параметров компонент OROCOS.
	* `config.get_rosparam` позволяет получить параметр от сервера параметров.
	* `config.get_peer_rosparams` устанавливает параметры компонента OROCOS используя соответствующее пространство имен сервера параметров.
	    Некоторые параметры обрабатываются специальным образом (`priority`, `period`, `services`).

6. Последовательно загружает `<moduleX>` командой `require`.

Ряд полезных переменных попадет в таблицу `config`: список модулей, оверлеев, категория логгера и т.п.

Для удобства использование этого скрипта его вызов инкапсулирован в скрипт bash `sweetie-bot-core`:

    sweetie-bot-core <module1>.lua <module2>.lua> ... <overlay1> <overlay2> ... __ns:=<namespace> __name:=<name>

#### Использование скрипта развертки OROCOS

Развертка осуществляется при помощи модулей lua и конфигурационных файлов:

    sweetie_bot_deploy
     |- sweetie-bot-core --- скрипт bash для запуска config.lua
     |- common --- скрипты и конфигурация общего назначения
     |   |- config.lua --- главный скрипт развертки
     |   |- config_extra.lua --- библиотека вспомогательных функций (обращение к rosparam, упрощение назначения сложных параметров и т.п.)
     |   |- reporting.lua --- функции загрузки и подключения `OCL::ReportingComponent`
     |   |- logger.lua
     |   \- logger.log4cpp
     |- motion_core
     |   |- resource_arbiter.lua, resource_control.yaml
     |   |- motion_core.lua --- арбитр и агрегатор (минимальный набор).
     |   |- motion.lua --- арбитр и агрегатор и подсистема управления приводами.
     |   |- virtual_motion.lua --- ядро для симуляции.
     |   \- herkulex_feedback.lua, herkulex_feedback.yaml, sweetie_bot_servos.cpf --- подсистема Herkulex.
     |- joint_space_control --- задатчики, работающие в угловой СК.
     |   |- controller_joint_state.lua --- контроллер `FollowJointState`.
     |   |- controller_joint_trajectory.lua --- контроллер `AnimationJointTrajectory`.
     |   |- controller_joint_space_all.lua --- все контроллеры.
     ... 

Модули подчиняются общим соглашениям:

1. Устанавливают между собой зависимости посредством `require`. Это позволяет загружать один модуль не более одного раза, независимо от того, сколько раз вызывалось `require`.

2. Загружают соответвующие компоненты, устанавливают параметры используя процедуру из `config_extra`, соединяют порты. 
    Благодаря вызову `require` можно считать, модули, от которых зависит данный уже загружены.

3. Создают переменную lua, по которой легко обращаться к загруженному компонету (`agregator_ref`, `servo_inv` и т.п.). 
    Имя переменной совпадает с именем компонента без префикса (см. [общие соглашения](common.md)).

    Отдельне модули (`resource_control`, задатчики) в соответвие со схемой имнования помещают переменные компонента в таблицу, играющую роль пространства имен:
        
        herkulex.all.array
        herkulex.head.sched
        resource_control.arbiter
        
    В этой же таблице могут присутсвовать вспомогательные переменные и функции модуля.

3. Для обращения к конфигурационным файлам (`.log4cpp` и `.cpf`) используют `config.file()`. Это позволяет замещать фалы при помощи механизма overlay.

4. Для обращения к не входящим в оверлеи модулям можно использовать нотацию `require "<overlay>.<module>"`. См. документацию lua к команде `require`.

5. По завершению выполнения модуля соответствующая подсистема должна быть в работоспособном состоянии.

Типовой вызов выглядит так:

    sweetie-bot-core controller_joint_state.lua motion.lua joint_space_control motion_core

Он загружает один контроллер и подсистему управления реальным роботом. Для успешного запуска все необходимые параметры должны быть загружены из yaml в сервер параметров.


### Конфигурация робота (пакеты `<robot_name>_deploy`)

Типичный каталог конфигурации робота содержит следующие файлы:

* `load_param.launch` 
    1. загружает параметры из `.yaml` файлов на сервер параметров.
	2. устанавливает `robot_description` и `robot_description_dynamics` (упрощенная динамическая модель для динамики).
	3. загружает конфигурационные файлы ORCOS (`.cpf`, `.log4cpp`) в виде строк а сервер параметров в пространство имен `/conf_file/`. При этом недопустимые символы в имени файла заменяются на `_` в соответствие с соглашениям ROS.
	4. загружает сохраненные траектории.
* `robot_module.launch` --- содержит команды запуска аппаратно-зависимой части робота.
* `logger.log4cpp` --- конфигурация системы журналирования.
* YAML файлы по подсистемам:
    * `controller.yaml` --- задатчики
    * `herkulex_feedback.yaml` --- приводы. Здесь используется деление приводов на группы, для каждой из которых запускается свои экземпляры системы `herkulex_`.
    * `motion.yaml` --- основные компоненты подсистемы управления движением.
    * `hmi.yaml` --- подсистема визуализации.
* `.cpf` файлы с динамически загружаемыми параметрами OROCOS или с несовместимыми с ROS типами данных.
    * `kinematic_chains.cpf` --- kinematic chains description.
    * `herkulex_servos_<group>.cpf` --- servos group definitions and hardware configuration and ID mapping.
    * `kinematics_inv_joint_limits.cpf` --- joint limits for `kinematics_inv_trac_ik` component.

### Хранение траекторий движения и поз

Эти объекты представляют собой сообщения ROS. На сервере параметром они хранятся в виде бинарных параметров с сериализованым сообщением. В файловой системе они сохраняются в виде `.json` 
файлов при помощи `rospy_message_converter`. 

Для облегчения этих процедур `sweetie_bot_deploy` содержит скрипт `store` (консольная утилита для сохранения/загрузки параметров сообщений в/из `.json`).
`load_param.launch` использует его для загрузки параметров на сервер.

Примеры:

    rosrun sweetie_bot_deploy store load /joint_trajectory/hoof_stamp hoof_stamp.json
    rosrun sweetie_bot_deploy store save /joint_trajectory/ movements

### Бортовые компьютер и компьютер оператора

* Все параметры и конфигурационные файлы должны загружаться на сервер параметров ROS. Непосредственно команды по загрузке находятся в `load_param.launch`
    Если конфигурационный файл не загружен на сервер, то подсистема OROCOS будет использовать локальную для нее версию.
* Скрипты `.launch` поддерживают тег `machine`, поэтому в режиме `run_real` компоненты на бортовом компьютере робота будут запускаться через `ssh` от имени `sweetie@<short_robot_name>`.
    Короткое имя робота --- это значение параметра `robot_name` без префикса `sweetie_bot_`, если он присуивует.
* Скрипты `.lua` всегда локальны. Т.е. они должны совпадать на машине оператора и роботе.

### Типичная процедура запуска

Запустить модель робота, rviz и `joint_trajectory_editor`:

    roslaunch sweetie_bot_deploy joint_space_control.launch
	roslaunch sweetie_bot_deploy joint_trajectory_editor.launch

Сохранить траекторию в json файлы в пакете `sweetie_bot_proto2_movements` (неявно будет использован скрипт `store`).

    rosrun sweetie_bot_proto2_deploy store-joint-trajectories save

Запустить базовую  систему управления роботом:

	host $ roslaunch sweetie_bot_deploy load_param.launch
	sweetie $ roslaunch sweetie_bot_deploy joint_space_control.launch run_real:=true host:=false robot:=true 
	host $ roslaunch sweetie_bot_deploy joint_space_control.launch run_real:=true host:=true robot:=false

Запустить базовую систему, MoveIt! и FlexB:

	host $ roslaunch sweetie_bot_deploy load_param.launch
	sweetie $ roslaunch sweetie_bot_deploy flexbe_control.launch run_real:=true host:=false robot:=true 
	host $ roslaunch sweetie_bot_deploy flexbe_control.launch run_real:=true run_moveit:=true host:=true robot:=false
	host $ roslaunch sweetie_bot_deploy flexbe.launch


Использование нескольких `roscore`
---------------------------------

Наиболее надежную систему можно получив, развернум несколько `roscore`: одно на роботе, другое на управляющем компьютере (http://wiki.ros.org/sig/Multimaster/Existing Techniques).

Единственный стабильный механизм (http://wiki.ros.org/multimaster_fkie) позволяет видеть топики и сервисы другого roscore, что достаточно, чтобы создавать единое рабочее пространство.
Некоторую сложность представляет синхронизация параметров. Эту возможность предоставляет `config_manager` из [`mongodb_store`](http://wiki.ros.org/mongodb_store).



