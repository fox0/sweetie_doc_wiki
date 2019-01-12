Раздел описывает типы данных, используемые при взаимодействии компонентов. 
При описании сообщений используется стандартная нотация ROS. В подсистемах,
основанных на OROCOS используются те же типы данных.

## Процедура генерации typekit OROCOS 

Для полноценного использования составных типов данных в OROCOS требуется 
создания typekit. Typekit является аналогом пакета с описаниями сообщений ROS.

Ниже описана процедура генерации typekit по пакету с сообщениями ROS.

**Исходные условия**: установлен [`rtt_ros_integration`](http://wiki.ros.org/rtt_ros_integration), 
установлены пакеты группы `rtt_*_msgs`,
имя пользовательского пакета сообщений ROS `robotname_msgs`.

1. В каталоге `src`:

        $ rosrun rtt_rosnode create_rtt_msgs robotname_msgs

    Будет создан пакет `rtt_robotname_msgs`

2. В файле `pacakge.xml` пакета, использующего созданный тип данных требуется указать зависимость:

        <depend package="rtt_robotname_msgs"/>

3. При написании кода C++ использовать заголовки:

        #include <robotname/typekit/MsgName.h>

4. При развертывании компонента убедитесь в наличие необходимых типов:

        # Поддержка элементарных типов ROS (к примеру, int64) появляется после этой команды.
        # Возможно, также потребуются импорт пакетов (rtt_*_msgs).
        import("rtt_ros");
        # Загружаем typekit
        import("rtt_robotname_msgs");
        # Вывести список зарегистрированных типов 
        .types


# Типы сообщений

Общее проектное решение: "толстые" сообщений. Это означает, что связанные данные передаются по возможности в составе 
одного большого пакета. При этом следует стремится к минимизации числа типов.

**Заголовок**: в заголовок можно внести разную полезную информацию.
1. seq --- номер цикла управления (только gait/animation, остальные наследуют).
2. timetamp --- время посылки сообщения (?)
3. frame --- имя компонента-источника сообщения (?).

## Состояние робота: кинематические параметры

### HerkulexPacket

**Семантика**: Либо корректный пакет протокола сервоприводов Herkulex, предназначенный для посылки или полученный от приводов. (Проверка контрольных сумм произведена), либо извещение о ошибке при получении ответа на запрос.

**Прагматика**: в текущей версии используется для взаимодействия между компонентами, реализующими разные уровни протокола (`herkulex_*`). Удобная форма для передачи конфигурационных сообщений от не РВ части в РВ.
 
    # Herkulex servo control protocol packet. Message incapsulates proper outgoing (to servo) or incoming (from servo) packet. 
    #
    #  * servo_id --- servo hardware ID.
    #  * command --- protocol command (see Herkulex Servo manual) or error code.
    #  * data --- packet optional data without checksum.
    #
    uint8 servo_id
    uint8 command
    uint8[] data
    # command types constants
    uint8 CMD_EEP_WRITE=0x1
    uint8 CMD_EEP_READ=0x2
    uint8 CMD_RAM_WRITE=0x3
    uint8 CMD_RAM_READ=0x4
    uint8 CMD_I_JOG=0x5
    uint8 CMD_S_JOG=0x6
    uint8 CMD_STAT=0x7
    uint8 CMD_ROLLBACK=0x8
    uint8 CMD_REBOOT=0x9
    uint8 ACK_EEP_WRITE=0x41
    uint8 ACK_EEP_READ=0x42
    uint8 ACK_RAM_WRITE=0x43
    uint8 ACK_RAM_READ=0x44
    uint8 ACK_I_JOG=0x45
    uint8 ACK_S_JOG=0x46
    uint8 ACK_STAT=0x47
    uint8 ACK_ROLLBACK=0x48
    uint8 ACK_REBOOT=0x49
    uint8 ERR_TIMEOUT=0x81
    uint8 ERR_CHECKSUM1=0x82
    uint8 ERR_CHECKSUM2=0x83
    uint8 HEADER_SIZE=7
    uint8 DATA_SIZE=213

### ServoGoal


**Семантика**: Целевое значение для группы приводов c дискретным позиционным управлением. 

Сервоприводы Herkulex обладают дискретным позиционным управление с трапецевидным профилем скоростей. Целевая позиция 
задается позицией и длительностью движения. Привод сам рассчитывает значение скорости и позиции для промежуточных моментов времени.
Сообщения задаёт цель для группы именованных приводов.

**Прагматика**: дает возможность управления приводами в дискретном позиционном режиме, является наиболее полной формой задающего воздействия для привода.
Сообщение унифицировано по форме с `sensor_msgs::JointState`.

```
# Goal definition for position controlled servo with trapezoidal speed profile.
#
#  * name --- servo identification string.
#  * target_position --- target positions (rad).
#  * playtime --- movement duration (s).
#
# Arrays sould have the same size. Only exception is playtime, which can alse contains one value 
# if movement duration is equal for all servos. 
#
Header header
string[] name
float64[] target_position
float64[] playtime
```

**Замечание**: реализация playtime с одним элементом не необходима. 

### ServoModel


**Семантика**: 7-параметрическая модель привода с системой управления, учитывает П-регулятор, силу трения (вязкую, Кулона, Штрибека) [Compliant Robot Behavior using Servo Actuator
Models identified by Iterative Learning Control].

**Прагматика**: представляет модель привода в компактной человекочитаемой форме. Облегчает назначение параметров пиводов и передачу их между компонентами.

```
# Servo model parameters.
#
#  * name --- servo identification string.
#  * kp --- proportional coefficient of servo P-regulator.
#  * kgear --- ratio of reduction gear (number of turns of servo per one turn of joint)
#  * alpha[4] --- servo model and friction parameters (identifcation result).
#  * qs, delta --- Stribeck parameteres.
#
string name
float64 kp
float64 kgear
float64[4] alpha
float64 qs
float64 delta
```

### JointState

**Семантика**: вектор состояния всего робота или его части в угловой СК. Включает позицию, скорость и момент приводов, но при этом 
поля могут быть опциональны. Предназначена для задания позы в той части системы управления, где принадлежность привода какой-либо кинематической 
цепочки не принципиальна (обращение приводов, показания датчиков, полная поза робота). 

**Прагматика**: дает возможность непрерывного управления любой группой приводов, позволяет визуализировать позу в rviz без дополнительных преобразований.

Стандартное сообщение ROS [`sensor_msgs::JointState`](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)

**Замечание**: Обязательно заполнены поля name, position, speed.

### JointState (sorted)

**Семантика**: вектор состояния всего робота в угловой СК, переменные состояния отсортированы по цепочкам для быстрого преобразования в `JntArray`. 

**Прагматика**: позволяет передавать состояние всех цепочек без введения нового типа сообщений. Возможно преобразование в `JntArray` без копирования с использованием `Eigen::Map`.

*Порядок сортировки*: сначала идут приводы кинематических цепочек. Сами цепочки и последовательность их следования задается при помощи парaметра `chains` в плагине `RobotModel`.
Порядок приводов в цепочке естественный (от начала к концу).  После цепочек идут "свободные" приводы из URDF в произвольном порядке. 

### MultiDOFJointState

**Семантика**: векторы состояния группы кинематической цепочки в декартовой СК. Положение, ориентация, скорость конца цепочки относительно ее основания. 
Цепочка задается именем.

**Прагматика**: Стандартный тип ROS `sensor_msg::MultiDOFJointState`. В текущем проекте все кинематические цепочки имеют тело робота в качестве основания. Имя цепочки задано при помощи `RobotModel`.
При генерации походок перемещение конечности удобно задавать относительно тела робота.

     # State vectors of the group of kinematics chains. It is assumed that all chains are linked to robot frame.
     #  * joint_names --- the name of kinematic chain (multiDOF joint).
     #  * transforms --- the last segment pose in coordinate system of kinematic chain base (robot frame).
     #  * twist --- the last segment speed.
     #  * wrench --- force applied to the last segment in robot frame coordinates (maybe world coordinates?)
     #
     # All arrays sould have the same size. Presece of limb, position and speed fields is necessary.

     Header header
     string[] joint_names
     geometry_msgs/Transform[] transforms
     geometry_msgs/Twist[] twist
     geometry_msgs/Wrench[] wrench

**Замечание**: Обязательно заполнены поля `joint_names`, `transforms`, `twist`.

**Замечание**: Присутствие force одновременно недостаточно и избыточно. Для нормальных расчетов динамики требуется знать силы, действующие на начало и конец цепочки.
В текущем проекте нет задач, для которых это понадобилось бы силовое взаимодействие (шлифовка, мытье полов, переноска яиц?). Однако он может использоваться вместо 
структуры `SupportState`.

### RigidBodyState

**Семантика**: дублирует `MultiDOFJointState`. Cостояние группы кинематической цепочки в декартовой СК. Положение, ориентация, скорость конца цепочки относительно ее основания. 
Цепочка задается именем.

**Прагматика**: типы `geometry_msgs` неудобны для вычислений, требуются дополнительные преобразования для вызова функций KDL. Сообщение предполагается использовать внутри подсистемы
OROCOS для более удобной передачи данных.

     # State vectors of the group of kinematics chains. It is assumed that all chains are linked to robot frame.
     #  * joint_names --- the name of kinematic chain (multiDOF joint).
     #  * transforms --- the last segment pose in coordinate system of kinematic chain base (robot frame).
     #  * twist --- the last segment speed.
     #  * wrench --- force applied to the last segment in robot frame coordinates (maybe world coordinates?)
     #
     # All arrays sould have the same size. Presece of limb, position and speed fields is necessary.

     Header header
     string[] joint_names
     kdl/Frame[] transforms
     kdl/Twist[] twist
     kdl/Wrench[] wrench

**Замечание**: если типы KDL не поддерживаются ROS, то сообщение придется создавать штатными средствами OROCOS.

**Замечание**: если отказаться от его реализации, то тип заменяется `MultiDOFJointState`.

### SupportState

**Семантика**: распределение веса робота по конечностям в относительных или абсолютных единицах измерения. 

**Прагматика**: инкапсулирует показания датчиков касания, передает предполагаемое распределение веса от модуля походки к модулю расчета обратной динамики (по аналогии c NimbRO-OP).
Введение отдельного сообщения позволяет избежать необходимости включения этой информации в виде поля force в `CartesianState` и `JointLimbState`, позволяет использовать `JointState` на выходе 
генераторов походки.

     # Robot weight distributon between legs.
     #  * name --- the name of the chain.
     #  * support --- leg support coefficients or z-component of reaction force. Support coefficient is the ratio of 
     # the weight corresponding to the leg to the total weight of robot.
     #
     # All arrays sould have the same size.
     #
     Header header
     string[] name
     float64[] support

**Замечание**: Принципиально возможно "занести" эти коэффициенты в `RigidBodyState`
однако отдельное сообщение все равно требуется для передачи выхода датчиков касания.

**Замечание**: можно отказаться от `name`, но тогда будут сложности с расширением массива датчиков касания.

## Действия и траектоии 

#### Траектория в угловой системе координат

Используются сообщения из [`control_msgs`](http://docs.ros.org/api/control_msgs/html/index-msg.html)

**Семантика**: 
* `control_msg::FollowJointTrajectoryGoal` --- траектория в угловой СК с информацией о требованиях к ее исполнению.
* `control_msg::FollowJointTrajectory` --- действие `actionlib` по исполнению траектории в угловой СК.

**Прагматика**: эти типы сообщений и действий используются `MoveIt!.

Сообщение `FollowJointTrajectoryGoal`:

    trajectory_msgs/JointTrajectory trajectory  # непосредственно траектория, массив JointState, снабженный метками времени
    JointTolerance[] path_tolerance             # допустимая ошибка для каждого сочленения при исполнении
    JointTolerance[] goal_tolerance             # допустимая ошибка для каждого сочленения в конце траектории
    duration goal_time_tolerance                # допустимое отклонение по времени

Поле `path_tolerance` используются для проверки начальных условий исполнения траектории и последующего  контроля движения. 
`goal_tolerance` проверяется по окончанию движения. 

#### Траектория в декартовой системе координат: `FollowStepSequence`

**Семантика**: последовательность шагов. Несет онформацию о смещении основания, исполнительных органов, наличии касания и внешних силах, рассчитанных модулем генерации походки.

**Прагматика**: траектория формируется библиотекой TOWR в виде внутренних структур данных (сплайнов), тип данного действия копирует сообщения `xpp_msg::RobotStateCartesianTrajectory,
однако позволяет более эффективно хранить большие массивы точек и задавать ориентацию исполнительных органов

    # Robot movement presented in cartesian space.
    #
    # Request:
    #  * append --- append this trajectory to previous trajectory if possible
    #  * ee_names --- end effector names (the same order and size as in trajectory)
    #  * time_from_start --- timestamps of points in trajectories, seconds.
    #  * base_motion --- base_link trajectory
    #  * ee_motion --- end effector trajectories represented in base_link frame.
    #  * position_tolerance --- maximal allowed offset from desired position for end effectors and robot base.
    #  * orientation_tolerance --- maximal allowed axis-angle errors for robot base and end effectors.
    #
    # Result:
    #  * error_code 
    #      SUCCESSFUL is returned on success, INVALID_TRAJECTORY if trajectory is inconsistent (unsupported 
    #      time intervals, nonmonotonic time, incorect vector length and so on), INVALID_END_EFFECTOR indicates
    #      that end effector name is unknown, UNABLE_TO_APPEND says that trajectory cannot be appended to current 
    #      action and TOLERANCE_VIOLATED is raised if psition or orientation error is higher then bound,
	#      INTERNAL_ERROR indicate that trajectory cannot be execued due to internal error.
    #  * error_string
    #
    # Feedback:
    #  * time_from_start --- current execution time of compound trajectory.
    #  
    std_msgs/Header

    bool append 
    #string[] ee_names
    #xpp_msgs/RobotStateCartesian[] trajectory
	float64[] time_from_start
	sweetie_bot_kinematics_msgs/RigidBodyTrajectory base_motion
	sweetie_bot_kinematics_msgs/RigidBodyTrajectory[] ee_motion

    float64 position_tolerance
    float64 orientation_tolerance
    ---
    int32 error_code
    string error_string

    int32 SUCCESSFUL = 0
    int32 INVALID_TRAJECTORY = -1 #
    int32 INVALID_END_EFFECTOR = -2
    int32 UNABLE_TO_APPEND = -3
    int32 TOLERANCE_VIOLATED = -4
    int32 INTERNAL_ERROR = -5
    ---
    float64 time_from_start


Вспомогательные типы:

    #  sweetie_bot_kinematics_msgs/RigidBodyTrajectory
    string name  # end effector name
    sweetie_bot_kinematics_msgs/RigidBodyTrajectoryPoint[] points


    #  sweetie_bot_kinematics_msgs/RigidBodyTrajectoryPoint 

    geometry_msgs/Pose pose
    kdl_msgs/Twist twist
    kdl_msgs/Twist accel                    
    bool contact 


#### Перемещение платформы в декартовой системе координат: `MoveBase`

**Семантика**: задание планировщику на перемещение платформы. Позволяет решать задачи перемещения платформы и принятие заданной позы. 

* Для перемещения платформы требуется указать тип походки и желаемую позу основаия. Если указаны огрничения для ног, они будут применены также. 
    При этом условия на контакты и Z координата для ног игнорируются: в конце движения все ноги находятся в состоянии контакта.
* Для принятия позы используется режим STEPS. Надо указать конечное положения центра масс и ног. 
    Если все ноги изанчально опорные, каждая цель для ноги декларирует один шаг, т.е. всего будет сделано `ee_goal.size()` шагов в том порядке, в каком идут цели. 
	В случае, если одна илибольше ног в воздухе, добавляется один шаг, чтобы опустить их на землю.

**Прагматика**: одно сообщение для разных задач связанных с шаганием упрощает сервер. 

Вспомогательный тип для задания цели движения

    # sweetie_bot_kinematics_msgs/EndEffectorGoal
	# Goal state of end effector.
	# 
	# * name --- end effector name.
	# * frame_type --- frame of the end effector pose.
	# * contact --- should be end effector be in contact at the end of motion.
	# * pose --- desired pose. Note that towr ignores end effector orientation.
	#
	string name
	uint32 frame_type
	geometry_msgs/Pose pose
	bool contact
	int32 WORLD = 0	# unmovable coordinate system
	int32 BASE_INTIAL = 1
	int32 BASE_FINAL = 2
	int32 PATH_INITIAL = 3
	int32 PATH_FINAL = 4

Действие

    # sweetie_bot_kinematics_msgs/MoveBase
	# Sweetie Bot base movemnt request.
	# 
	# * gait_type --- one of posible gaits. Only free gait supports end effector end pose without contact.
	# * duration --- motion duration
	# * steps --- the number of step cycles, it is ignored in STEPS mode.
	# * base_goal --- goal pose of robot base
	# * ee_goal --- goal poses for end effector. For free gait their order defines step sequece.
	# * position_tolerance --- admissable position error.
	# * orientation_tolerance  --- admissable orientaton error.
	#
    int32 gait_type
    duration duration
	uint32 steps
    
    geometry_msgs/Pose base_goal
    sweetie_bot_kinematics_msgs/EndEffectorGoal ee_goal
    
    geometry_msgs/Vector3 position_tolerance
    geometry_msgs/Vector3 orientation_tolerance

	INT32 STEPS = 0 
	INT32 GAIT_STEP = 1
	INT32 GAIT_TROT = 2
    --
    int32 error_code
    string error_string
    
    int32 SUCCESSFUL = 0
    int32 NO_SOLUTION = -1 #
    int32 INVALID_END_EFFECTOR = -2
    int32 UNABLE_TO_APPEND = -3
    int32 TOLERANCE_VIOLATED = -4
    --
    duration time_from_start


#### Текстовое действие

**Семантика**: любое действие, задаваемое текстовым идентификатором. Такие действия могут использованы для выбора траектории среди сохраненных, выбора анимации изображений глаз.

**Замечание**: надо поискать в ROS аналоги.

**Замечание**: возможно расширение за счет введения параметров в виде пар строка и число или строка и строка.


**`TextCommand`** --- базовая текстовая команда.
 
    # Command identified by text string.
    #
    #  * type --- type of action. 
    #  * command --- text action identificator. A
    #  * args --- optional arguments names or list of arguments.
    #  * values --- optianal arguments values.
    #
    #  Usage of args is not recommended. Text indentifer sould have clear and definite semantic without any arguments.
    #  
    #  Example: { type : eyes/emotion, command: happy }
    #  Example: { type : joint_trajectory, command: greeting }
    #  
    string type
    string command
    strings args[]
    float64 values[]


**`TextCommandStamped`** --- текстовая команда, привязанная ко времени. 

    TextActionGoal command
    duration time_from_start


**`TextAction`** --- текстовое действие `actionlib`
 
    TextCommand command
    ---
    string status
    ---
    int32 error_code
    string error_string

**Замечание**: наилучший способ представления `Feedback` и `Result` не ясен из-за разнородности возможных команд. 

**TODO**: добавить совместимость с `FollowJointTrajectoryResult` по структуре полей.

#### Траектория в угловой СК с текстовыми действиями

**Семантика**: задает движение робота с сопутствующими действиями (смены анимаций, режимов работы и т.п.)

**`FollowJointTrajectoryWithActions`**  --- действие `actionlib` для траектории с информацией о дополнительных действиях. 

    FollowJointTrajectoryGoal trajectory
    TextCommandStamped[] text_commands
    ---
    FollowJointTrajectoryResult trajectory
    ---
    FollowJointTrajectoryFeedback trajectory


**`FollowJointTrajectoryWithActionsGoal`**  --- траектория в угловой СК и информацией о дополнительных действиях.


## Состояние робота: декоративные элементы

Сообщения управления хвостом, ушами и глазами. 

### Уши и хвост

Для управления хвостом и ушами (в совокупности) можно использовать сообщения `JointState`.

### Глаза

* Для управления направлением взгляда используется `gemetry_msg::Point` или `sensor_msg::JointState`, 
    В обоих случаях передаются 3 числа --- положение точки фокуса относительно головы. `JointState` 
    удобнее за счет возможности использования стандартной системы управления движением.

* Общее состояние глаза: `TextCommand` с типом `eyes/emotion`.

* Отдельная анимация: `TextCommand` с типом `eyes/animation`.


## Распределение ресурсов

Эти сообщения используются для переключения походок и анимаций. Пока приводится только сообщение базового варианта архитектуры c
с вытесняющей моделью распределения.

### ResourceRequest

**Семантика**: Запрос ресурсов у арбитра. Уведомляет арбитр о желании получить некий набор ресурсов и приоритеты каждого одельного 
ресурса (их важность для компонента). Задаются два приоритета: приоритет при ожидании активации и приоритет в активном стоянии. 
Подробнее в [переключении походок](gait-switchong).`

    # Resource assignment request.
    #  * requester_name --- name of the controller component emitting request.
	#  * request_id --- high 16 bits contains controller request counter number, low 16 bits contains an unique requester indentifer.
    #  * resources --- resources set (kinematic chains names: leg1, leg2, ..., tail, eyes).
    #  * pending_pri --- priorities in pending state (same size as resources).
    #  * operational_pri --- priorities in operational state (same size as resources).
    #
    # request_id is optional if operation is used to emmit request.
    #

    string requester_name 
	uint32 request_id
    string[] resource
    float[] pri_operational
    float[] pri_pending

**Замечание**: `request_id` необходим для контроля соответсвий `ResourceRequest` -- `ResourceAssignment` -- `ResourceRequesterState`.

### ResourceRequesterState

**Семантика**: Испльзуется для уведомления арбитра о согласии с выделенным набором ресурсов (компонент остается активен) 
и о деактивации компонента.

    # Acknowledge of ResourceAssigment (is_operational = true) or notification about deactivation (is_operational = false).
    #  * requester_name --- name of the controller component emitting request.
	#  * request_id / request_number --- the indentifer of last ResourceRequester emitted by controller, 
    #       or resource arbiter request counter value from ResourceAssigment message.
    #  * is_operational --- flag operational state.
    
    string requester_name 
    uint32 request_id / uint32 request_number
    bool is_operational

**Замечание**: сообщение может быть объединено с `ResourceRequest`. 
Тогда компонент может отказаться от части выделенных ему ресурсов. 

### ResourceAssignment

**Семантика**: Распределение ресурсов между задатчиками.

    # Resource assignment to controllers.
    #  * resource --- assignment resources list.
    #  * request_id/request_number --- the list of indentifers of last ResourceRequests of pending controllers
    #       or resource arbiter request counter value from ResourceAssigment message.
    #  * owner --- owner controller name.
    string[] resource
    uint32[] request_id / uint32 request_number
    string[] owner

### ControllersState

**Семантика**: публикуемый арбитром статус задатчиков.

**Прагматика**: может быть использован для мониторинга состояний компонент.

    # Controllers state message published by ResourceArbiter on any controller state change. All arrays have the same length.
    # * name --- list of controllers.
    # * state --- controler states.
    # * request_id --- ID of last message exchange with controller.
    # request_id increases with each resource request. It must be equal to or greater then request_id from ResourceRequests 
    # if given request was processed by arbiter and confirmation form controller is received.
	string[] name
    uint32[] request_id
    uint8[] state
    # state constants
    uint8 NONOPERATIONAL=0
    uint8 PENDING=1
    uint8 OPERATIONAL=2
    uint8 OPERATIONAL_PENDING=3

**Замечание**: если сторонний компонент занает `request_id` последнего `ResourceRequests` задатчика, то сравнивая его с `request_id` из этого сообщения,
можно понять, был ли обработан запрос и получено подтверждение состояния задатчика. Если `request_id` из сообщения больше или равно `request_id` из запроса,
то запрос был обработан.

### SetOperational (ROS service)

**Семантика**: команда активации/деактивации задатчика.

**Прагматика**: без знания `request_id` высокий уровен не сможет из сообщения `ControllersState` понять, обработан ли конкретный запрос ресурсов.

    # Activation/deactivation command for controller.
    #  * is_operational --- target controller state.
    #  * request_id --- resource request id or zero in case of failure. 
    bool is_operational
    --
    uint32 request_id

### SetOperational (ROS action)

**Семантика**: действие активации/деактивации задатчика. Позволяет высшему уровню выбрать контроллируемый список ресурсов, при необходимости задатчик может изменить его и известить о неудаче в процессе исполнения.

**Прагматика**: высший уровень должен знать, активен ли активированный задатчик. Этого можно достигнуть только используя интерфейс действий. Поле `resources` позволяет управлять произвольным набором элементов робота (если задатчику все равно, чем управлять). Если же задатчик может управлять только фиксированным набором --- он извещает об этом высший уровень.

    ### SetOperational (ROS service)
    # Controller activation/deactivation action.
    #
    # Request: activate/deactivate controller
    # * operational --- new controller state. Allows to deactivate component.
    # * resources --- list of resorces owned by controllers. Some controllers may ignore this field, 
    #      or have different interpretation of it.
    # 
    # Feedback: controller is active.
    # * resources --- actual resources set controlled by component.
    # 
    # Result: controller is not active  (deactivation can be triggered by iternal logic or external request)
    # * error_code --- deactivation reason
    # * error_string --- human redable deactivation reason
    #
    bool operational
    string[] resources
    ---
    int32 error_code
    string error_string
    
    int32 SUCCESSFUL = 0 # goal is achived or deactivation was triggered by external factor (e.g. action is ABORTED or PREEMTED).
    int32 COMPONENT_IS_NOT_READY = -1 # controller is not configured properly.
    int32 INVALID_RESOURCE_SET = -2 # Unable to start with prescribed resource set (for example, cannot use head as support leg, or cann't balance on one leg).
    int32 START_CONDITIONS_VIOLATED = -3 # In current robot state controller cann't start. 
    int32 EXECUTION_ERROR = -4 # Internal error of started controller.
    ---
    string[] resources

## Тактирование и синхронизация

### TimerEvent

**Семантика**: Сообщение инициируют исполнение компонент с политикой `ON_TIMER`.
Поля содержат номер события (из может быть в рамках цикла управления несколько), номер цикла управления, время. 

**Прагматика**: полезность поля `seq` (цикл управления), совместимость с ROS.

     # Timer event message.
     #  * event_id --- event type (timer ID),
     #  * seq --- control cycle sequece number,
     #  * stamp --- event time.
     #
     uint8 event_id
     uint32 seq
     time stamp

**Замечание**: использование стандартных средств OROCOS не позволяет передать поля `seq` и `stamp`.

## Информация о аварийных ситуациях 

Предлагается использовать стандартные [DiagnosticStatus](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html). 

**Семантика** полей:
1. level --- уровень сообщения (информация, предупреждение, ошибка)
2. name --- тип сообщения (ошибка кинематики, аппаратная ошибка и т.д. Конкретные списки --- в описании компонента.
3. message --- человекочитаемое описание проблемы
4. hardware_id --- имя компонента, сформировавшего ошибку.

**Прагматика**: использование стандартных средств ROS.


