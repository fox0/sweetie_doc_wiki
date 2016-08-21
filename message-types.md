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

1. В катологе `src`:

        $ rosrun rtt_rosnode create_rtt_msgs robotname_msgs

    Будет создан пакет `rtt_robotname_msgs`

2. В файле `pacakge.xml` пакета, использующего созданный тип данных требуется указать зависимость:

        <depend package="rtt_robotname_msgs"/>

3. При написании кода C++ использовать заголовки:

        #include <robotname/typekit/MsgName.h>

4. При разертывании компонента убедитесь в наличие необходимых типов:

        # Поддержка элементарных типов ROS (к примеру, int64) появляется после этой команды.
        # Возможно, также потребуются импорт пакетов (rtt_*_msgs).
        import("rtt_ros");
        # Загружаем typekit
        import("rtt_robotname_msgs");
        # Вывести список зарегистрированных типов 
        .types


# Типы сообщений

Общее проектное решение: "толстые" сообщений. Это означает, что связанные данные передаются по возможности в составе 
одного большого пакета. При этом следует стремится к миминизации числа типов.

**Заголовок**: в заголовок можно внести разную полезную информацию.
1. seq --- номер цикла управления (только gait/animation, остальные наследуют).
2. timetamp --- время посылки сообщения (?)
3. frame --- имя компонента-источника сообщения (?).

## Состояние робота: кинематические параметры

### HerkulexPacket

**Семантика**: Либо корректный пакет протокола сервоприводов Herkulex, предназначенный для посылки или полученный от приводов. (Проверка контрольных сумм произведена), либо извещение о ошибке при получении ответа на запрос.

**Прагматика**: в текущей версии используется для взаимоджействия между компонентами, реализующими разные уровни протокола (`herkulex_*`). Удобная форма для передачи конфигурационных сообщений от не РВ части в РВ.
 
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
Сообщения здает цель для группы именованных приводов.

**Прагматика**: дает возможность управления приводами в дискретном позиционном режиме, является наиболее полной формой задающего воздействия для привода.
Сообщение унифицироано по форме с `sensor_msgs::JointState`.

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

**Замечание**: реализация playtime с одним элементом не необходима. 

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

*Порядок сортировки*: сначала идут приводы кинематических цепочк. Сами цепочки и последовательность их следования задается при помощи парaметра `chains` в плагине `RobotModel`.
Порядок приводов в цепочке естественный (от начала к концу).  После цепочек идут "свободные" приводы из URDF в произвольном порядке. 

### CartesianState

**Семантика**: векторы состояния группы кинематической цепочки в декартовой СК. Положение, ориентация, скорость конца цепочки относительно ее основания. 
Цепочка задается именем.

**Прагматика**: В текущем проекте все кинематические цепочки имеют тело робота в качестве основания. Имя цепочки можно отождествлять с именем последнего сегмента.
При генерации походок перешещение конечности удобно задавать относительно тела робота.

     # State vectors of the group of kinematics chains. It is assumed that all chains are linked to robot frame.
     #  * name --- the name of kinematic chain.
     #  * position --- the last segment pose in coordinate system of kinematic chain base (robot frame).
     #  * speed --- the last segment speed.
     #  * force --- external force applied to the last segment in robot frame coordinates (maybe world coordinates?)
     #
     # All arrays sould have the same size. Presece of limb, position and speed fields is necessary.
     Header header
     string[] name
     pose[] position
     twist[] speed
     # wrench[] force

**Замечание**: Обязательно заполнены поля name, position, speed.

**Замечание**: Присутствеи force одновременно недостаточно и избыточно. Для нормальных расчетов динамики требуется знать силы, действующие на начало и конец цепочки.
В текущем проекте нет задач, для которых это понадобилось бы силовое взаимоействие (шлифовка, мытье полов, переноска яиц?). Однако он может использоваться вместо 
структуры `SupportState`.

### SupportState

**Семантика**: распределние веса робота по конечностям в отнсительных единицах измерения. 

**Прагматика**: инкапсулирует показания датчиков касания, передает предполагаемое распределение веса от модуля походки к модулю расчета обратной динамики (по аналогии c NimbRO-OP).
Введение отдельного сообщения позволяет избежать необходимости включения этой информации в виде поля force в `CartesianState` и `JointLimbState`, позволяет использовать `JointState` на выходе 
генераторов походки.

     # Robot weight distributon between legs.
     #  * name --- the name of the chain.
     #  * support --- leg support coefficients. Support coefficient is the ratio of 
     # the weight corresponding to the leg to the total weight of robot.
     #
     # All arrays sould have the same size.
     #
     Header header
     string[4] name
     float64[4] support

**Замечание**: Принципиально возможно "занести" эти коэффициенты в CartesianState и JointLimbState, 
однако отдельное сообщение все равно требуется для передачи выхода датчиков касания.

**Замечание**: можно отказаться от `name`, но тогда будут сложности с расшитрениеи массива датчиков касания.


## Состояние робота: декоративные элементы

Сообщения управления хвостом, ушами и глазами. 

### Уши и хвост

Для управления хвостом и ушами (в совокупоности) можно использовать сообщения `JointState` или `JointLimbState`.

### Глаза

Для управления направлением взгляда теоретически можно использовать `JointState` (или `JointLimbState`) и `CartesianState`. 
Действительно, положение зрачка задается двумя углами, дибо точкой в пространстве, куда направлен взгляд.

Однако, существуют еще ряд параметров: положение века, его ориентация, диаметр зрачка. Также особые эффекты. 

    Формат сообщения подлежит определнию. 

**Альтернативы**:

1. Использование геометрических сообщений и дополнительных сообщений с описанием эмоции.
2. Большие сообщения для управления глазами.


## Распределение ресурсов

Эти сообщения используются для переключения походок и анимаций. Пока приводится только сообщение базового варианта архитектуры c
с вытесняющей моделью распределения.

### ResourceList (бывшее ResourceRequest)

**Семантика**: универсальное сообщение, используется для уведомления о запросах компонента на ресурсы,
уведомления о деактивации компонента (освобождении ресурсов), передачи списка ресурсов.

    # Set of resources and status of controllers.
    #  * name --- name of the controller component emmiting request.
    #  * status --- state of the controller component emmiting request.
    #  * resourses --- resources set (kinematic chains names: leg1, leg2, ..., tail, eyes).
    #  * pending_pri --- priorities in pending state
    #  * operational_pri --- priorities in operational state
    #
	#  Only necessary fields are filled. Possible sets are:
    #  * Deactivating notification from controller: name, status
    #  * Resource request from controller: name, status, request, priorities 
    #  * Resuorce list: resources
    #
    string name
    uint8 status
    string[] resources
    uint8[] pending_pri 
    uint8[] operational_pri 
    # statuses
    uint8 NONOPERATIONAL=0
    uint8 OPERATIONAL=1
    uint8 PENDING=2

**Замечание**. Заполняются только необходимые поля.

**Замечание**. Последние два поля нужны только для системы с приоритетами.

**Замечание**: Более машинное представление ресурсов --- ID и битовые векторы. Однако такой подход полразумевает необходимость 
ощих средств преобразования их в имена.

### ResourceRequest)

**Семантика**: Запрос ресурсов у арбитра. Уведомляет арбитр о желании получить некий набор ресурсов и приоритет. 
Задаются два приоритета: приоритет при ожидании активации и приоритет в активном сотоянии. Подробнее в [переключении походок](gait-switchong).`

    # Resource assigment request.
    #  * requester_name --- name of the controller component emmiting request.
    #  * resourses --- resources set (kinematic chains names: leg1, leg2, ..., tail, eyes).
    #  * pending_pri --- priorities in pending state (same size as resources).
    #  * operational_pri --- priorities in operational state (same size as resources).

	string requester_name    
    string[] resource       
    float[] pri_operational
    float[] pri_pending

### ResourceRequesterState

**Семантика**: Уведомление арбитр о согласии с выделенным набором ресурсов (компонент остается активен) и о деактивации компонета.

    # Acknowledge of ResourceList (is_operational = true) or notification about deactivation (is_operational = false).
    bool is_operational

### ResourceRequesterState

**Семантика**: Распределение ресурсов между задатчиками.

    # Resource assigment to controllers.
    #  * resource --- assigment resources list.
    #  * owner --- owner controller name.
    string[] resource
    string[] owner

## Тактирование и синхронизация

### TimerEvent

**Семантика**: Сообщение иницируют исполнение компонент с политикой `ON_TIMER`.
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

**Замечание**: испоьзование стандртных средств OROCOS не позволяет передать поля `seq` и `stamp`.

## Информация о аварийных ситуациях 

Предлажается использовать стандартные [DiagnosticStatus](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html). 

**Семантика** полей:
1. level --- уровень сообщения (инвормация, предупредление, ошибка)
2. name --- тип сообщения (ошибка кинематики, аппаратная ошибка и т.д. Конкретные списки --- в описании компонента.
3. message --- человекочитаемое описание проблемы
4. hardware_id --- имя компонента, сформировавшего ошибку.

**Прагматика**: использование стандартных средств ROS.


