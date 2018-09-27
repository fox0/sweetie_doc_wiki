Задатчики движения
==================

Список задатчиков движения. Подробнее смотрите [описание типового задатчика](components-gait).
Задатчики реализующие интерфейс действия `SetOperational` являются наследниками `SimpleControllerBase`.

1. **Повторение сохраненного движения** [`ExecuteJointTrajectory`](components-animation-stored-move) (бывший `AnimationJointTrajectory`).
    Исполнения и создание анимаций путем задания траекторий в угловой СК. Поддержка MoveIt!
    Траектория может быть снабжена информацией о контактах.

    Интерфейс; действие `control_msgs::FollowJointTrajectory`

2. **Управление в угловой СК** [`FollowJointState`](components-follow-joint-state). 
    Получает сообщение от [joint_state_publisher](http://wiki.ros.org/joint_state_publisher) и передает его двигателям.

    Интерфейс; действие `sweetie_bot_resource_control_msgs::SetOperational`, сообщения `JointState`.

3. **Отключение всех приводов** [`TorqueMainSwitch`](components-torque-off). 
	Отключает все приводы всех `HerkulexArray` при активации. 

    Интерфейс; действие `sweetie_bot_resource_control_msgs::SetOperational`.

4. **Управление свободной кинематической цепочкой в декартовой СК** [`FollowPose`](components-follow-pose). 
    Управление движением заданной ноги в декартовой системе координат, так что она принимает заданную позу в неподвижной СК.
    Использует обратную кинематику.

    Интерфейс; действие `sweetie_bot_resource_control_msgs::SetOperational`, сообщения `geomtery_msga::Pose`.

5. **Управление позой тела в декартовой СК** [`FollowStance`](components-follow-stance). 
    Управление движением платформы в декартовой системе координат, так что она принимает заданную позу в неподвижной СК.
    Список опрных ног не менятся.

    Интерфейс; действие `sweetie_bot_resource_control_msgs::SetOperational`, сообщения `geomtery_msga::Pose`.

6. **Шаговый контроллер** [`ExecuteStepSequence`](components-step-sequence). 
    Испольнить заданную траекторию движения в виде последовательности шагов. Последовательность синтезируется генератором походки (TOWR).

    Интерфейс; действие `sweetie_bot_kinematics_msgs::FollowStepSequence`.


