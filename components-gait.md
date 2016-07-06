Задатчики (походки, анимации)
=============================

Компонент формирует задающее воздействии для некоторго набора ресурсов.
Для части ресурсов --- зарегистрированных кинематических цепочек --- выдает желаемую позу в угловой или декартовой системе кординат.
Для остальных ресурсов форма задания определяется типом.

Задатчики имеют встроенную подсистему переключения (захвата ресурсов).

Способ взаимодействия с верхним уровнем не декларируется, но рекомендуется использовать `actionlib.

Надор интерфейсов сильно зависит от назанчения. Далее перечисленны большинство возможных способов взаимодействия,
реальный компонент реализует только часть,.

### Входные порты

Синхронизация

1. `sync` (`TimerEvent`, EventPort) --- синхронизация таймера.

Текущее состояние робота

1. `pose_joints` (`JointState`/`JointLimbState`) --- состояние робота в угловой СК (желаемая или реальная по датчикам).
1. `pose_cartesian` (`CartesianState`) --- состояние робота в декартовой СК (желаемая или реальная по датчикам).
1. `support` (`SupportState`) --- показания датчтиков касания.

Высший уровень (зависит от реализации)

1. Желаемая скорость тела.
1. Целевого позиции тела.
1. Желаемые параметры движений.

### Выходные порты

Поза робота (gait)

1. `ref_pose_joints` (`JointState`/`JointLimbState`) --- задающее воздействие в угловой системе координат.
1. `ref_pose_cartesian` (`CartesianState`) --- задающее воздействие в декартовой системе координат.
1. `ref_support` (`SupportState`) --- Ожидаемое распределение веса (исли не 

Поза роботоа (animation)

1. `ref_anim_joints` (`JointState`/`JointLimbState`) --- задающее воздействие в угловой системе координат: хвост, уши.
1. `ref_eyes`--- задающее воздействие для глаз.


### Параметры

Параметры походки и анимаций, настраиваемые один раз при запуске.

### Операции

Запрос/выделение ресурсов (реализованы в [классе-родителе](componentes-resource-control)):

1. Предоставляет: `releaseReosources`
1. Требует: `requestExclusiveResources`

Интерфейс `actionlib`:

1. Получить задание (цель двжения, его парметры).
1. Прекратить исполнение задания.
2. Средства извещения о завершенни, отказе исполнения.

### Семантика исполнения

После исполнения `configure` компонент готов к работе и запуску.

Запуск осуществляется вызовом `actionlib`, либо `start()` (эти механизмы исключают друг друга).
После вызова операции проводится проверка возможности запуска в данном состоянии робота. 
Формируется отказ (`Goal Rejected` или возврат `false`), либо создается запрос ресурсов.

Непосредственный переход в состояние `Running` происходит по подтверждению выделения ресурсов.

В состоянии `Running` исполняется основной код задатчика с периодом, определяемым таймером.
Значения времени и периода дискретизации переносятся из `TimerEvent` в выходные сообщения.

По завершению движения, либо по требованию освобждения ресурсов производится остановка выполнения.
Производится информирование высшего уровня средсвтами `actionlib` (цель достигнута/отвергнута),
лио иным способом, если это требуется.

**Замечание**: компонент сам управляет своим исполнение (вызывает `start()/stop()`). Такое соглашение 
позвояет реализовать другую семантику исполнения (например, компонент всегда запущен).

### Детали реализации.

Для компонентов на базе `actionlib` сдледует удалить операцию `start()` из внешнего интерфейса.

Компоненты задатчиков является наследником класса `ResourceControl::ResourceConsumer`, см. [компонент переключения походок](componentes-resource-control).
Запрос ресурсов осуществялется вызовом `bool requestExclusiveResources(const strings& resourses)`, 
при приходе требования о освобождении вызовается метод `notifyResourcesRelease()` (реализуется в наследнике).


Такой механизм в перспективе позволить усложнить протокол выделения ресурсов, не модифицируя сами компоненты задатчиков.
Также он избавляет програмиста от необходимости проверять запросы на осовобождение в каждой реализации задатчика.


### Исключения

Зависят от реализации.


Список задатчиков
-----------------

**Замечание**: Список имеет смысл раширить, дополнить и выделить в отдельную страницу

### Походки

* Движение шагом с заданной скоростью и радиусом кривизны.
* Свободная походка.
* Стать на месте.


### Анимации

* Трансляция сообщений `JointState` от `rviz`.
* Управление заданной ноги с джойстика в декартовой СК. Интерфейс джойстика --- отдельный компонент.
* Проиграть движение из файла.
* Типовые движени: лечь, сесть, встать, помахать копытом.


