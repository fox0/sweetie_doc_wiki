Содержание
1. Описание используемых инструментов при разработке ПО
    1. Git
        1. [Введение](git-introduction)
        1. [Обзор основных понятий](git-terms)
        1. [Установка git на linux](git-install-linux)
        1. [Установка git на windows](git-install-windows)
        1. [Создание и работа с тестовым репозиторием](git-new-repo)
        1. [Описание репозиториев проекта](git-project-repos)
        1. Поток разработки
            1. [Описание используемого нами потока разработки](git-workflow)
            1. [Памятка: Типичные операции с репозиторием](git-typical-operations)
    5. ROS
        1. [Введение](ros-introduction)
        1. [Установка](ros-installation)
        1. [Создание пакетов использующих «публикацию/подписку»](ros-create-pub-sub)
        1. [Создание и вызов сервисов](ros-create-service)
        1. [Параметр сервер (Parameter Server)](ros-parameters)
    6. OROCOS
        1. [Введение](orocos-introduction)
        1. Примеры
           1. [Пример1: Передача данных через порт](orocos-example1)
           2. [Пример2: Операции](orocos-example2)
           3. [Добавляем интеграцию с ROS примеру 1](orocos-ros-example1)
           4. [Добавляем интеграцию с ROS примеру 2](orocos-ros-example2)
           5. [Сервисы и плагины](orocos-services)
2. Описание аппаратной части
    1. [Глаза](eyes-hardware)
1. Описание архитектуры ПО
    1. [Общие соглашения](common)
    2. Математическое обеспечение
        1. [Обратная задача кинематики](kinematics-analytical)
    3. Общее описание архитектуры
        1. [Основные цели](goals)
        1. [Архитектура подсистемы управления движением](architecture)
        1. [Развертывание](deployment)
        1. [Переключение задатчиков](gait-switching)
		1. [Управление глазами](eyes-control)
    4. Детали реализации: подсистема управления движением
        1. [Типы сообщений подсистемы управления движением](message-types)
        1. [Компонент агрегатор позы `agregator_gait`](components-agregator-gait)
        1. [Компоненты задатчиков](components-gait)
        1. [Список задатчиков](components-gaits-and-animations)
        1. [Компонент-арбитр ресурсов](components-resource-control)
        1. [Подсистема исполнения сохраненных движений](components-animation-stored-move)
        1. [Плагин с кинодинамической моделью робота](plugin-robotmodel)
        1. [Компоненты кинематики `kinematics`](components-kinematics)
        1. [Компонент расчета динамики `dynamics_inv`](components-dynamics)
        1. [Компонент обращения модели приводов `servo_inv`](components-servo-inv)
        1. [Компонент-интерфейс приводов `herkulex_`](components-herkulex-alt) 
        1. [Компонент таймера](components-timer) 
        1. [Вспомогательная библиотека: фильтры](library-filters)
