Надо больше!

Содержание
1. [Общие соглашения](common)
2. Математическое обеспечение
    1. [Обратная задача кинематики](kinematics-analytical)
3. Общее описание архитектуры
    1. [Основные цели](goals)
    1. [Архитектура подсистемы управления движением](architecture)
    1. [Развертывание](deployment)
    1. [Переключение задатчиков](gait-switching)
4. Детали реализации: подсистема управления движением
    1. [Типы сообщений подсистемы управления движением](message-types)
    1. [Компонент агрегатор позы `agregator_gait`](components-agregator-gait)
    1. [Компоненты задатчиков](components-gait)
    1. [Компонент-арбитр ресурсов](components-resource-control)
    1. [Плагин с кинодинамической моделью робота](plugin-robotmodel)
    1. [Компоненты кинематики `kinematics`](components-kinematics)
    1. [Компонент расчета динамики `dynamics_inv`](components-dynamics)
    1. [Компонент обращения модели приводов `servo_inv`](components-servo-inv)
    1. [Компонент-интерфейс приводов `herkulex_`](components-herkulex-alt) 
    1. [Вспомогательная библиотека: фильтры](library-filters)
5. Использование ROS
    1. [Введение](ros-introduction)
    1. [Установка](ros-installation)
    1. [Создание пакетов использующих «публикацию/подписку»](ros-create-pub-sub)
    1. [Создание и вызов сервисов](ros-create-service)
6. Использование OROCOS
    1. [Введение](orocos-introduction)
    1. Примеры
       1. [Передача данных через порт](orocos-example1)
       2. [Операции](orocos-example2)
       3. Добавляем интеграцию с ROS
          * [Пример1](orocos-ros-example1)
          * [Пример2](orocos-ros-example2)
       4. Параметры
       1. [Сервисы и плагины](orocos-services)
