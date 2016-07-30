Надо больше!

Содержание
1. [Общие соглашения](common)
2. Математическое обоспечение
    1. [Обратная задача кинематики](kinematics-analytical)
2. Общее описание архитектуры
    1. [Основные цели](goals)
    1. [Архитектура подсистемы управления движением](architecture)
    1. [Развертывание](deployment)
    1. [Переключение задатчиков](gait-switching)
3. Детали реализации: подсистема управления движением
    1. [Типы сообщений подсистемы управления движением](message-types)
    1. [Компонент агрегатор позы `agregator_gait`](components-agregator-gait)
    1. [Компоненты задатчиков](components-gait)
    1. [Компонент-арбитр ресурсов](components-resource-control)
    1. [Плагин с кинодинамической моделью робота](plugin-robotmodel)
    1. [Компоненты кинематики `kinematics`](components-kinematics)
    1. [Компонент рассчета динамики `dynamic_inv`](components-dynamics)
    1. [Компонент обращения модели приводов `servo_inv`](components-servo-inv)
    1. [Компонент-интерфейс приводов `herkulex_`](components-herkulex-alt) 
    1. [Вспомогательная библиотека: фильтры](library-filters)
3. Использование OROCOS
    1. [Сервисы и плагины](orocos-services)
