# ros-orocos.pdf
 Цель данной страницы --- описание основных возможностей ROS и пакетов группы OROCOS  в контексте задачи управления движением. 

Обе системы *компонентно-ориентированы*, т.е. работающее приложение --- совокупность компонент, взаимодействующих через известные интерфейсы. 

**Основная задача проектирования --- выделение компонент и их интерфейсов** .

1. Компоненты должны быть универсальны (использование их в разных вариантах системы, с разными настройками).

2. Относительно слабо связаны между собой. Хороший критерий --- компонент можно испорльзовать без любого другого.

3. Следует избегать повторения функций, при необходимости использовать соответвующие  приемы: библиотеки, плагины.

4. Интерфейс компонента должен быть задокументирован. Полностью!

## Жизненый цикл компонента

Компонент (нода в ROS) загружается, настраивается, запускается, работает, останавливается, выгружается.
В OROCOS эти стадии четко разделены, в ROS явного разделения нет, но можно сделаь искуственно через операции.

В OROCOS компонент --- это всегда одна нить (поток исполнения). Там выделяется три режима работы: периодический (по таймеру),  по событию на порту (обработка сообщения), по прерыванию. OROCOS разработан так, что програмист не должен думать о синхронизации потоков. 

В ROS ограничения на нити нет, так и нет основных режимов работы. Все в руках программиста.

## Средства взаимодействия компонент

В таблицы привдены основные элементы интерфейса компонент. Все элементы строго типизированы.

| ROS                | OROCOS                | Примечания  |
|----------------|--------------------|--------------------|
| топик   |  порты | Передача типизированых сообщений издатель/подписчик, основной интерфейс РВ. |
| сервис | операция |  Аналог методов классов. Взаимодействие  запрос-ответ.  Используются для осуществления сложных операций, настройки. |
| параметр | параметр |  Аналог полей классов. Основной средство настройки компонент. |
| плагин | плагин  | Аналог динамической библиотеки. Некий програмный объект загружаемый в компонент и придающий ему некую дополнительную функциональность. |

Наиболее близкий аналог сигналов ТАУ --- сообщения передаваемые по портам/топикам. Примеры сигналов: поза, скорость и т.п.
Следует отметить, что *порты могут быть использованы для синхронизации*, т.к. один компонент извщает другой о событии.

## Именование 

### Статическое
ROS, OROCOS:  `пакет/типа компонента`
    
В ROS возможны вложения пакетов. 

### Динамическое  (времени выполнения)
OROCOS: `имя компонента/ресурс`  
Ресурс --- имя порта, операции, парметра...

В ROS к этой схеме добавляется:

1.  Пространства имен (внутри и снаружи).

2. Механизм относительныъх имен.

3. Name remapping: стандартное имя заменяется на псевдоним.

4. Средсва агрегирования *roslauch* (группа компонентов объединябтся в большой виртуальный компонент).

Компонент определнного типа может быть заружен под заданным 

## Развертывание приложения

Работующее приложение собирается из набора компонент. 
Их связывани и настройка позволяет получить различную функциональность и режимы работы. 

В  OROCSO эту задачу решает специальный компонент `deployer `, работающий в интерактивном режиме и исполняющий скрипты OPS и/или XML файлы. Возможно применения rtlua и языка lua. Язsк OPS алгоритмически полон. Позволяет писать компоненты и автоматы.

В ROS есть средства командной строки (`rosrun`), загрузки конфигурации из (`roslauch`: YAML *.lauch файлы), централизированный сервис настроек (`Parameter server` и `dynamic_reconfigure`). 

<table>
<tr>
<th>Операция </th><th> ROS </th><th> OROCOS </th>
</tr>
<tr>
<td>  интерактивная настройка </td><td> коммандная строка </td><td> `deployer` </td>
</tr>
<tr>
<td>  загрузка компонент </td>
<td>  `roslauch`, *.lauch файлы </td> 
<td>  
<ul><li>
                 `deployer`, *.ops, *.xml  
</li><li> `rtlua`
</li></ul>
</td>
</tr> 
<tr>
<td>  связывание портов </td>
<td>  
<ul><li>
                ноды сами знают имена топиков
</li><li> name remapping
</li><li> `roslauch` + remappnig
</li></ul>
</td>
<td>  
Явно указываются связи портов.
<ul><li>
                `deployer`, *.ops, *.xml  
</li><li>`rtlua`
</li></ul>
</td>
</tr> 
<tr>
<td>  подключение плагинов </td>
<td>  
<ul><li>
                командная строка
</li><li>`roslauch`
</li></ul>
</td>
<td>  
<ul><li>
                `deployer`, `rtlua`
</li><li>`rtt_roscomm`: порты превращатся в топики.
</li></ul>
</td>
</tr> 
<tr>
<td>  назначение параметров </td>
<td>  
<ul><li>
               `parameter_server`
      <ul><li>
                      yaml файлы
      </li><li> компоненты сами запрашивают параметры
      </li><li> remapping --- изменение имени запрашиваемого параметра
      </li></ul>
<li/><li> `dynamic_reconfigure`
    <ul><li>
                    .cfg файлы с декларациями парметров в каждом компоненте
    </li><li> запрос их у `parameter_server`
    </li><li> изменение во время выполнени при помощи операций
    </li><li> gui, единое древо динамических парметров
    </li><li> remapping не раболтает (?)
   </li><li> видимо, не перваривает одинаковые декларации парметров в разных .cfg
    </li></ul>
</li><li>`nimbro_config_server`
    <ul><li>
                   призван решить проблему `dynamic_reconfigure` с одинаковыми параметрами в разных
      компонентах. 
    </li><li> синтаксис без .cfg, имя параметра в коде.
     </li></ul>
</li></ul>
</td>
<td>  
<ul><li>
                `deployer`,  `rtlua`, xml 
</li><li> `rtt_ros_param`, `rtt_dynamic_reconfigure` --- интеграция с ROS, праметры OROCOS обретают все свойства ROS.
</li></ul>
</td>
</tr> 
</table>
 