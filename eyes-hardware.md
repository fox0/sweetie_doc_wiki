Реализация системы отображенния глаз
====================================

Робот снабжен двумя экранами, используемыми для отображения глаз. 

**Базовые требования**:

1. Способность отображать широкий спектр эмоций (изображение рисуется на уровне ПО).
2. Достаточное быстродействие для отображения базового набора эффектов.
3. Базовый набор эффектов включает полигональную 2D графику (все элементы глаза --- многоугольники, залитые цветом), градиенты (?), смешивание цветов (?).
2. Простота реализации на аппаратном и программном уровне.
5. Минимальная загрузка ЦПУ.


### Текущая реализация

Робот снабжен парой экранов [2.8' ЖК экранов](http://www.hotmcu.com/28-touch-screen-tft-lcd-with-all-interface-p-63.html) на базе технологии TN c контроллером ILI9325.
Экраны имеют разрешение 320x240, подключены через шину SPI, отображаются драйвером [tftfb](https://github.com/notro/fbtft) в пару кадровых буферов (framebuffer).
Частота обновления равна приблизительно 10-20 кадров в секунду и ограничена пропускной способностью SPI.

Текущая версия управляющего ПО представляет пару компонент ROS, которые при помощи библиотеки Qt выводят изображения в кадровые буферы. 
Из поддерживаемых эффектов имеется только перемещение зрачков и моргание. Последнее использовать невозможно из-за отсутствия синхронизации.

Возможностей имеющейся системы явно недостаточно, поэтому
разрабатывается новая версия [компонента управления глаз](eyes-control.md), поддерживающая множество анимаций, разные способы отображения и прочее. 

**Принятые архитектурные решения**:

0. Для отображения глаз используются ЖК экраны.
1. Управляющий компонент монолитен, формирует изображение на два глаза.
2. Управляющий компонент использует библиотеку Qt5.

На настоящий момент эти решения не обсуждаются, их можно рассматривать как спецтребования.

### Обсуждение

Существенную проблему представляет нераспространенность готовых решений по подключению двух полноценных экранов к одному встраиваемому устройству.
Основной видеоинтерфейс один (HDMI, DPI) один и рассчитан на использование с одним экраном. 

В связи с переходом с Rashberry PI 3 (ядро BCM2837) на BeagleBoard Black/Green Wireless (ядро [AM3358](http://www.ti.com/product/am3358)) уменьшается тактовая частота ЦПУ 
с 1.2 ГГц до 1. ГГц, четыре ядра заменяются на одно. Соответственно открыт вопрос достаточности производительности для обсчета движения и отрисовки изображения глаз.
Кране заманчивым выглядит использование аппаратного графического ускорителя, однако он напрямую привязан к базовому видеоинтерфейсу, также могут
возникнуть сложности использования его из Qt.

Информация о том, как Qt5 обеспечивает графическое ускорение 2-х мерной графики достаточно скудна. 

1. Qt обладает встроенным программным отрисовщиком (`Raster`), в большинстве случаев используется именно он: http://doc.qt.io/qt-5/topics-graphics.html

2. В определенном контексте Qt задействует аппаратной ускорение. При этом набор операций определяется возможностями оборудования/драйвера. Всего есть пять уровней:
    отсутствие аппаратного ускорения, ускорение операций с альфа-каналом, 2х мерная векторная графика, 3D, программируемый конвеер 3D. http://blog.qt.io/blog/2009/03/13/using-hardware-acceleration-for-graphics/
    Недоступные операции заменяются программными, что особенно медленно, если требуется графический контроллер и ЦПУ используют разную память (Non-UMA). 
    Но, вероятно, на нашем оборудовании память общая.

3.  Оба встраиваемых компьютера поддерживают OpenGL ES 2.0, т.е. теоретически они должны предоставлять полный спектр возможностей аппаратного ускорения. 
    Предположительно ускоритель и процессор используют общую память (UMA).

4. Не ясно, как понять, происходит ли в Qt аппаратное ускорение. Очевидно, оно имеет место, при отрисовке на контексты типа `QGLWidget`, в случае 
    `QPixmap` и `QImage` оно имеет только в определенных ситуациях (http://blog.qt.io/blog/2009/12/16/qt-graphics-and-performance-an-overview/ ) при 
    этом не ясно, как влияет выбранный формат данных (для всех ли форматов происходит ускорение, какой формат лучше выбирать). Вероятно, предпочтителен `QPixmap` 
    оптимизированный под конкретное представление на данном устройстве. В любом случае, ускорение будет иметь место только при использовании `QPainter`, любые формы 
    попиксельных операций ускоряться не будут. 

5. Передача изображения через основной графический интерфейс будет выполняться, скорее всего, аппаратно на уровне графического ускорителя. 
    Передача через SPI будет происходить попиксельно, либо через DMA, зависит от реализации драйвера.

Соответственно, наиболее перспективным выглядит использование графического ускорителя как для отрисовки изображений. В этом случае загрузка ЦПУ будет минимальной за счет максимального 
использования возможностей ускорителя.

## Варианты реализации 

### Два экрана на SPI шинах

Текущая реализация, опирающаяся на [tftfb](https://github.com/notro/fbtft) в качестве драйвера.

**Плюсы**:

* Наиболее отработанное решение. 
* Простота программной модели: два кадровых буфера, с которыми либо связывается контекст `QImage` или `QWidget`. В первом случае не требуется развертывание приложения Qt,
    т.к. `QImage` может быть ассоциировано напрямую с областью памяти кадрового буфера. (Пример). Во втором кадровые буферы играют роль экранов.
* На BeagleBoard можно использовать два SPI интерфейса. Однако это уменьшает число доступных UART.

**Минусы**:

* При разрешениях больше 320x240 частота обновления кадров становится практически неприемлемой. Это связано с ограниченной пропускной способность SPI.
    * Частота 16 МГц, разрешение 480x320, 16 бит: не более 6 кадров/c (новые экраны). 
    * Частота 30 МГц, разрешение 320x240, 16 бит, два экрана на одной шине: не более 12 кадров/c (старые экраны).

* Не ясно, осуществляется ли графическое ускорения. Вероятно, при отображении в буфер экрана средствами Qt оно имеет место. 
    В случае отрисовки в `QImage` 16-битного формата ситуация менее ясная.


### Использование HDMI

Существуют готовое решения, способные разделить HDMI сигнал на два экрана: [aliexpress](https://ru.aliexpress.com/item/3-81-inch-1080x1200-AMOLED-display-screenn-3D-VR-head-mounted-display-with-HDMI-to-MIPI/32798181688.html?spm=2114.03010208.3.16.cILH2d&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10065_10151_10208_10068_5310018_10301_10136_10137_10060_10155_10062_437_10154_10056_10055_10054_10059_303_100031_10099_10103_10102_10096_10169_10052_10053_10142_10107_10050_10051_5320018_10084_10083_10080_10082_10081_10110_519_10111_10112_10113_10114_10182_10078_10079_10073_10123_10189_10127_142_10125,searchweb201603_9,ppcSwitch_5&btsid=e4d64333-c083-42bd-9b5d-d1844c2235fc&algo_expid=0458969f-b1c6-4f37-839c-ea2b437f054c-2&algo_pvid=0458969f-b1c6-4f37-839c-ea2b437f054c).
Устройство используется для очков виртуальной реальности (https://www.youtube.com/watch?v=u4TvYW1vDYw ), программно они воспринимаются как единый экран HDMI, его левая половина отображается на 
левом, правая --- на правом. 

**Плюсы**:

* Готовое устройство. 
* OLED экраны.
* Удобная программная модель: рисуем один кадр, получаем два глаза.

**Минусы**:

* Цена порядка 250-300 $.
* Решение одного производителя.
* Экраны обладают очень большим разрешением 1200x1080 каждый. Однако устройство явно поддерживает другие видео режимы, как минимум 720 строк.

### Использование DPI (параллельный интерфейс)

Наиболее простой и хорошо совместимый интерфейс. Является основным для BeagleBoard, на Rashberry PI 3 отсутствует (не разведен ?). 
Представляет собой параллельную шину данных (8 или 16 бит в нашем случае), сигналы строковой и кадровой синхронизации. 
Скорость значительно выше SPI, однако частоты все равно составляют десятки МГц.

Теоретически при помощи аппаратного преобразователя можно разбить вывод контроллера на два экрана.

1. Покадровая разбивка: первая половина строк идет на первый экран, вторая на второй. Разделитель сигнала считает строки с начала кадра. 
    Первую половину подает на один экран, затем формирует ложный сигнал вертикальной синхронизации и переключать вывод на другой экран и подает вторую половину строк.

    **Проблемы**: 
    * соблюдение таймингов контроллера экранов (необходимы надлежащие паузы при подача строчной и кадровой синхронизации, см. документация ILI9325 и http://www.nxp.com/wcm_documents/techzones/microcontrollers-techzone/Presentations/graphics.lcd.technologies.pdf ) 
    * Не совсем ясна сложность конфигурации программной части. Согласно https://www.linusakesson.net/hardware/beagleboard/vga.php достаточно просто сконфигурировать видео режим. 
        С другой стороны в может потребоваться указывать длительность пауз, как это описано http://www.nxp.com/wcm_documents/techzones/microcontrollers-techzone/Presentations/graphics.lcd.technologies.pdf
    * поведение экрана в отсутствие сигнала (отсутствие тактового сигнала в частности).  Так первый экран, получает кадр, а затем отключается от шины на время передачи кадра второму экрану. 
        При отсутствии тактирования изображение просто не обновляется (так ведет себя ILI9325 в VSYNC режиме, стр. 40), либо может быть дополнительный сигнал ENABLE (см. тот же ILI9325).
        Однако все может зависеть от модели. 
    * Подачи инициализационной последовательности на экран. Однако ее необходимость не ясна. DPI достаточно примитивен, по идее, сконфигурированный в этом режиме
        экран должен работать в режиме внешнего тактирования без всякой инициализации. Можно отключать разделение экранов на время ее подачи через видео интерфейс (дополнительный GPIO сигнал),
        либо ввести отдельный МК, подающий инициализирующую последовательность, а потом переключающий экраны на видео интерфейс. 

    **Плюсы**:    
    * Простая программная модель: один экран сверху другого.
    * Полный набор цветов.
    * Задействован основной видео интерфейс, не должно быть проблем с ускорением.

    **Минусы**:
    * Сложности может составить настройка нестандартного видео режима (320x480, например).


2. Построчная разбивка: первая половина строки идет на один экран, вторая --- на другой. Принципиального отличия от покадровой разбивки нет, только больше требования к оборудованию разделителя: 
    надо считать не строки, а пиксели.

3. Побайтовая разбивка. Экраны работают в 8-битном режиме (RGB233), видео контроллер в 16-битном (RGB565). Возможны два варианта разбивки сигнала:
    * побайтово: старшая часть 16-битного слова передаваемого видео контроллером идет на один экран, младшая --- на другой. Отображается 256 цветов.
    * поканально: старшая часть цветового канала идет на один экран, младшая на другой. Отображается 128 цветов.
    Это технически самый простой вариант разделителя сигнала, надо просто развести разные линии по разным экранам.

    **Проблемы**: 
    * побайтовое разделение требует проведение попиксельных операций для объединения 2-х изображений, наиболее прямая реализация требует использования ЦПУ.
    * в поканальном разделении возможно использование альфа-смешения для совмещения двух изображения. Используемые цвета ограничиваются теми, у которых в RGB565 младшие биты нулевые, 
        при альфаблендинге фоновое изображение (один глаз)  остается неизменным, а к нему прибавляется накладываемое изображение (второй глаз), помноженное на альфа-коэффициент, соответствующий
        битовому сдвигу на 2 бита (Source Over Blending mode: https://doc.qt.io/archives/qq/qq17-compositionmodes.html). 
        Такие операции должен быть способен осуществлять ускоритель. При такой реализации число цветов будет 64.
    * Подачи инициализационной последовательности на экран (если нужна). Для ее подачи потребуется собственный модуль ядра, как минимум надо будет прописать в готовом инициализационные последовательности в виде
        пар байтов для обоих экранов. Второй вариант --- дискретный МК, который подает инициализационные последовательности на экраны, а потом переключает их на видео интерфейс. 

    **Плюсы**:    
    * Простота электронной части (в случае отсутствия МК).

    **Минусы**:
    * Программная реализация требует попиксельных операций.
    * Мало цветов.

## Выводы и предложения

Очень привлекательно выглядит DPI с покадровым разделением.

1. Надо четко выяснить алгоритм работы контроллера экранов (того же ILI9325). Насколько строго должны соблюдаться тайминги, можно ли их увеличить? Что происходит, 
     если на время отключить тактирование или импульсы синхронизации?  При этом нужно запустит экран и при необходимости надо провести эксперименты.

2. Надо установить, нужна ли инициализационная последовательность и требования к ее таймингам (насколько сложно сделать нужный МК).

3. Надо провести эксперименты по задействованию аппаратного ускорения при отрисовке в `QImage` и `QPixmap`. Работает ли оно на нашем оборудовании? 
    Для каких форматов какие операции ускорятся (альфа-смешение, отрисовка полигонов, градиенты)? Наличие ускорения можно проверить по косвенным признакам 
    (наличие нужных возможностей у графического ускорителя, проверка типа `QPaintDevice`). Но наиболее надежный вариант --- тестовые программы для разных операция и замеры 
    длительности исполнения и загрузки ЦПУ в режиме принудительного программного рендеринга и без него 
    (настраивается через командную строку в опциях Qt http://blog.qt.io/blog/2009/12/16/qt-graphics-and-performance-an-overview/ ).

4. Возможен ли подходящий режим альфа-смешения для поканального разделения изображения? Насколько сложны и длительны попиксельные операции в побайтном разделении по сравнению с поканальным?


## Ссылки по теме:
http://processors.wiki.ti.com/index.php/GPU_Compositing
http://blog.qt.io/blog/2009/11/20/building-qt-to-make-use-of-the-beagle-boards-sgx-gpu/