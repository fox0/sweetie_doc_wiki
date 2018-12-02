Конфигурация компьютеров
=======================

Настройка прав назначения приоритетов реального времени
-------------------------------------------------------

Настраивается в файле `/etc/security/limits.conf`, необходим для запуска компонентов РВ OROCOS.

```
<username>			-		rtprio			99
```
или 
```
@<groupname>			-		rtprio			99
```

Конфигурация сети
------------------

Стандартная схема управления роботом предполагает наличие в сети 2-х компьютеров: машины оператора и бортового компьютера робота.
Сеть должна предоставлять следующие сервисы:

* **`dhcp`** При загрузке адреса получаются автоматически, компьютер регистрируется в сети.
* **`dns`** Должно обеспечивайся разрешение имен компьютеров, необходимое для работы ROS.
* **`ntp`** Обеспечивает синхронизацию часов в сети, также необходимое для функционирования ROS.

Стандартные имена компьютеров: `proto2`, `proto3`.

Возможны две конфигурации сети:
* Роутер + бортовой компьютер + машина оператора. Сервисы предоставляет роутер, что существенно ограничивает совместимые 
* Бортовой компьютер + машина оператора. Сервисы предоставляет машина оператора.

### Полезные команды

* Запрос ассоциаций локального ntp сервера `ntpq -p` и его состояние `ntpstat`.
* Принудительная синхронизация с ntp сервером (локальный должен быть остановлен) `ntpdate <address>`.
* `cat /etc/resolv.conf`

### Конфигурация межсететвого экрана

Для функционирования ROS экран должен быть отключен между ROS-машинами. 

### Конфигурация роутрера `Turris Omnia`

Роутер настраивается через стандартный интерфейс со следующими ньюансами.
Использует [OpenWRT](https://openwrt.org/docs), конфигурационные файлы сервисов обновляются автоматически при загрузке
через [систему UCI](https://oldwiki.archive.openwrt.org/doc/uci) параметры которой сосредоточены в `/etc/config/`.

1. В Web-интерфейсе нельзя убрать доменное имя локальной сети. 
    Это проявлеятся в появлении в `resolv.conf` клинетских машин строчки `search <имя подсети>` и/или 
	не рабоающей команде разрешения по имени `nslookup <hostname>`.

	**Решение**: удалить из конфигурационных файлов `/etc/config/dhcp` и  `/etc/config/dhcp-opkg` строки
	```
doamin <network>
local <network>
    ```

2. Передача информации о `dns` и `ntp` сервера: опции 6 и 42 протокола DHCP соответственно 
	(должно работать через web-интерфейс, в крайнем случае отредактировать `/etc/config/dnsmasq`).

3. Настройка `ntpd`. Похоже, настройки UCI не затрагивают `ntpd`, а только `ntpdate`.
    * По умолчанию, `ntp` сервер не запускается. Требуется убрать из `/etc/rc.d/` ссылку на `ntpdate` и заменить на `ntpd`.
	* Настроить `ntpd`, редактируя `/etc/ntp.conf`


### Конфигурация `dnsmasq`

Настраивается на стороне машины оператора, если она должна играть роль сервера в случае отсутствия роутера.

1. Сетевой интерфейс должен быть сконфигурирован в режиме `static` в файле `/etc/network/interfaces`
    ```
iface eth0 inet static
address 192.168.3.1
mask 255.255.255.0
gateway 192.168.3.1
    ```

2. Конфигурация демона `dnsmasq` в `/etc/dnsmasq.conf`
    ```
listen-address=192.168.3.1
listen-address=127.0.0.1
dhcp-range=192.168.3.50,192.168.3.150,12h
dhcp-option=option:dns-server,192.168.3.1
dhcp-option=option:ntp-server,192.168.3.1
	```

### Конфигурация `dhclient`

Убедится, что запрашивает ntp-сервера и клиент передает свое имя.
```
option rfc3442-classless-static-routes code 121 = array of unsigned integer 8;

send host-name = gethostname();
request subnet-mask, broadcast-address, time-offset, routers,
	domain-name, domain-name-servers, domain-search, host-name,
	dhcp6.name-servers, dhcp6.domain-search, dhcp6.fqdn, dhcp6.sntp-servers,
	netbios-name-servers, netbios-scope, interface-mtu,
	rfc3442-classless-static-routes, ntp-servers;
```

Механизм раздачи адреса NTP сервера через DHCP следующий.
`dhclient` запускает `/etc/dhcp/dhclient-exit-hooks.d/ntp`, который создает временный файл конфигурации (`/run/` или в `/var/lib/ntp/`) 
и перезапускает NTP сервер с ним. Согласно документации **он должен быть исполнимым**. В armbian это было не так.

### Конфигурация `ntpd` на клиента и сервере

Серверы NTP симметричны. Они "меряются" только своими stratum, у кого меньше, то и прав. 
(stratum 0 --- атомные часы, stratum 1 --- то, что синхронизируется с атомными часами и т.д.)

Должен быть настроен на стороне роутера  как сервер, если RTC часы роутера надежны. 
В ином случае конфигурировать его, как клиента, чтобы синхронизировался с машиной оператора. 

Машина оператора настраивается как сервер, главное, устанавливать `stratum` ниже (больше по числовому значению), 
чем у роутера, если он тоже имеет часы.

Бортовой компьютер --- клиент.

Конфигурационный файл `/etc/ntp.conf` сервера

```
driftfile /var/lib/ntp/ntp.drift

# NTP server list. Add servers form internet.
server 0.ubuntu.pool.ntp.org
server 1.ubuntu.pool.ntp.org
server 2.ubuntu.pool.ntp.org
server 3.ubuntu.pool.ntp.org

# System RTC clocks is marked time source of 10 stratum.
# Must present only on servers.
server 127.127.1.0
fudge 127.127.1.0 stratum 10

# By default, exchange time with everybody, but don't allow configuration and assotiation
restrict -4 default kod notrap nomodify nopeer noquery
restrict -6 default kod notrap nomodify nopeer noquery

# Local users may interrogate the ntp server more closely.
restrict 192.168.3.1 mask 255.255.0.0 nomodify notrap
restrict 127.0.0.1
restrict ::1

# If you want to provide time to your local subnet, change the next line.
#broadcast 192.168.1.255

# If you want to listen to time broadcasts on your local subnet, de-comment the
# next lines.  Please do this only if you trust everybody on the network!
#disable auth
#broadcastclient

```
Самые важные строки --- это `server 127.127.1.0` и `fudge`. 
Они говорят, что часам реального времени можно доверять.

Конфигурационный файл `/etc/ntp.conf` сервера
```
driftfile /var/lib/ntp/ntp.drift

# NTP server list. Add servers form internet.
server 0.ubuntu.pool.ntp.org
server 1.ubuntu.pool.ntp.org
server 2.ubuntu.pool.ntp.org
server 3.ubuntu.pool.ntp.org

# By default, exchange time with everybody, but don't allow configuration and assotiation
restrict -4 default kod notrap nomodify nopeer noquery
restrict -6 default kod notrap nomodify nopeer noquery

# Local users may interrogate the ntp server more closely.
restrict 192.168.3.1 mask 255.255.0.0 nomodify notrap
restrict 127.0.0.1
restrict ::1

# If you want to listen to time broadcasts on your local subnet, de-comment the
# next lines.  Please do this only if you trust everybody on the network!
#disable auth
#broadcastclient

#Syncronize even if difference is very large.
tinker panic 0
```
Самая важная строка --- `tinker panic 0`. Она позволяет системе без RTC синхронизироваться с сервером.





