# Установка и настройка git на windows
В данном руководстве описан процесс установки и настройки git на windows.

Описание данного процесса на linux смотри [тут](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/git-install-linux).

## Установка git
Скачиваем установщик с [официального сайта](https://git-scm.com/download/win).

![01](/uploads/ec9ed28b97142269880271d0bf9f8447/01.png)

![02](/uploads/38ea145eee671da9c9d8932d4cb5b18c/02.png)

![03](/uploads/568993ffad98b44fba6d60bc77d96f28/03.png)

### Генерация SSH ключа
Генерация ключа необходима чтобы была возможность работать с репозитарием по протоколу SSH.

Запускаем Пуск -> Git -> Git GUI и генерируем в нем новый ключ.

![04](/uploads/2c1f359264ad2a4d29491f7501f1e4cf/04.png)

И вставляем его в свой профиль на гитлабе в разделе [SSH keys](https://gitlab.com/profile/keys). В поле Title можно ввести имя компьютера или пользователя.
![2016-10-26_12-05-45](/uploads/a6659490f8c24f72f939d19c542deda9/2016-10-26_12-05-45.png)

Ключи хранятся в каталоге пользователя `C:\Users\Username\.ssh`

Запускаем cmd или Git bash и клонируем репозитарий:

При первом подключении спросит `Are you sure you want to continue connecting (yes/no)?` нужно ввести **yes**.

```
D:\files\repos\sweetiebot>git clone git@gitlab.com:sweetiebot/electro.git
Cloning into 'electro'...
The authenticity of host 'gitlab.com (104.210.2.228)' can't be established.
ECDSA key fingerprint is SHA256:HbW3g8zUjNSksFbqTiUWPWg2Bq1x8xdGUrliXFzSnUw.
Are you sure you want to continue connecting (yes/no)? yes
Warning: Permanently added 'gitlab.com,104.210.2.228' (ECDSA) to the list of known hosts.
remote: Counting objects: 248, done.
remote: Compressing objects: 100% (93/93), done.
Receiving objects: 100% (248/248), 1019.33 KiB | 1.09 MiB/s, done.
remote: Total 248 (delta 123), reused 232 (delta 112)
Resolving deltas: 100% (123/123), done.
```

## Настройка git
Обязательные настройки:
```
git config --global user.name "your_name"
git config --global user.email "your_email@example.com"
```
Рекомендованные настройки:
```
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.st status
git config --global alias.hist "log --pretty=format:'%h %ad | %s%d [%an]' --graph --date=short"
git config --global alias.type 'cat-file -t'
git config --global alias.dump 'cat-file -p'
git config --global push.default simple
```

### Смотри далее:
* [Установка git на linux](git-install-linux);
* [Создание и работа с тестовым репозитарием](git-new-repo);
* [Описание репозитариев проекта](git-project-repos);
* [Типичные операции с репозитарием](git-typical-operations);
* [Поток разработки](git-workflow).