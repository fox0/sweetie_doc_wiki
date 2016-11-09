# Установка и настройка git
В данном руководстве описан процесс установки и настройки git на linux.

Описание данного процесса на Windows смотри [тут](git-install-windows).

## Установка git
В линуксе обычно достаточно установить git из официального репозитария. 

На Ubuntu 14.04 как и на большинстве deb дистрибутивов для этого необходимо выполнить в терминале:
```
$ sudo apt-get update
$ sudo apt-get install git
```
### Генерация SSH ключа
Генерация ключа необходима чтобы была возможность работать с репозитарием по протоколу SSH.
```
$ ssh-keygen -t rsa -C "your_email@example.com"
Generating public/private rsa key pair.
Enter file in which to save the key (/home/user/.ssh/id_rsa): [ENTER (оставляем по умолчанию)]
Created directory '/home/user/.ssh'.
Enter passphrase (empty for no passphrase): [тут тоже нажать ENTER (без пароля)]
Enter same passphrase again: [еще раз ENTER]
Your identification has been saved in /home/user/.ssh/id_rsa.
Your public key has been saved in /home/user/.ssh/id_rsa.pub.
The key fingerprint is:
0d:cb:7a:1a:91:3e:29:5e:76:d7:56:98:ff:c9:4b:f9 your_email@example.com
```
В папке `~/.ssh/` появятся два файла `id_rsa` и `id_rsa.pub`

В первом содержится приватный ключ, во втором публичный.

## Добавление публичного ключа в свой профиль на gitlab
Передача публичного ключа необходима для работы с репозитарием без ввода пароля.

Копируем содержимое файла `id_rsa.pub` в буфер обмена.
```
$ cat ~/.ssh/id_rsa.pub
```
И вставляем его в свой профиль на гитлабе в разделе [SSH keys](https://gitlab.com/profile/keys). В поле Title можно ввести имя компьютера или пользователя.
![2016-10-26_12-05-45](/uploads/a6659490f8c24f72f939d19c542deda9/2016-10-26_12-05-45.png)

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
* [Установка git на windows](git-install-windows);
* [Создание и работа с тестовым репозитарием](git-new-repo);
* [Описание репозитариев проекта](git-project-repos);
* [Типичные операции с репозитарием](git-typical-operations);
* [Поток разработки](git-workflow).