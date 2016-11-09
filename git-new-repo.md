## Создание тестового репозитария и подключение к нему
1. Регистрируемся на сайте gitlab.com;
1. Кликаем на + вверху слева рядом с аватаром или переходим по ссылке [`https://gitlab.com/projects/new`](https://gitlab.com/projects/new)
1. Указываем имя репозитария;
1. Для примера создадим публичный репозитарий (выбираем public);
1. Нажимаем кнопку `Create project`
![gitlab_create_new_repo-fs8](/uploads/df8044e77c6feb032e3e4c9cd578b912/gitlab_create_new_repo-fs8.png)

Откроется страница нового репозитария. Там будет ссылка вида `git@gitlab.com:username/test_project.git` копируем её в буфер обмена.

## Клонирование репозитария
Создаём папку, например ~/repos/ и клонируем репозитарий себе:
```
$ mkdir -p ~/repos/
$ cd ~/repos/
$ git clone git@gitlab.com:username/test_project.git
```
При первом подключении может выдать предупреждение. Нужно ввести `yes` и нажать Enter.
```
The authenticity of host 'gitlab.com (104.210.2.228)' can't be established.
RSA key fingerprint is b6:03:0e:39:97:9e:d0:e7:24:ce:a3:77:3e:01:42:09.
Are you sure you want to continue connecting (yes/no)? yes
```
Должна появиться директория с именем репозитария.
## Первый комит
Заходим в директорию репозитария и создадим там файл `README.md`

В него можно добавить например описание репозитария. В качестве языка разметки может быть использован [Markdown](https://gitlab.com/help/user/markdown).
```
cd test_project
touch README.md
git add README.md
git commit -m "add README"
git push -u origin master
```
Содержимое файла теперь должно отображаться на странице репозитария в gitlab.

### Смотри далее:
* [Описание репозитариев проекта](git-project-repos);
* [Типичные операции с репозитарием](git-typical-operations);
* [Поток разработки](git-workflow).