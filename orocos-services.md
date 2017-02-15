# Сервисы и плагины
Сервисы позволяют создавать связанные группы элементов внешнего интерфейса объекта.  Такая группа может включать операции, порты и параметры.
Плагины предназначены для динамического расширения функциональности компонента и представляет собой реализацию сервиса, 
которую можно добавить в компонент.

Таким образом, плагин способен расширить интерфейс компонента за счет портов, операций и параметров,
которые будут доступны в пространстве имен, совпадающем с именем, под которым был загружен плагин (сервис):

    component.portname // порт компонента
    component.pluginname.portname // порт плагина

Однако, поскольку плагин имеет доступ к `TaskContext` исходного компонента, то ничего не препятствует модификации 
интерфейса, загрузившего его компонента, но это не желательно.

Исполнение кода сервиса занимается `ExecutionEngine` исходного компонента, однако аналогов `startHook`, `updateHook` и т.п. у него нет.
Операции пишутся стандартным образом, для обработки сообщений можно использовать callback, задаваемый при вызове `addEventPort`.

Пусть описан сервис:

```cpp
    class HelloInterface  // программисткий интерфейс
    {
        public:
            virtual void hello() = 0; 
    }
    
    class HelloService : public HelloInterface, public RTT::Service  // реализация сервиса
    {
        public:
            HelloService(TaskContext * tc) : 
            Service("hello", tc) 
            {
                addOperation("hello", &hello, this);
            }
            void hello() {
                cout << "Hello, world!" << endl;
            }
    }
    
    class Hello : public ServiceRequester { // внешний интерфейс для удаленного вызова операций сервиса (опционален)
        public: 
            OperationCaller<void()> hello;
    
            Hello(TaskContext * owner) :
                ServiceRequester("hello", owner),
                hello("hello")
            {
                addOperationCaller(hello);
            }
    }
```

Использовать сервисы и плагины можно разным способами.

#### 1. Использование сервиса (плагина) удаленного компонента. 
Сервис предоставляется удаленным компонентом, его операции используются в другом компоненте.

Конструктор компонента, предоставляющего сервис:

```cpp
         ProviderComponent() {
             shared_ptr<HelloService> hello_serv = new HelloService("hello", this);
             this->provides()->addService(hello_serv);
         }
```
Компонент, использующий сервис:

```cpp
         class RequesterComponent() {
             shared_ptr<Hello> hello_req; // boost::shared_ptr
         public:
             RequesterComponent() {
                 hello_req = new Hello(this);
                 this->requires()->addServiceRequester(hello_req);
             }
             configureHook() {
                 return hello_req->ready(); // проверить, доступны ли его операции
             }
             updateHook() {
                 hello_req->hello();   
                 this->requires("hello")->hello();
             }
          }
```

Связывание `ServiceRequester` и `Service` разных компонент осуществляется вызовом deployer `connectServices()`, 
соединяющим *одноименные* сервисы. У `ServiceRequester` есть также метод `connectTo()`.

`ops файл:`
```
          loadService("requester_component", "hello");
          requester_component.loadService("hello");
```     

#### 2. Использование сервиса (плагина) текущего компонента при помощи внешнего интерфейса:

Этот способ дает доступ только к элементам интерфейса сервиса, зарегистрированных в OROCOS как внешние.

```cpp
         class Component() {
             shared_ptr<Hello> hello_req;
         public:
             configureHook() {
                 hello_req = getProvider<Hello>("hello"); //попытается загрузить нужный сервис, если он отсутствует.
                 return hello_req != 0;
             }
             updateHook() {
                 hello_req->hello();   
                 getProvider<Hello>("hello")->hello();
             }
          }
```

#### 3. Использование сервиса (плагина) текущего компонента при помощи внутреннего интерфейса:

В этом случае не происходит регистрация элементов внешнего интерфейса. Поэтому этот способ больше всего подходит для взаимодействия с элементами сервиса, не предназначенными для использованиями компонентами, не являющимися владельцами сервиса. 

```cpp
         class Component() {
         public:
             shared_ptr<HelloInterface *> hello_if
             configureHook() {
				 hello_if = dynamic_pointer_cast<HelloInterface>(getService("hello"));
                 return hello_if != 0;
             }
             updateHook() {
                 hello_if->hello();
             }
          }
```



Нет стандартных способов загрузить плагин не под его именем. Т.е. тип жестко связан с именем, 
Отчасти это можно решить, добавив операции, получающую имя плагина/сервиса в виде параметра:

```cpp
    loadHello(string& name) {
        hello_req = getProvider<Hello>(name);
        hello_if = dynamic_pointer_cast<HelloInterface>(getService("hello"));
    }
```