# Ультразвуковые сонары, ИК датчики динамиксель

Для управления динамикселем требуется библиотека - [AX-12A.h](https://github.com/ThingType/AX-12A-servo-library/tree/master/src). 

Для VL53 библиотека ставится с помощью менеджра библиотек


## запуск с ROSа
```
rosrun rosserial_python serial_node.py /dev/ttyUSB2 _name:="sonar_vl_dinam"
```
