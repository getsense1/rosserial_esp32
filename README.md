## rosserial_esp32


This [rosserial](http://wiki.ros.org/rosserial) ros_lib ESP32 port 
that enable communication between ROS(Kinetic on Ubuntu Trusty 14.04 LTS) node and ESP32(not the Andruino framework) using ESP-IDF TCP stack.


```
how work
$ git clone https://github.com/getsense1/rosserial_esp32
$ cd ./rosserial_esp32
$ cp -fr ./esp-idf/components/rosserial_esp32 esp/esp-idf/components/
copy _main.c, esp_chatter.cpp to IDF project folder
build and flash bin image to ESP32 board 
$ rosrun rosserial_python serial_node.py tcp
reset ESP32 board
look at the message "Hello world" in the ROS env
$ rostopic echo chatter
```

