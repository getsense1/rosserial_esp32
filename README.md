## rosserial_esp32


This [rosserial](http://wiki.ros.org/rosserial) ros_lib ESP32 port 
that enable communication between ROS(Kinetic on Ubuntu Trusty 14.04 LTS) node and ESP32(not the Arduino framework) using ESP-IDF TCP stack.


```
how work
$ git clone https://github.com/getsense1/rosserial_esp32
$ cd ./rosserial_esp32
$ cp -fr ./esp-idf/components/rosserial_esp32 esp/esp-idf/components/
copy _main.c, esp_chatter.cpp, include to IDF project folder
build and flash bin image to ESP32 board 
$ rosrun rosserial_python serial_node.py tcp
reset ESP32 board
look at on screen a Msg "~..." size 500 bytes buffer from Publisher topic "chatter" in the ROS env repeat again
$ rostopic echo chatter
take a look bit rate from Publisher "chatter"
$ rostopic hz chatter
on screen the following:
average rate: 377.732
	min: 0.000s max: 0.053s std: dev: 0.00434s window: 50000
377msg/sec, msg->500 bytes -> 377*500*8=1508000 bit/sec

send a Msg to Subscriber topic "chatter_recv"
$ rostopic type chatter_recv
$ rosmsg show std msgs/String
$ rostopic pub chatter_recv std msgs/String -- '~Hello world~'
look at in the ESP-IDF env, like make monitor ->  string '~Hello world~' on screen
```

