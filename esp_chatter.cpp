#include "ros.h"
#include "std_msgs/String.h"
#include "esp_chatter.h"
#include "esp_log.h"

static const char *TAG4 = "TCP->";

void _chat_recv_cb( const std_msgs::String& _msg){
    ESP_LOGI(TAG4, "msg recieved: %s",_msg.data);
}

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::String> chatter_recv("chatter_recv", &_chat_recv_cb );



char hello[512];

void rosserial_setup()
{
  // Initialize ROS
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(chatter_recv);

}

void rosserial_publish()
{


  memset(hello,'~',500);
  hello[500]='\0';
  str_msg.data = hello;
  // Send the message
  chatter.publish(&str_msg);
  nh.spinOnce();

}
