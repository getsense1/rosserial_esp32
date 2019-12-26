#include "ros.h"
#include "std_msgs/String.h"
#include "esp_chatter.h"
#include "esp_log.h"

#define _PKG_TIMES_MAX  50

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
unsigned long _time_ms_start;
unsigned long _time_ms_end;
unsigned short _pkg_times=0;

  memset(hello,'~',500);
  hello[500]='\0';
  str_msg.data = hello;
  while(1) {
    if(_pkg_times==0){
        _time_ms_start = esp_timer_get_time() / 1000;
    }
    // Send the message
    chatter.publish(&str_msg);
    if(nh.spinOnce()<0){
        ESP_LOGE(TAG4, "spinOnce -> ERROR");
        continue;
    }
    _pkg_times += 1;
    if(_pkg_times==_PKG_TIMES_MAX){
        _pkg_times=0;
        _time_ms_end = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG4, "time spend: %d ms",int32_t(_time_ms_end-_time_ms_start));
    }
  }

}
