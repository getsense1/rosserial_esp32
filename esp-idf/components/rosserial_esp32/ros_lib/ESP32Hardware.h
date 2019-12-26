#ifndef ROS_ESP32_HARDWARE_H_
#define ROS_ESP32_HARDWARE_H_

extern "C" {
#include "stdio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"


void initialise_wifi(void);
void wait_for_ip();
unsigned char _conn_ros(void);
unsigned char _ros_recv(unsigned char *p_buff,unsigned short _size);
int _ros_send(unsigned char *p_buff,unsigned short _size);

}
static const char *TAG = "TCP->";

class ESP32Hardware
{
    protected:
        uint8_t rx_buf[1024];

    public:
        ESP32Hardware()
        {
        }

        // Initialization code for ESP32
        void init(){
            unsigned char _res;

            initialise_wifi();
            wait_for_ip();
            _res=_conn_ros();
            if(!_res){
                ESP_LOGE(TAG, "Connection to ROS faild\n");
            }
        }

        // read a byte from the serial port. -1 = failure
        int read(int *_data){
            int _res;
            _res = _ros_recv((unsigned char *)rx_buf,1);
            if (_res == 1) {
                 //ESP_LOGE(TAG, "recv buf->%x",rx_buf[0]);
                *_data = rx_buf[0];
                return 0x1;
            } else {
                //ESP_LOGE(TAG, "Recv from ROS faild");
                return _res;
            }
        }

        // write data to the connection to ROS
        void write(uint8_t* data, int len){
            int _res;
            int length = len;
            int totalsent = 0;
                while (totalsent < length){
                    _res = _ros_send((unsigned char *)(data+totalsent),(size_t)length);
                    if(_res < 0){
                        ;//ESP_LOGE(TAG, "Sending to ROS faild");
                    }else{
                        totalsent += _res;
                        //ESP_LOGE(TAG, "sent buf length->%d",_res);
                    }
                }
        }

        // returns milliseconds since start of program
        unsigned long time(){
            unsigned long _time_ms = esp_timer_get_time() / 1000;

                //ESP_LOGE(TAG, "curr time->%d ms",(unsigned int)_time_ms);
                return _time_ms;
        }
};
#endif
