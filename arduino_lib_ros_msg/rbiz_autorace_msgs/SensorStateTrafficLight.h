#ifndef _ROS_rbiz_autorace_msgs_SensorStateTrafficLight_h
#define _ROS_rbiz_autorace_msgs_SensorStateTrafficLight_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace rbiz_autorace_msgs
{

  class SensorStateTrafficLight : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      uint16_t sensor_distance[3];
      typedef float _elapsed_time_type;
      _elapsed_time_type elapsed_time;
      uint8_t is_started[3];
      typedef uint8_t _is_able_to_pass_type;
      _is_able_to_pass_type is_able_to_pass;
      typedef int32_t _vehicle_state_type;
      _vehicle_state_type vehicle_state;
      typedef int32_t _led_color_type;
      _led_color_type led_color;
      typedef float _battery_type;
      _battery_type battery;

    SensorStateTrafficLight():
      stamp(),
      sensor_distance(),
      elapsed_time(0),
      is_started(),
      is_able_to_pass(0),
      vehicle_state(0),
      led_color(0),
      battery(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->sensor_distance[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_distance[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sensor_distance[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_elapsed_time;
      u_elapsed_time.real = this->elapsed_time;
      *(outbuffer + offset + 0) = (u_elapsed_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elapsed_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elapsed_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elapsed_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->elapsed_time);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->is_started[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_started[i]);
      }
      *(outbuffer + offset + 0) = (this->is_able_to_pass >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_able_to_pass);
      union {
        int32_t real;
        uint32_t base;
      } u_vehicle_state;
      u_vehicle_state.real = this->vehicle_state;
      *(outbuffer + offset + 0) = (u_vehicle_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vehicle_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vehicle_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vehicle_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vehicle_state);
      union {
        int32_t real;
        uint32_t base;
      } u_led_color;
      u_led_color.real = this->led_color;
      *(outbuffer + offset + 0) = (u_led_color.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_led_color.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_led_color.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_led_color.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->led_color);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      for( uint32_t i = 0; i < 3; i++){
      this->sensor_distance[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->sensor_distance[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sensor_distance[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_elapsed_time;
      u_elapsed_time.base = 0;
      u_elapsed_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elapsed_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elapsed_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elapsed_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->elapsed_time = u_elapsed_time.real;
      offset += sizeof(this->elapsed_time);
      for( uint32_t i = 0; i < 3; i++){
      this->is_started[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->is_started[i]);
      }
      this->is_able_to_pass =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->is_able_to_pass);
      union {
        int32_t real;
        uint32_t base;
      } u_vehicle_state;
      u_vehicle_state.base = 0;
      u_vehicle_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vehicle_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vehicle_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vehicle_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vehicle_state = u_vehicle_state.real;
      offset += sizeof(this->vehicle_state);
      union {
        int32_t real;
        uint32_t base;
      } u_led_color;
      u_led_color.base = 0;
      u_led_color.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_led_color.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_led_color.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_led_color.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->led_color = u_led_color.real;
      offset += sizeof(this->led_color);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
     return offset;
    }

    const char * getType(){ return "rbiz_autorace_msgs/SensorStateTrafficLight"; };
    const char * getMD5(){ return "b423d5d5817b5271afa821c767c480c9"; };

  };

}
#endif