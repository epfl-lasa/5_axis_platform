#ifndef _ROS_custom_msgs_FootInputMsg_v2_h
#define _ROS_custom_msgs_FootInputMsg_v2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class FootInputMsg_v2 : public ros::Msg
  {
    public:
      typedef int16_t _set_axis_type;
      _set_axis_type set_axis;
      float set_position[5];
      float set_effort[5];
      float set_twist[5];
      typedef int16_t _set_machine_state_type;
      _set_machine_state_type set_machine_state;
      int16_t wrench_comp[4];
      typedef int16_t _set_controller_type;
      _set_controller_type set_controller;
      typedef int16_t _default_gains_type;
      _default_gains_type default_gains;
      float pos_kp_gains[5];
      float pos_ki_gains[5];
      float pos_kd_gains[5];
      float speed_kp_gains[5];
      float speed_ki_gains[5];
      float speed_kd_gains[5];

    FootInputMsg_v2():
      set_axis(0),
      set_position(),
      set_effort(),
      set_twist(),
      set_machine_state(0),
      wrench_comp(),
      set_controller(0),
      default_gains(0),
      pos_kp_gains(),
      pos_ki_gains(),
      pos_kd_gains(),
      speed_kp_gains(),
      speed_ki_gains(),
      speed_kd_gains()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_set_axis;
      u_set_axis.real = this->set_axis;
      *(outbuffer + offset + 0) = (u_set_axis.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_axis.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->set_axis);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_set_positioni;
      u_set_positioni.real = this->set_position[i];
      *(outbuffer + offset + 0) = (u_set_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_set_efforti;
      u_set_efforti.real = this->set_effort[i];
      *(outbuffer + offset + 0) = (u_set_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_efforti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_effort[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_set_twisti;
      u_set_twisti.real = this->set_twist[i];
      *(outbuffer + offset + 0) = (u_set_twisti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_twisti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_twisti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_twisti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_twist[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_set_machine_state;
      u_set_machine_state.real = this->set_machine_state;
      *(outbuffer + offset + 0) = (u_set_machine_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_machine_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->set_machine_state);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_wrench_compi;
      u_wrench_compi.real = this->wrench_comp[i];
      *(outbuffer + offset + 0) = (u_wrench_compi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wrench_compi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wrench_comp[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_set_controller;
      u_set_controller.real = this->set_controller;
      *(outbuffer + offset + 0) = (u_set_controller.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_controller.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->set_controller);
      union {
        int16_t real;
        uint16_t base;
      } u_default_gains;
      u_default_gains.real = this->default_gains;
      *(outbuffer + offset + 0) = (u_default_gains.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_default_gains.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->default_gains);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_kp_gainsi;
      u_pos_kp_gainsi.real = this->pos_kp_gains[i];
      *(outbuffer + offset + 0) = (u_pos_kp_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_kp_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_kp_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_kp_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_kp_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_ki_gainsi;
      u_pos_ki_gainsi.real = this->pos_ki_gains[i];
      *(outbuffer + offset + 0) = (u_pos_ki_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_ki_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_ki_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_ki_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_ki_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_kd_gainsi;
      u_pos_kd_gainsi.real = this->pos_kd_gains[i];
      *(outbuffer + offset + 0) = (u_pos_kd_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_kd_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_kd_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_kd_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_kd_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_kp_gainsi;
      u_speed_kp_gainsi.real = this->speed_kp_gains[i];
      *(outbuffer + offset + 0) = (u_speed_kp_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_kp_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_kp_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_kp_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_kp_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_ki_gainsi;
      u_speed_ki_gainsi.real = this->speed_ki_gains[i];
      *(outbuffer + offset + 0) = (u_speed_ki_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_ki_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_ki_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_ki_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_ki_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_kd_gainsi;
      u_speed_kd_gainsi.real = this->speed_kd_gains[i];
      *(outbuffer + offset + 0) = (u_speed_kd_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_kd_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_kd_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_kd_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_kd_gains[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_set_axis;
      u_set_axis.base = 0;
      u_set_axis.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_axis.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->set_axis = u_set_axis.real;
      offset += sizeof(this->set_axis);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_set_positioni;
      u_set_positioni.base = 0;
      u_set_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_position[i] = u_set_positioni.real;
      offset += sizeof(this->set_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_set_efforti;
      u_set_efforti.base = 0;
      u_set_efforti.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_efforti.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_efforti.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_efforti.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_effort[i] = u_set_efforti.real;
      offset += sizeof(this->set_effort[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_set_twisti;
      u_set_twisti.base = 0;
      u_set_twisti.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_twisti.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_twisti.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_twisti.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_twist[i] = u_set_twisti.real;
      offset += sizeof(this->set_twist[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_set_machine_state;
      u_set_machine_state.base = 0;
      u_set_machine_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_machine_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->set_machine_state = u_set_machine_state.real;
      offset += sizeof(this->set_machine_state);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_wrench_compi;
      u_wrench_compi.base = 0;
      u_wrench_compi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wrench_compi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wrench_comp[i] = u_wrench_compi.real;
      offset += sizeof(this->wrench_comp[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_set_controller;
      u_set_controller.base = 0;
      u_set_controller.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_controller.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->set_controller = u_set_controller.real;
      offset += sizeof(this->set_controller);
      union {
        int16_t real;
        uint16_t base;
      } u_default_gains;
      u_default_gains.base = 0;
      u_default_gains.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_default_gains.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->default_gains = u_default_gains.real;
      offset += sizeof(this->default_gains);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_kp_gainsi;
      u_pos_kp_gainsi.base = 0;
      u_pos_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_kp_gains[i] = u_pos_kp_gainsi.real;
      offset += sizeof(this->pos_kp_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_ki_gainsi;
      u_pos_ki_gainsi.base = 0;
      u_pos_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_ki_gains[i] = u_pos_ki_gainsi.real;
      offset += sizeof(this->pos_ki_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_kd_gainsi;
      u_pos_kd_gainsi.base = 0;
      u_pos_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_kd_gains[i] = u_pos_kd_gainsi.real;
      offset += sizeof(this->pos_kd_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_kp_gainsi;
      u_speed_kp_gainsi.base = 0;
      u_speed_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_kp_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_kp_gains[i] = u_speed_kp_gainsi.real;
      offset += sizeof(this->speed_kp_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_ki_gainsi;
      u_speed_ki_gainsi.base = 0;
      u_speed_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_ki_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_ki_gains[i] = u_speed_ki_gainsi.real;
      offset += sizeof(this->speed_ki_gains[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_kd_gainsi;
      u_speed_kd_gainsi.base = 0;
      u_speed_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_kd_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_kd_gains[i] = u_speed_kd_gainsi.real;
      offset += sizeof(this->speed_kd_gains[i]);
      }
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootInputMsg_v2"; };
    const char * getMD5(){ return "e268400ca8bbd6cd7c628338404b28d0"; };

  };

}
#endif