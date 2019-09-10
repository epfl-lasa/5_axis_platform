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
      typedef int8_t _set_axis_type;
      _set_axis_type set_axis;
      float set_position[5];
      float set_effort[5];
      float set_twist[5];

    FootInputMsg_v2():
      set_axis(0),
      set_position(),
      set_effort(),
      set_twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_set_axis;
      u_set_axis.real = this->set_axis;
      *(outbuffer + offset + 0) = (u_set_axis.base >> (8 * 0)) & 0xFF;
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_set_axis;
      u_set_axis.base = 0;
      u_set_axis.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
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
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootInputMsg_v2"; };
    const char * getMD5(){ return "a6357f5fa0bbc6faeb620e9a6e5f5cdb"; };

  };

}
#endif