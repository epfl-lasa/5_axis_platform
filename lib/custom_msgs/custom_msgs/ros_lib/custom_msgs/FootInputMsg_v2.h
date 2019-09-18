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
      float ros_position[5];
      float ros_effort[5];
      float ros_speed[5];

    FootInputMsg_v2():
      ros_position(),
      ros_effort(),
      ros_speed()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_positioni;
      u_ros_positioni.real = this->ros_position[i];
      *(outbuffer + offset + 0) = (u_ros_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_efforti;
      u_ros_efforti.real = this->ros_effort[i];
      *(outbuffer + offset + 0) = (u_ros_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_efforti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_effort[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedi;
      u_ros_speedi.real = this->ros_speed[i];
      *(outbuffer + offset + 0) = (u_ros_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speed[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_positioni;
      u_ros_positioni.base = 0;
      u_ros_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_position[i] = u_ros_positioni.real;
      offset += sizeof(this->ros_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_efforti;
      u_ros_efforti.base = 0;
      u_ros_efforti.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_efforti.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_efforti.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_efforti.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_effort[i] = u_ros_efforti.real;
      offset += sizeof(this->ros_effort[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedi;
      u_ros_speedi.base = 0;
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speed[i] = u_ros_speedi.real;
      offset += sizeof(this->ros_speed[i]);
      }
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootInputMsg_v2"; };
    const char * getMD5(){ return "095e5c7c22c8b78b64735e710025ea3a"; };

  };

}
#endif