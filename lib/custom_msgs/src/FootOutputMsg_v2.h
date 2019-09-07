#ifndef _ROS_custom_msgs_FootOutputMsg_v2_h
#define _ROS_custom_msgs_FootOutputMsg_v2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace custom_msgs
{

  class FootOutputMsg_v2 : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int16_t _id_type;
      _id_type id;
      float joint_position[5];
      float joint_speed[5];
      float desired_efforts[5];
      float actual_efforts[5];
      typedef int16_t _state_type;
      _state_type state;

    FootOutputMsg_v2():
      stamp(),
      id(0),
      joint_position(),
      joint_speed(),
      desired_efforts(),
      actual_efforts(),
      state(0)
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
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_positioni;
      u_joint_positioni.real = this->joint_position[i];
      *(outbuffer + offset + 0) = (u_joint_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_speedi;
      u_joint_speedi.real = this->joint_speed[i];
      *(outbuffer + offset + 0) = (u_joint_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_speed[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_desired_effortsi;
      u_desired_effortsi.real = this->desired_efforts[i];
      *(outbuffer + offset + 0) = (u_desired_effortsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_desired_effortsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_desired_effortsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_desired_effortsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->desired_efforts[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_actual_effortsi;
      u_actual_effortsi.real = this->actual_efforts[i];
      *(outbuffer + offset + 0) = (u_actual_effortsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_actual_effortsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_actual_effortsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_actual_effortsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actual_efforts[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
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
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_positioni;
      u_joint_positioni.base = 0;
      u_joint_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_position[i] = u_joint_positioni.real;
      offset += sizeof(this->joint_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_speedi;
      u_joint_speedi.base = 0;
      u_joint_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_speed[i] = u_joint_speedi.real;
      offset += sizeof(this->joint_speed[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_desired_effortsi;
      u_desired_effortsi.base = 0;
      u_desired_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_desired_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_desired_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_desired_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->desired_efforts[i] = u_desired_effortsi.real;
      offset += sizeof(this->desired_efforts[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_actual_effortsi;
      u_actual_effortsi.base = 0;
      u_actual_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_actual_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_actual_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_actual_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->actual_efforts[i] = u_actual_effortsi.real;
      offset += sizeof(this->actual_efforts[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootOutputMsg_v2"; };
    const char * getMD5(){ return "eddadd050c6c8746c76f9596a2a9b3c8"; };

  };

}
#endif