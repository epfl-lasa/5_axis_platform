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
      typedef uint8_t _id_type;
      _id_type id;
      float position[5];
      float speed[5];
      float ctrl_efforts[5];
      float meas_efforts[5];
      typedef uint8_t _controller_type_type;
      _controller_type_type controller_type;
      typedef uint8_t _machine_state_type;
      _machine_state_type machine_state;

    FootOutputMsg_v2():
      stamp(),
      id(0),
      position(),
      speed(),
      ctrl_efforts(),
      meas_efforts(),
      controller_type(0),
      machine_state(0)
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
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speedi;
      u_speedi.real = this->speed[i];
      *(outbuffer + offset + 0) = (u_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ctrl_effortsi;
      u_ctrl_effortsi.real = this->ctrl_efforts[i];
      *(outbuffer + offset + 0) = (u_ctrl_effortsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ctrl_effortsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ctrl_effortsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ctrl_effortsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ctrl_efforts[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_meas_effortsi;
      u_meas_effortsi.real = this->meas_efforts[i];
      *(outbuffer + offset + 0) = (u_meas_effortsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_meas_effortsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_meas_effortsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_meas_effortsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meas_efforts[i]);
      }
      *(outbuffer + offset + 0) = (this->controller_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_type);
      *(outbuffer + offset + 0) = (this->machine_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->machine_state);
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
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.base = 0;
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position[i] = u_positioni.real;
      offset += sizeof(this->position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speedi;
      u_speedi.base = 0;
      u_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed[i] = u_speedi.real;
      offset += sizeof(this->speed[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ctrl_effortsi;
      u_ctrl_effortsi.base = 0;
      u_ctrl_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ctrl_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ctrl_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ctrl_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ctrl_efforts[i] = u_ctrl_effortsi.real;
      offset += sizeof(this->ctrl_efforts[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_meas_effortsi;
      u_meas_effortsi.base = 0;
      u_meas_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_meas_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_meas_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_meas_effortsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->meas_efforts[i] = u_meas_effortsi.real;
      offset += sizeof(this->meas_efforts[i]);
      }
      this->controller_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_type);
      this->machine_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->machine_state);
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootOutputMsg_v2"; };
    const char * getMD5(){ return "bdc2e1c2a3252f2109c35a5f1eeafcde"; };

  };

}
#endif