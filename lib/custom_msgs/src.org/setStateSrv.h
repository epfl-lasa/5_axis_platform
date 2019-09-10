#ifndef _ROS_SERVICE_setStateSrv_h
#define _ROS_SERVICE_setStateSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

static const char SETSTATESRV[] = "custom_msgs/setStateSrv";

  class setStateSrvRequest : public ros::Msg
  {
    public:
      typedef int8_t _machine_state_type;
      _machine_state_type machine_state;
      uint8_t wrench_comp[4];

    setStateSrvRequest():
      machine_state(0),
      wrench_comp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_machine_state;
      u_machine_state.real = this->machine_state;
      *(outbuffer + offset + 0) = (u_machine_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->machine_state);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->wrench_comp[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wrench_comp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_machine_state;
      u_machine_state.base = 0;
      u_machine_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->machine_state = u_machine_state.real;
      offset += sizeof(this->machine_state);
      for( uint32_t i = 0; i < 4; i++){
      this->wrench_comp[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->wrench_comp[i]);
      }
     return offset;
    }

    const char * getType(){ return SETSTATESRV; };
    const char * getMD5(){ return "26fba948b673492f84a209420a9ce591"; };

  };

  class setStateSrvResponse : public ros::Msg
  {
    public:
      typedef bool _mS_new_type;
      _mS_new_type mS_new;

    setStateSrvResponse():
      mS_new(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_mS_new;
      u_mS_new.real = this->mS_new;
      *(outbuffer + offset + 0) = (u_mS_new.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mS_new);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_mS_new;
      u_mS_new.base = 0;
      u_mS_new.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mS_new = u_mS_new.real;
      offset += sizeof(this->mS_new);
     return offset;
    }

    const char * getType(){ return SETSTATESRV; };
    const char * getMD5(){ return "b0d7765d1d55d1af98b2141df9232de5"; };

  };

  class setStateSrv {
    public:
    typedef setStateSrvRequest Request;
    typedef setStateSrvResponse Response;
  };

}
#endif
