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
      typedef int8_t _ros_machineState_type;
      _ros_machineState_type ros_machineState;
      uint32_t ros_effortComp_length;
      typedef uint8_t _ros_effortComp_type;
      _ros_effortComp_type st_ros_effortComp;
      _ros_effortComp_type * ros_effortComp;

    setStateSrvRequest():
      ros_machineState(0),
      ros_effortComp_length(0), ros_effortComp(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ros_machineState;
      u_ros_machineState.real = this->ros_machineState;
      *(outbuffer + offset + 0) = (u_ros_machineState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_machineState);
      *(outbuffer + offset + 0) = (this->ros_effortComp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ros_effortComp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ros_effortComp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ros_effortComp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_effortComp_length);
      for( uint32_t i = 0; i < ros_effortComp_length; i++){
      *(outbuffer + offset + 0) = (this->ros_effortComp[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_effortComp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ros_machineState;
      u_ros_machineState.base = 0;
      u_ros_machineState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_machineState = u_ros_machineState.real;
      offset += sizeof(this->ros_machineState);
      uint32_t ros_effortComp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ros_effortComp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ros_effortComp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ros_effortComp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ros_effortComp_length);
      if(ros_effortComp_lengthT > ros_effortComp_length)
        this->ros_effortComp = (uint8_t*)realloc(this->ros_effortComp, ros_effortComp_lengthT * sizeof(uint8_t));
      ros_effortComp_length = ros_effortComp_lengthT;
      for( uint32_t i = 0; i < ros_effortComp_length; i++){
      this->st_ros_effortComp =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_ros_effortComp);
        memcpy( &(this->ros_effortComp[i]), &(this->st_ros_effortComp), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return SETSTATESRV; };
    const char * getMD5(){ return "a5055ad8e0b2a6be8a39c16fccf50195"; };

  };

  class setStateSrvResponse : public ros::Msg
  {
    public:
      typedef bool _platform_newState_type;
      _platform_newState_type platform_newState;

    setStateSrvResponse():
      platform_newState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_newState;
      u_platform_newState.real = this->platform_newState;
      *(outbuffer + offset + 0) = (u_platform_newState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_newState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_newState;
      u_platform_newState.base = 0;
      u_platform_newState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_newState = u_platform_newState.real;
      offset += sizeof(this->platform_newState);
     return offset;
    }

    const char * getType(){ return SETSTATESRV; };
    const char * getMD5(){ return "11d99a622803a32975b5ee3ee7b6fdad"; };

  };

  class setStateSrv {
    public:
    typedef setStateSrvRequest Request;
    typedef setStateSrvResponse Response;
  };

}
#endif
