#ifndef _ROS_SERVICE_setControllerSrv_h
#define _ROS_SERVICE_setControllerSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

static const char SETCONTROLLERSRV[] = "custom_msgs/setControllerSrv";

  class setControllerSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _ros_controllerType_type;
      _ros_controllerType_type ros_controllerType;
      typedef bool _ros_defaultControl_type;
      _ros_defaultControl_type ros_defaultControl;
      typedef int8_t _ros_controlledAxis_type;
      _ros_controlledAxis_type ros_controlledAxis;

    setControllerSrvRequest():
      ros_controllerType(0),
      ros_defaultControl(0),
      ros_controlledAxis(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ros_controllerType >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_controllerType);
      union {
        bool real;
        uint8_t base;
      } u_ros_defaultControl;
      u_ros_defaultControl.real = this->ros_defaultControl;
      *(outbuffer + offset + 0) = (u_ros_defaultControl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_defaultControl);
      union {
        int8_t real;
        uint8_t base;
      } u_ros_controlledAxis;
      u_ros_controlledAxis.real = this->ros_controlledAxis;
      *(outbuffer + offset + 0) = (u_ros_controlledAxis.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_controlledAxis);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->ros_controllerType =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ros_controllerType);
      union {
        bool real;
        uint8_t base;
      } u_ros_defaultControl;
      u_ros_defaultControl.base = 0;
      u_ros_defaultControl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_defaultControl = u_ros_defaultControl.real;
      offset += sizeof(this->ros_defaultControl);
      union {
        int8_t real;
        uint8_t base;
      } u_ros_controlledAxis;
      u_ros_controlledAxis.base = 0;
      u_ros_controlledAxis.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_controlledAxis = u_ros_controlledAxis.real;
      offset += sizeof(this->ros_controlledAxis);
     return offset;
    }

    const char * getType(){ return SETCONTROLLERSRV; };
    const char * getMD5(){ return "dfecad7c4bd7acd4a30656a865a27ce2"; };

  };

  class setControllerSrvResponse : public ros::Msg
  {
    public:
      typedef bool _platform_controlOk_type;
      _platform_controlOk_type platform_controlOk;

    setControllerSrvResponse():
      platform_controlOk(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_controlOk;
      u_platform_controlOk.real = this->platform_controlOk;
      *(outbuffer + offset + 0) = (u_platform_controlOk.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_controlOk);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_controlOk;
      u_platform_controlOk.base = 0;
      u_platform_controlOk.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_controlOk = u_platform_controlOk.real;
      offset += sizeof(this->platform_controlOk);
     return offset;
    }

    const char * getType(){ return SETCONTROLLERSRV; };
    const char * getMD5(){ return "b3690307401e27c573abefe10dae9da3"; };

  };

  class setControllerSrv {
    public:
    typedef setControllerSrvRequest Request;
    typedef setControllerSrvResponse Response;
  };

}
#endif
