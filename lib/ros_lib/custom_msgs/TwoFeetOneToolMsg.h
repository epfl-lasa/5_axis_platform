#ifndef _ROS_custom_msgs_TwoFeetOneToolMsg_h
#define _ROS_custom_msgs_TwoFeetOneToolMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace custom_msgs
{

  class TwoFeetOneToolMsg : public ros::Msg
  {
    public:
      typedef uint8_t _currentTool_type;
      _currentTool_type currentTool;
      typedef uint8_t _currentControlMode_type;
      _currentControlMode_type currentControlMode;
      typedef uint8_t _mixedPlatformStateMachine_type;
      _mixedPlatformStateMachine_type mixedPlatformStateMachine;
      double mixedPlatformOffset[5];
      typedef sensor_msgs::JointState _mixedPlatformJointState_type;
      _mixedPlatformJointState_type mixedPlatformJointState;

    TwoFeetOneToolMsg():
      currentTool(0),
      currentControlMode(0),
      mixedPlatformStateMachine(0),
      mixedPlatformOffset(),
      mixedPlatformJointState()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->currentTool >> (8 * 0)) & 0xFF;
      offset += sizeof(this->currentTool);
      *(outbuffer + offset + 0) = (this->currentControlMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->currentControlMode);
      *(outbuffer + offset + 0) = (this->mixedPlatformStateMachine >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mixedPlatformStateMachine);
      for( uint32_t i = 0; i < 5; i++){
      union {
        double real;
        uint64_t base;
      } u_mixedPlatformOffseti;
      u_mixedPlatformOffseti.real = this->mixedPlatformOffset[i];
      *(outbuffer + offset + 0) = (u_mixedPlatformOffseti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mixedPlatformOffseti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mixedPlatformOffseti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mixedPlatformOffseti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mixedPlatformOffseti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mixedPlatformOffseti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mixedPlatformOffseti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mixedPlatformOffseti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mixedPlatformOffset[i]);
      }
      offset += this->mixedPlatformJointState.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->currentTool =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->currentTool);
      this->currentControlMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->currentControlMode);
      this->mixedPlatformStateMachine =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mixedPlatformStateMachine);
      for( uint32_t i = 0; i < 5; i++){
      union {
        double real;
        uint64_t base;
      } u_mixedPlatformOffseti;
      u_mixedPlatformOffseti.base = 0;
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mixedPlatformOffseti.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mixedPlatformOffset[i] = u_mixedPlatformOffseti.real;
      offset += sizeof(this->mixedPlatformOffset[i]);
      }
      offset += this->mixedPlatformJointState.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "custom_msgs/TwoFeetOneToolMsg"; };
    const char * getMD5(){ return "c13947aa600fe4cbf6de59e8c2aad533"; };

  };

}
#endif