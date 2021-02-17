#ifndef _ROS_custom_msgs_SurgicalTaskStateMsg_h
#define _ROS_custom_msgs_SurgicalTaskStateMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"

namespace custom_msgs
{

  class SurgicalTaskStateMsg : public ros::Msg
  {
    public:
      typedef ros::Time _surgicalTaskStamp_type;
      _surgicalTaskStamp_type surgicalTaskStamp;
      typedef uint8_t _surgicalTaskState_type;
      _surgicalTaskState_type surgicalTaskState;
      uint8_t robotsToolState[2];
      geometry_msgs::Pose allToolsPoseWRTImage[4];
      geometry_msgs::Twist allToolsTwistWRTImage[4];
      geometry_msgs::Wrench allToolsWrench[4];

    SurgicalTaskStateMsg():
      surgicalTaskStamp(),
      surgicalTaskState(0),
      robotsToolState(),
      allToolsPoseWRTImage(),
      allToolsTwistWRTImage(),
      allToolsWrench()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->surgicalTaskStamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->surgicalTaskStamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->surgicalTaskStamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->surgicalTaskStamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->surgicalTaskStamp.sec);
      *(outbuffer + offset + 0) = (this->surgicalTaskStamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->surgicalTaskStamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->surgicalTaskStamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->surgicalTaskStamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->surgicalTaskStamp.nsec);
      *(outbuffer + offset + 0) = (this->surgicalTaskState >> (8 * 0)) & 0xFF;
      offset += sizeof(this->surgicalTaskState);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->robotsToolState[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robotsToolState[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->allToolsPoseWRTImage[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->allToolsTwistWRTImage[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->allToolsWrench[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->surgicalTaskStamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->surgicalTaskStamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->surgicalTaskStamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->surgicalTaskStamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->surgicalTaskStamp.sec);
      this->surgicalTaskStamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->surgicalTaskStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->surgicalTaskStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->surgicalTaskStamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->surgicalTaskStamp.nsec);
      this->surgicalTaskState =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->surgicalTaskState);
      for( uint32_t i = 0; i < 2; i++){
      this->robotsToolState[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robotsToolState[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->allToolsPoseWRTImage[i].deserialize(inbuffer + offset);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->allToolsTwistWRTImage[i].deserialize(inbuffer + offset);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->allToolsWrench[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "custom_msgs/SurgicalTaskStateMsg"; };
    const char * getMD5(){ return "ca2a5477bc5bf73576fc92c9770df09a"; };

  };

}
#endif