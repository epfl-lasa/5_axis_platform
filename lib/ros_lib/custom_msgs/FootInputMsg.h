#ifndef _ROS_custom_msgs_FootInputMsg_h
#define _ROS_custom_msgs_FootInputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class FootInputMsg : public ros::Msg
  {
    public:
      float ros_position[5];
      float ros_speed[5];
      float ros_effort[5];
      float ros_effortM[5];
      float ros_filterAxisForce[5];
      float ros_kp[5];
      float ros_kd[5];
      float ros_ki[5];

    FootInputMsg():
      ros_position(),
      ros_speed(),
      ros_effort(),
      ros_effortM(),
      ros_filterAxisForce(),
      ros_kp(),
      ros_kd(),
      ros_ki()
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
      } u_ros_speedi;
      u_ros_speedi.real = this->ros_speed[i];
      *(outbuffer + offset + 0) = (u_ros_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speed[i]);
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
      } u_ros_effortMi;
      u_ros_effortMi.real = this->ros_effortM[i];
      *(outbuffer + offset + 0) = (u_ros_effortMi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_effortMi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_effortMi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_effortMi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_effortM[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_filterAxisForcei;
      u_ros_filterAxisForcei.real = this->ros_filterAxisForce[i];
      *(outbuffer + offset + 0) = (u_ros_filterAxisForcei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_filterAxisForcei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_filterAxisForcei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_filterAxisForcei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_filterAxisForce[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_kpi;
      u_ros_kpi.real = this->ros_kp[i];
      *(outbuffer + offset + 0) = (u_ros_kpi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_kpi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_kpi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_kpi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_kp[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_kdi;
      u_ros_kdi.real = this->ros_kd[i];
      *(outbuffer + offset + 0) = (u_ros_kdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_kdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_kdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_kdi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_kd[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_kii;
      u_ros_kii.real = this->ros_ki[i];
      *(outbuffer + offset + 0) = (u_ros_kii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_kii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_kii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_kii.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_ki[i]);
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
      } u_ros_speedi;
      u_ros_speedi.base = 0;
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speed[i] = u_ros_speedi.real;
      offset += sizeof(this->ros_speed[i]);
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
      } u_ros_effortMi;
      u_ros_effortMi.base = 0;
      u_ros_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_effortM[i] = u_ros_effortMi.real;
      offset += sizeof(this->ros_effortM[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_filterAxisForcei;
      u_ros_filterAxisForcei.base = 0;
      u_ros_filterAxisForcei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_filterAxisForcei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_filterAxisForcei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_filterAxisForcei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_filterAxisForce[i] = u_ros_filterAxisForcei.real;
      offset += sizeof(this->ros_filterAxisForce[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_kpi;
      u_ros_kpi.base = 0;
      u_ros_kpi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_kpi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_kpi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_kpi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_kp[i] = u_ros_kpi.real;
      offset += sizeof(this->ros_kp[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_kdi;
      u_ros_kdi.base = 0;
      u_ros_kdi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_kdi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_kdi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_kdi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_kd[i] = u_ros_kdi.real;
      offset += sizeof(this->ros_kd[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_kii;
      u_ros_kii.base = 0;
      u_ros_kii.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_kii.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_kii.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_kii.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_ki[i] = u_ros_kii.real;
      offset += sizeof(this->ros_ki[i]);
      }
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootInputMsg"; };
    const char * getMD5(){ return "728ace458772abb6fc46fc86fef9a7b6"; };

  };

}
#endif