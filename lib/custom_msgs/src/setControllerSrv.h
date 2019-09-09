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
      typedef uint8_t _set_ctrlType_type;
      _set_ctrlType_type set_ctrlType;
      typedef bool _default_ctrl_type;
      _default_ctrl_type default_ctrl;
      float pos_p[5];
      float pos_i[5];
      float pos_d[5];
      float speed_p[5];
      float speed_i[5];
      float speed_d[5];

    setControllerSrvRequest():
      set_ctrlType(0),
      default_ctrl(0),
      pos_p(),
      pos_i(),
      pos_d(),
      speed_p(),
      speed_i(),
      speed_d()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->set_ctrlType >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_ctrlType);
      union {
        bool real;
        uint8_t base;
      } u_default_ctrl;
      u_default_ctrl.real = this->default_ctrl;
      *(outbuffer + offset + 0) = (u_default_ctrl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->default_ctrl);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_pi;
      u_pos_pi.real = this->pos_p[i];
      *(outbuffer + offset + 0) = (u_pos_pi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_pi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_pi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_pi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_p[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_ii;
      u_pos_ii.real = this->pos_i[i];
      *(outbuffer + offset + 0) = (u_pos_ii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_ii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_ii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_ii.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_i[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_di;
      u_pos_di.real = this->pos_d[i];
      *(outbuffer + offset + 0) = (u_pos_di.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_di.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_di.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_di.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_d[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_pi;
      u_speed_pi.real = this->speed_p[i];
      *(outbuffer + offset + 0) = (u_speed_pi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_pi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_pi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_pi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_p[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_ii;
      u_speed_ii.real = this->speed_i[i];
      *(outbuffer + offset + 0) = (u_speed_ii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_ii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_ii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_ii.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_i[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_di;
      u_speed_di.real = this->speed_d[i];
      *(outbuffer + offset + 0) = (u_speed_di.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_di.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_di.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_di.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_d[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->set_ctrlType =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->set_ctrlType);
      union {
        bool real;
        uint8_t base;
      } u_default_ctrl;
      u_default_ctrl.base = 0;
      u_default_ctrl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->default_ctrl = u_default_ctrl.real;
      offset += sizeof(this->default_ctrl);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_pi;
      u_pos_pi.base = 0;
      u_pos_pi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_pi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_pi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_pi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_p[i] = u_pos_pi.real;
      offset += sizeof(this->pos_p[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_ii;
      u_pos_ii.base = 0;
      u_pos_ii.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_ii.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_ii.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_ii.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_i[i] = u_pos_ii.real;
      offset += sizeof(this->pos_i[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_pos_di;
      u_pos_di.base = 0;
      u_pos_di.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_di.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_di.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_di.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_d[i] = u_pos_di.real;
      offset += sizeof(this->pos_d[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_pi;
      u_speed_pi.base = 0;
      u_speed_pi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_pi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_pi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_pi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_p[i] = u_speed_pi.real;
      offset += sizeof(this->speed_p[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_ii;
      u_speed_ii.base = 0;
      u_speed_ii.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_ii.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_ii.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_ii.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_i[i] = u_speed_ii.real;
      offset += sizeof(this->speed_i[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_speed_di;
      u_speed_di.base = 0;
      u_speed_di.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_di.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_di.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_di.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_d[i] = u_speed_di.real;
      offset += sizeof(this->speed_d[i]);
      }
     return offset;
    }

    const char * getType(){ return SETCONTROLLERSRV; };
    const char * getMD5(){ return "cecae5aa64048a33f9054a5f2bfe401e"; };

  };

  class setControllerSrvResponse : public ros::Msg
  {
    public:
      typedef bool _sC_ok_type;
      _sC_ok_type sC_ok;

    setControllerSrvResponse():
      sC_ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_sC_ok;
      u_sC_ok.real = this->sC_ok;
      *(outbuffer + offset + 0) = (u_sC_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sC_ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_sC_ok;
      u_sC_ok.base = 0;
      u_sC_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sC_ok = u_sC_ok.real;
      offset += sizeof(this->sC_ok);
     return offset;
    }

    const char * getType(){ return SETCONTROLLERSRV; };
    const char * getMD5(){ return "fcd0f9e92bed11d2bdaa289a2a08ba11"; };

  };

  class setControllerSrv {
    public:
    typedef setControllerSrvRequest Request;
    typedef setControllerSrvResponse Response;
  };

}
#endif
