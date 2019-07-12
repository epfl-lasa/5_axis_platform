#ifndef _ROS_custom_msgs_FootOutputMsg_h
#define _ROS_custom_msgs_FootOutputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace custom_msgs
{

  class FootOutputMsg : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int16_t _id_type;
      _id_type id;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _phi_type;
      _phi_type phi;
      typedef float _theta_type;
      _theta_type theta;
      typedef float _psi_type;
      _psi_type psi;
      typedef float _Fx_type;
      _Fx_type Fx;
      typedef float _Fy_type;
      _Fy_type Fy;
      typedef float _Tphi_type;
      _Tphi_type Tphi;
      typedef float _Ttheta_type;
      _Ttheta_type Ttheta;
      typedef float _Tpsi_type;
      _Tpsi_type Tpsi;
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _wphi_type;
      _wphi_type wphi;
      typedef float _wtheta_type;
      _wtheta_type wtheta;
      typedef float _wpsi_type;
      _wpsi_type wpsi;
      typedef int16_t _state_type;
      _state_type state;

    FootOutputMsg():
      stamp(),
      id(0),
      x(0),
      y(0),
      phi(0),
      theta(0),
      psi(0),
      Fx(0),
      Fy(0),
      Tphi(0),
      Ttheta(0),
      Tpsi(0),
      vx(0),
      vy(0),
      wphi(0),
      wtheta(0),
      wpsi(0),
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
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.real = this->phi;
      *(outbuffer + offset + 0) = (u_phi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phi);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_psi;
      u_psi.real = this->psi;
      *(outbuffer + offset + 0) = (u_psi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_psi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_psi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_psi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->psi);
      union {
        float real;
        uint32_t base;
      } u_Fx;
      u_Fx.real = this->Fx;
      *(outbuffer + offset + 0) = (u_Fx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fx);
      union {
        float real;
        uint32_t base;
      } u_Fy;
      u_Fy.real = this->Fy;
      *(outbuffer + offset + 0) = (u_Fy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fy);
      union {
        float real;
        uint32_t base;
      } u_Tphi;
      u_Tphi.real = this->Tphi;
      *(outbuffer + offset + 0) = (u_Tphi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tphi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tphi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tphi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tphi);
      union {
        float real;
        uint32_t base;
      } u_Ttheta;
      u_Ttheta.real = this->Ttheta;
      *(outbuffer + offset + 0) = (u_Ttheta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ttheta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ttheta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ttheta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ttheta);
      union {
        float real;
        uint32_t base;
      } u_Tpsi;
      u_Tpsi.real = this->Tpsi;
      *(outbuffer + offset + 0) = (u_Tpsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tpsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tpsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tpsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tpsi);
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_wphi;
      u_wphi.real = this->wphi;
      *(outbuffer + offset + 0) = (u_wphi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wphi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wphi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wphi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wphi);
      union {
        float real;
        uint32_t base;
      } u_wtheta;
      u_wtheta.real = this->wtheta;
      *(outbuffer + offset + 0) = (u_wtheta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wtheta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wtheta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wtheta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wtheta);
      union {
        float real;
        uint32_t base;
      } u_wpsi;
      u_wpsi.real = this->wpsi;
      *(outbuffer + offset + 0) = (u_wpsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wpsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wpsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wpsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wpsi);
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
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.base = 0;
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->phi = u_phi.real;
      offset += sizeof(this->phi);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_psi;
      u_psi.base = 0;
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->psi = u_psi.real;
      offset += sizeof(this->psi);
      union {
        float real;
        uint32_t base;
      } u_Fx;
      u_Fx.base = 0;
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fx = u_Fx.real;
      offset += sizeof(this->Fx);
      union {
        float real;
        uint32_t base;
      } u_Fy;
      u_Fy.base = 0;
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fy = u_Fy.real;
      offset += sizeof(this->Fy);
      union {
        float real;
        uint32_t base;
      } u_Tphi;
      u_Tphi.base = 0;
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tphi = u_Tphi.real;
      offset += sizeof(this->Tphi);
      union {
        float real;
        uint32_t base;
      } u_Ttheta;
      u_Ttheta.base = 0;
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Ttheta = u_Ttheta.real;
      offset += sizeof(this->Ttheta);
      union {
        float real;
        uint32_t base;
      } u_Tpsi;
      u_Tpsi.base = 0;
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tpsi = u_Tpsi.real;
      offset += sizeof(this->Tpsi);
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_wphi;
      u_wphi.base = 0;
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wphi = u_wphi.real;
      offset += sizeof(this->wphi);
      union {
        float real;
        uint32_t base;
      } u_wtheta;
      u_wtheta.base = 0;
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wtheta = u_wtheta.real;
      offset += sizeof(this->wtheta);
      union {
        float real;
        uint32_t base;
      } u_wpsi;
      u_wpsi.base = 0;
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wpsi = u_wpsi.real;
      offset += sizeof(this->wpsi);
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

    const char * getType(){ return "custom_msgs/FootOutputMsg"; };
    const char * getMD5(){ return "d8e84508485f8ff66f85ecd7bdcbd77f"; };

  };

}
#endif