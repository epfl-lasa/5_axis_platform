#ifndef MACROS_RCM_CONTROL_H
#define MACROS_RCM_CONTROL_H


#define SX_RCM (_platform_posRCM(0) - _position(X))
#define SY_RCM (_platform_posRCM(1) - _position(Y))
#define SZ_RCM (_platform_posRCM(2)-r3)

#define INV_NORM_RCM_ANGLE powf((sx_rcm * sx_rcm + sy_rcm * sy_rcm + sz_rcm * sz_rcm),-0.5f);

#define RCM_COS_DIFF_ANGLE                                                     \
  acosf(inv_norm_rcm * (_s_phi * sx_rcm +                             \
       _c_phi * _c_theta * sz_rcm -        \
       _c_phi * _s_theta * sy_rcm))

#define RCM_EFFORT_Y 0.0f

#define RCM_EFFORT_X 0.0f

#define RCM_EFFORT_PITCH                         \
  inv_norm_rcm * ( _c_phi * _c_theta * sy_rcm +     \
  _c_phi * _s_theta * sz_rcm)

#define RCM_EFFORT_ROLL                                                        \
    _c_theta * (inv_norm_rcm * _s_phi * sz_rcm -      \
                        inv_norm_rcm * _c_phi * _c_theta *       \
                            sx_rcm) -                                  \
        _s_theta * (inv_norm_rcm * _s_phi * sy_rcm +   \
                            inv_norm_rcm * _c_phi * _s_theta *   \
                              sx_rcm)

#define RCM_EFFORT_YAW 0.0f

#endif //MACROS_RCM_CONTROL_H