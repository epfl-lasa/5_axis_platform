#ifndef MACROS_FORCE_MEASUREMENT_H
#define MACROS_FORCE_MEASUREMENT_H

#define WO_LAST_LINK_CORRECTION 0
#define W_LAST_LINK_CORRECTION 1
#define LAST_LINK_FS W_LAST_LINK_CORRECTION



// THERE IS AN ISSUE IN THESE EXPLICIT EQUATIONS WITH THE MEASUREMENT IN PITCH... 
// SO I DECIDED TO INSTEAD OF SENDING THE 6D WRENCH FROM THE F/T SENSOR, 
// I CONVERT J^T * TAU IN THE FOOT_FORCE_MEAS NODE IN FOOT_PLATFORM_ROS

#if (LAST_LINK_FS)
#define effortM_Y                                                              \
  (_ros_forceSensor[FY] -                                                                        \
   _massLinks(LINK_PEDAL) * GRAVITY * (_s_psi * _s_theta -                       \
                     _c_psi * _c_theta * _s_phi)) *    \
          (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) -         \
      (_ros_forceSensor[FX] +                                                                    \
       _massLinks(LINK_PEDAL) * GRAVITY *                                                        \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi)) *             \
          (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -         \
      _c_phi * _s_theta *                                                  \
          (_ros_forceSensor[FZ] - _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta)

#define effortM_X                                                              \
  _s_phi * (_ros_forceSensor[FZ] - _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) +       \
      _c_phi * _c_psi *                                                    \
          (_ros_forceSensor[FY] -                                                                \
           _massLinks(LINK_PEDAL) * GRAVITY *                                                    \
               (_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi)) +         \
      _c_phi * _s_psi *                                                    \
          (_ros_forceSensor[FX] +                                                                \
           _massLinks(LINK_PEDAL) * GRAVITY *                                                    \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi))

#define effortM_PITCH                                                          \
  (_ros_forceSensor[FX] +                                                                        \
   _massLinks(LINK_PEDAL) * GRAVITY * (_c_psi * _s_theta +                       \
                     _c_theta * _s_phi * _s_psi)) *    \
          (_c_phi * _c_theta * d6 *                      \
               (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) +    \
           _c_phi * _s_theta * d6 *                      \
               (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi)) -   \
      (_ros_forceSensor[FY] -                                                                    \
       _massLinks(LINK_PEDAL) * GRAVITY *                                                        \
           (_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi)) *             \
          (_c_phi * _c_theta * d6 *                      \
               (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) +    \
           _c_phi * _s_theta * d6 *                      \
               (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi)) +   \
      (_c_phi * _c_theta * d6 * _c_phi * _s_theta -  \
       _c_phi * _s_theta * d6 * _c_phi * _c_theta) * \
          (_ros_forceSensor[FZ] - _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) -          \
      _s_phi * (LINKS_COM[LINK_PEDAL][0] * _massLinks(LINK_PEDAL) * GRAVITY *                                   \
                      (_s_psi * _s_theta -                     \
                       _c_psi * _c_theta * _s_phi) -   \
                  _ros_forceSensor[TZ] +                                                         \
                  LINKS_COM[LINK_PEDAL][1] * _massLinks(LINK_PEDAL) * GRAVITY *                                   \
                      (_c_psi * _s_theta +                     \
                       _c_theta * _s_phi * _s_psi)) +  \
      _c_phi * _c_psi *                                                    \
          (_ros_forceSensor[TY] +                                                                \
           LINKS_COM[LINK_PEDAL][2] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) +          \
           LINKS_COM[LINK_PEDAL][0] * _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) +     \
      _c_phi * _s_psi *                                                    \
          (_ros_forceSensor[TX] +                                                                \
           LINKS_COM[LINK_PEDAL][2] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) -          \
           LINKS_COM[LINK_PEDAL][1] * _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta)

#define effortM_ROLL                                                           \
  (_c_theta * _c_phi * _s_theta -                                  \
   _s_theta * _c_phi * _c_theta) *                                 \
          (LINKS_COM[LINK_PEDAL][0] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) -          \
           _ros_forceSensor[TZ] +                                                                \
           LINKS_COM[LINK_PEDAL][1] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi)) -         \
      (_c_theta *                                                      \
           (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) +        \
       _s_theta *                                                      \
           (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi)) *       \
          (_ros_forceSensor[TX] +                                                                \
           LINKS_COM[LINK_PEDAL][2] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) -          \
           LINKS_COM[LINK_PEDAL][1] * _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) +     \
      (_c_theta *                                                      \
           (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) +        \
       _s_theta *                                                      \
           (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi)) *       \
          (_ros_forceSensor[TY] +                                                                \
           LINKS_COM[LINK_PEDAL][2] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) +          \
           LINKS_COM[LINK_PEDAL][0] * _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) -     \
      (_ros_forceSensor[FZ] - _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) *              \
          (_c_theta * _s_phi * d6 * _c_phi *           \
               _c_theta -                                                    \
           _s_phi * (_c_phi * _s_theta *_s_theta                     \
                       * d6 + _c_phi * _c_theta *_c_theta      \
                       * d6) +                                         \
           _s_phi * _s_theta * d6 * _c_phi *           \
               _s_theta) +                                                   \
      (_ros_forceSensor[FY] -                                                                    \
       _massLinks(LINK_PEDAL) * GRAVITY *                                                        \
           (_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi)) *             \
          (_c_phi * _c_psi *                                               \
               (_c_phi * _s_theta * _s_theta                            \
                * d6 + _c_phi * _c_theta *_c_theta             \
                 * d6) -                                                \
           _c_theta * _s_phi * d6 *                      \
               (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) +    \
           _s_phi * _s_theta * d6 *                      \
               (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta)) +   \
      (_ros_forceSensor[FX] +                                                                    \
       _massLinks(LINK_PEDAL) * GRAVITY *                                                        \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi)) *             \
          (_c_phi * _s_psi *                                               \
               (_c_phi * _s_theta *_s_theta                            \
                * d6 + _c_phi * _c_theta *_c_theta             \
                * d6) +                                                \
           _c_theta * _s_phi * d6 *                      \
               (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) -    \
           _s_phi * _s_theta * d6 *                      \
               (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta))

#define effortM_YAW                                                            \
  (_c_phi * _c_theta *                                         \
       (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) +            \
   _s_phi * _c_phi * _c_psi -                                      \
   _c_phi * _s_theta *                                         \
       (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta)) *           \
          (_ros_forceSensor[TY] +                                                                \
           LINKS_COM[LINK_PEDAL][2] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) +          \
           LINKS_COM[LINK_PEDAL][0] * _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) -     \
      (_s_phi * _s_phi +                                             \
       _c_phi * _c_theta * _c_phi * _c_theta +             \
       _c_phi * _s_theta * _c_phi * _s_theta) *            \
          (LINKS_COM[LINK_PEDAL][0] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) -          \
           _ros_forceSensor[TZ] +                                                                \
           LINKS_COM[LINK_PEDAL][1] * _massLinks(LINK_PEDAL) * GRAVITY *                                          \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi)) +         \
      (_ros_forceSensor[TX] +                                                                    \
       LINKS_COM[LINK_PEDAL][2] * _massLinks(LINK_PEDAL) * GRAVITY *                                              \
           (_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) -              \
       LINKS_COM[LINK_PEDAL][1] * _massLinks(LINK_PEDAL) * GRAVITY * _c_phi * _c_theta) *         \
          (_c_phi * _s_theta *                                 \
               (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -    \
           _c_phi * _c_theta *                                 \
               (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +    \
           _s_phi * _c_phi * _s_psi)

#else //LAST_LINK_FS

#define effortM_Y  \
      _ros_forceSensor[FY] * (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) - \
      _ros_forceSensor[FX] * (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) - \
      _ros_forceSensor[FZ] * _c_phi * _s_theta 

#define effortM_X  \
        _ros_forceSensor[FZ] * _s_phi + \
        _ros_forceSensor[FY] * _c_phi * _c_psi + \
        _ros_forceSensor[FX] * _c_phi * _s_psi 

#define effortM_PITCH  \
      _ros_forceSensor[FX] * \
          (_c_phi * _c_theta * d6 * \
               (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) + \
           _c_phi * _s_theta * d6 * \
               (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi)) - \
      _ros_forceSensor[FY] * \
          (_c_phi * _c_theta * d6 * \
               (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) + \
           _c_phi * _s_theta * d6 * \
               (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi)) + \
      _ros_forceSensor[TZ] * _s_phi + \
      _ros_forceSensor[FZ] * (_c_phi * _c_theta * d6 * _c_phi * _s_theta - \
                              _c_phi * _s_theta * d6 * _c_phi * _c_theta) + \
      _ros_forceSensor[TY] * _c_phi * _c_psi + \
      _ros_forceSensor[TX] * _c_phi * _s_psi 

#define effortM_ROLL  \
      _ros_forceSensor[TY] * \
          (_c_theta * (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) + \
           _s_theta * (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi)) - \
      _ros_forceSensor[TX] * \
          (_c_theta * (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) + \
           _s_theta * (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi)) - \
      _ros_forceSensor[FZ] * (_c_theta * _s_phi * d6 * _c_phi * _c_theta - \
                              _s_phi * (_c_phi * _s_theta * _s_theta * d6 + \
                                        _c_phi * _c_theta * _c_theta * d6) + \
                              _s_phi * _s_theta * d6 * _c_phi * _s_theta) - \
      _ros_forceSensor[TZ] * \
          (_c_theta * _c_phi * _s_theta - _s_theta * _c_phi * _c_theta) + \
      _ros_forceSensor[FY] * \
          (_c_phi * _c_psi * (_c_phi * _s_theta * _s_theta * d6 + \
                              _c_phi * _c_theta * _c_theta * d6) - \
           _c_theta * _s_phi * d6 * \
               (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) + \
           _s_phi * _s_theta * d6 * \
               (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta)) + \
      _ros_forceSensor[FX] * \
          (_c_phi * _s_psi * (_c_phi * _s_theta * _s_theta * d6 + \
                              _c_phi * _c_theta * _c_theta * d6) + \
           _c_theta * _s_phi * d6 * \
               (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) - \
           _s_phi * _s_theta * d6 * \
               (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta)) 
 
#define effortM_YAW \
        _ros_forceSensor[TY] * \
                      (_c_phi * _c_theta * \
                           (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) + \
                       _s_phi * _c_phi * _c_psi - \
                       _c_phi * _s_theta * \
                           (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta)) + \
                  _ros_forceSensor[TX] * \
                      (_c_phi * _s_theta * \
                           (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) - \
                       _c_phi * _c_theta * \
                           (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) + \
                       _s_phi * _c_phi * _s_psi) + \
                  _ros_forceSensor[TZ] * \
                      (_s_phi * _s_phi + _c_phi * _c_theta * _c_phi * _c_theta + \
                       _c_phi * _s_theta * _c_phi * _s_theta)
#endif // 

#endif //MACROS_FORCE_MEASUREMENT_H