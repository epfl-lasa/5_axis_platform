#ifndef MACROS_COMPENSATION_INERTIA_H
#define MACROS_COMPENSATION_INERTIA_H


/************************MACROS FOR EQUATION COMPENSATION*******************************/

#define XX 0
#define XY 1
#define XZ 2
#define YY 4
#define YZ 5
#define ZZ 8

#define COMP_INERTIA_EQ_Y \
    6 * _massLinks(LINK_PEDAL) * _acceleration(Y) +\
    _acceleration(ROLL) *\
        (_massLinks(LINK_PEDAL) * _s_theta *\
             (LINKS_COM[LINK_ROLL][1] * _c_phi +\
              LINKS_COM[LINK_ROLL][0] * _s_phi) +\
         _massLinks(LINK_PEDAL) * _s_theta *\
             (LINKS_COM[LINK_YAW][2] * _s_phi -\
              _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -\
                        LINKS_COM[LINK_YAW][1] * _s_psi)) +\
         _massLinks(LINK_PEDAL) * _s_theta *\
             (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +\
              LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +\
              LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) -\
    _acceleration(YAW) *\
        (_massLinks(LINK_PEDAL) *\
             (_s_phi * (LINKS_COM[LINK_PEDAL][1] *\
                            (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -\
                        LINKS_COM[LINK_PEDAL][0] *\
                            (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +\
                        LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +\
                        d6 * _c_phi * _c_theta) -\
              _c_phi * _c_theta *\
                  (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +\
                   LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +\
                   LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +\
         _massLinks(LINK_PEDAL) *\
             (_s_phi * (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -\
                        _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +\
                                    LINKS_COM[LINK_YAW][0] * _s_psi) +\
                        _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -\
                                             LINKS_COM[LINK_YAW][1] * _s_psi)) -\
              _c_phi * _c_theta *\
                  (LINKS_COM[LINK_YAW][2] * _s_phi -\
                   _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -\
                             LINKS_COM[LINK_YAW][1] * _s_psi)))) -\
    _acceleration(PITCH) *\
        (_massLinks(LINK_PEDAL) *\
             (LINKS_COM[LINK_PEDAL][1] *\
                  (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -\
              LINKS_COM[LINK_PEDAL][0] *\
                  (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +\
              LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +\
              d6 * _c_phi * _c_theta) +\
         _massLinks(LINK_PEDAL) * (LINKS_COM[LINK_PITCH][0] * _c_theta -\
                                   LINKS_COM[LINK_PITCH][1] * _s_theta) +\
         _massLinks(LINK_PEDAL) *\
             (LINKS_COM[LINK_ROLL][2] * _s_theta +\
              _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -\
                          LINKS_COM[LINK_ROLL][1] * _s_phi)) +\
         _massLinks(LINK_PEDAL) *\
             (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -\
              _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +\
                          LINKS_COM[LINK_YAW][0] * _s_psi) +\
              _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -\
                                   LINKS_COM[LINK_YAW][1] * _s_psi)))

/*==============================================================================================================================================================*/


#define COMP_INERTIA_EQ_X                                                      \
  5 * _massLinks(LINK_PEDAL) *_acceleration(X) -                                                      \
      _acceleration(YAW) *(_massLinks(LINK_PEDAL) *                                                   \
                    (_c_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) -       \
                          LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                    \
                          d6 * _c_phi * _c_theta) -                        \
                     _c_phi * _c_theta *                                   \
                         (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                    \
                          d6 * _c_phi * _s_theta)) -                       \
                _massLinks(LINK_PEDAL) *                                                   \
                    (_c_phi * _c_theta *                                   \
                         (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) + \
                          LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                      \
                          _s_phi * _s_theta *                              \
                              (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -         \
                     _c_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                      \
                          _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) + \
                          _c_theta * _s_phi *                              \
                              (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) +       \
      _acceleration(ROLL) *(                                                              \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_theta *                                                    \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +               \
               _s_theta *                                                    \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +              \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_theta * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) -    \
                             LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                 \
                             d6 * _c_phi * _c_theta) +                     \
               _s_theta * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                             LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                 \
                             d6 * _c_phi * _s_theta)) +                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_theta *                                                    \
                   (LINKS_COM[LINK_ROLL][2] * _s_theta +                                      \
                    _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -    \
               _s_theta *                                                    \
                   (LINKS_COM[LINK_ROLL][2] * _c_theta -                                      \
                    _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi))))

/*=========================================================================================================================================================================================*/


#define COMP_INERTIA_EQ_PITCH \
_acceleration(PITCH)                                       \
  *(LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  + _c_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _s_phi) + \
    _s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _s_phi) +             \
    _s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _c_phi * _c_psi + \
                      LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi) +                       \
    _s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +     \
                      LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi) +                         \
    _massLinks(LINK_PEDAL) * (LINKS_COM[LINK_PITCH][1] * _c_theta + LINKS_COM[LINK_PITCH][0] * _s_theta) *   \
        (LINKS_COM[LINK_PITCH][1] * _c_theta + LINKS_COM[LINK_PITCH][0] * _s_theta) +                        \
    _massLinks(LINK_PEDAL) * (LINKS_COM[LINK_PITCH][0] * _c_theta - LINKS_COM[LINK_PITCH][1] * _s_theta) *   \
        (LINKS_COM[LINK_PITCH][0] * _c_theta - LINKS_COM[LINK_PITCH][1] * _s_theta) +                        \
    _massLinks(LINK_PEDAL) *                                                               \
        (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                        \
                    _c_psi * _c_theta * _s_phi) -      \
         LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                        \
                    _c_theta * _s_phi * _s_psi) +      \
         LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                         \
         _c_phi * _c_theta * d6) *                       \
        (LINKS_COM[LINK_PEDAL][1] *                                                             \
             (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -      \
         LINKS_COM[LINK_PEDAL][0] *                                                             \
             (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +      \
         LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta + d6 * _c_phi * _c_theta) +       \
    _c_phi * _c_psi *                                          \
        (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +              \
         LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                                    \
    _c_phi * _s_psi *                                          \
        (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +              \
         LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) +                                    \
    _massLinks(LINK_PEDAL) * (_c_theta *                                           \
                      (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) +    \
                  LINKS_COM[LINK_ROLL][2] * _s_theta) *                                 \
        (LINKS_COM[LINK_ROLL][2] * _s_theta +                                                 \
         _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -               \
    _massLinks(LINK_PEDAL) * (_s_theta *                                           \
                      (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) -    \
                  LINKS_COM[LINK_ROLL][2] * _c_theta) *                                 \
        (LINKS_COM[LINK_ROLL][2] * _c_theta -                                                 \
         _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -               \
    _c_phi * _c_psi *                                          \
        (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +                  \
         LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                                      \
    _massLinks(LINK_PEDAL) *                                                               \
        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                                       \
         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +                  \
         _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) *      \
        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
         _s_theta *                                                    \
             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +               \
         _c_theta * _s_phi *                                   \
             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +              \
    _c_phi * _s_psi *                                          \
        (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +                  \
         LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) +                                      \
    _massLinks(LINK_PEDAL) *                                                               \
        (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                        \
                    _s_phi * _s_psi * _s_theta) -      \
         LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                        \
                    _c_psi * _s_phi * _s_theta) +      \
         LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                         \
         _c_phi * _s_theta * d6) *                       \
        (LINKS_COM[LINK_PEDAL][0] *                                                             \
             (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -      \
         LINKS_COM[LINK_PEDAL][1] *                                                             \
             (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) +      \
         LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta + d6 * _c_phi * _s_theta) +       \
    _massLinks(LINK_PEDAL) *                                                               \
        (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +                  \
         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                                       \
         _s_phi * _s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) *      \
        (_c_theta *                                                    \
             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +               \
         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
         _s_phi * _s_theta *                                   \
             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) -             \
      _acceleration(Y) *(_massLinks(LINK_PEDAL) *                                                     \
                  (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                 \
                   _s_theta *                                          \
                       (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +     \
                   _c_theta * _s_phi *                         \
                       (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +    \
              _massLinks(LINK_PEDAL) * (_c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -      \
                                                LINKS_COM[LINK_ROLL][1] * _s_phi) +     \
                            LINKS_COM[LINK_ROLL][2] * _s_theta) +                       \
              _massLinks(LINK_PEDAL) *                                                     \
                  (LINKS_COM[LINK_PITCH][0] * _c_theta - LINKS_COM[LINK_PITCH][1] * _s_theta) +  \
              _massLinks(LINK_PEDAL) * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -     \
                                       _c_psi * _c_theta *     \
                                           _s_phi) -                   \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +     \
                                       _c_theta * _s_phi *     \
                                           _s_psi) +                   \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +      \
                            _c_phi * _c_theta * d6)) -   \
      _acceleration(ROLL) *(                                                              \
          _c_theta *                                                         \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) -                         \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                         \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) -                        \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) -                         \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) +                         \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) +                        \
          _c_theta *                                                         \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                           \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) +                           \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi)) +                          \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                           \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) -                           \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi)) -                          \
          _c_theta * (_c_theta *                                     \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * _s_phi) -      \
                        _c_phi * _s_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _s_phi) +      \
                        _s_phi * _s_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _s_phi)) -     \
          _s_theta * (_s_theta *                                     \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * _s_phi) +      \
                        _c_phi * _c_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _s_phi) -      \
                        _c_theta * _s_phi *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _s_phi)) -     \
          _massLinks(LINK_PEDAL) * _c_theta *                                            \
              (LINKS_COM[LINK_YAW][2] * _s_phi -                                              \
               _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) *             \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                     \
               _s_phi * _s_theta *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
          _massLinks(LINK_PEDAL) * _s_theta *                                            \
              (LINKS_COM[LINK_YAW][2] * _s_phi -                                              \
               _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) *             \
              (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                     \
               _s_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               _c_theta * _s_phi *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -        \
          _massLinks(LINK_PEDAL) * _c_theta *                                            \
              (_s_theta *                                              \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) -       \
               LINKS_COM[LINK_ROLL][2] * _c_theta) *                                    \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +                        \
          _massLinks(LINK_PEDAL) * _s_theta *                                            \
              (LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_s_psi * _s_theta -                        \
                    _c_psi * _c_theta * _s_phi) -      \
               LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _s_theta +                        \
                    _c_theta * _s_phi * _s_psi) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                   \
               _c_phi * _c_theta * d6) *                 \
              (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                            \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                                 \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                                \
          _massLinks(LINK_PEDAL) * _s_theta *                                            \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) +       \
               LINKS_COM[LINK_ROLL][2] * _s_theta) *                                    \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) -                        \
          _massLinks(LINK_PEDAL) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +      \
                                     LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +           \
                                     LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) *          \
              (LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _c_theta -                        \
                    _s_phi * _s_psi * _s_theta) -      \
               LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_c_theta * _s_psi +                        \
                    _c_psi * _s_phi * _s_theta) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                   \
               _c_phi * _s_theta * d6)) +                \
      _acceleration(YAW) *(                                                               \
          _s_phi *                                                           \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi) +                         \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                         \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi)) +                        \
          _s_phi *                                                           \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi) -                           \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                           \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi)) +                          \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                      \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _c_phi * _c_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *       \
              (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                     \
               _s_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               _c_theta * _s_phi *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
          _c_phi * _s_theta *                                              \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) -                         \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                         \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) +                        \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                      \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _c_phi * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *       \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                     \
               _s_phi * _s_theta *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -        \
          _c_phi * _c_theta *                                              \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                           \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) -                           \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi)) +                          \
          _c_phi * _s_theta *                                              \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                           \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) +                           \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi)) +                          \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) -      \
                           LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                           LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                   \
                           d6 * _c_phi * _c_theta) -                       \
               _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +        \
                                        LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *      \
              (LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_s_psi * _s_theta -                        \
                    _c_psi * _c_theta * _s_phi) -      \
               LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _s_theta +                        \
                    _c_theta * _s_phi * _s_psi) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                   \
               _c_phi * _c_theta * d6) +                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                           LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                   \
                           d6 * _c_phi * _s_theta) -                       \
               _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +        \
                                        LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *      \
              (LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _c_theta -                        \
                    _s_phi * _s_psi * _s_theta) -      \
               LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_c_theta * _s_psi +                        \
                    _c_psi * _s_phi * _s_theta) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                   \
               _c_phi * _s_theta * d6) +                 \
          _c_phi * _c_theta *                                              \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) -                         \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) +                         \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)))

/*============================================================================================================================================================*/

#define COMP_INERTIA_EQ_ROLL                                                   \
  _acceleration(Y) *(_massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) -            \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi)) -                                      \
      _acceleration(PITCH) *(                                                             \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] *                                                     \
                   (_c_theta * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) +    \
                    _s_theta * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) -   \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] *                                                     \
                   (_c_theta * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                    _s_theta * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_c_theta * _c_phi * _s_theta -         \
                            _s_theta * _c_phi * _c_theta)) +       \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] *                                                       \
                   (_c_theta * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                    _s_theta * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] *                                                       \
                   (_c_theta * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) +    \
                    _s_theta * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_c_theta * _c_phi * _s_theta -           \
                          _s_theta * _c_phi * _c_theta)) -         \
          _c_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ] * (_c_theta * _c_theta +                     \
                           _s_theta * _s_theta) -                    \
               LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY] * (_c_theta * _c_phi * _s_theta -          \
                           _s_theta * _c_phi * _c_theta) +         \
               LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY] * (_c_theta * _s_phi * _s_theta -          \
                           _s_theta * _c_theta * _s_phi)) -        \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ] * (_c_theta * _c_theta +                     \
                           _s_theta * _s_theta) -                    \
               LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX] * (_c_theta * _c_phi * _s_theta -          \
                           _s_theta * _c_phi * _c_theta) +         \
               LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY] * (_c_theta * _s_phi * _s_theta -          \
                           _s_theta * _c_theta * _s_phi)) +        \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] *                                                     \
                   (_c_theta * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) +    \
                    _s_theta * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) -   \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] *                                                     \
                   (_c_theta * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                    _s_theta * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta * _c_phi * _s_theta -         \
                            _s_theta * _c_phi * _c_theta)) +       \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] *                                                     \
                   (_c_theta * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) +    \
                    _s_theta * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) -   \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] *                                                     \
                   (_c_theta * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                    _s_theta * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta * _c_phi * _s_theta -         \
                            _s_theta * _c_phi * _c_theta)) -       \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] *                                                       \
                   (_c_theta * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                    _s_theta * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] *                                                       \
                   (_c_theta * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) +    \
                    _s_theta * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta * _c_phi * _s_theta -           \
                          _s_theta * _c_phi * _c_theta)) +         \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] *                                                       \
                   (_c_theta * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                    _s_theta * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] *                                                       \
                   (_c_theta * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) +    \
                    _s_theta * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) +   \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta * _c_phi * _s_theta -           \
                          _s_theta * _c_phi * _c_theta)) +         \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +            \
               LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                                 \
               _s_phi * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                                 \
               _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +            \
               _c_theta * _s_phi *                                         \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) *                    \
              (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                              \
                          _c_psi * _c_theta * _s_phi) -                  \
               LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                              \
                          _c_theta * _s_phi * _s_psi) +                  \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta + d6 * _c_phi * _c_theta) - \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) *                    \
              (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                              \
                          _s_phi * _s_psi * _s_theta) -                  \
               LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                              \
                          _c_psi * _s_phi * _s_theta) +                  \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta + d6 * _c_phi * _s_theta) + \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) *            \
              (LINKS_COM[LINK_ROLL][2] * _c_theta -                                           \
               _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) +         \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) *            \
              (LINKS_COM[LINK_ROLL][2] * _s_theta +                                           \
               _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi))) -        \
      _acceleration(YAW) *(                                                               \
          _s_phi *                                                           \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) -    \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _s_phi *                                                           \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) - \
          _c_phi * _c_theta *                                              \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) -    \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _c_phi * _s_theta *                                              \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_theta *                                              \
                   (LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) - \
                    LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                    _c_phi * _s_theta * d6) +            \
               _c_theta *                                              \
                   (LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) - \
                    LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _s_theta +                   \
                         _c_theta * _s_phi * _s_psi) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                    _c_phi * _c_theta * d6)) *           \
              (_c_phi * _s_theta *                                         \
                   (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                         \
                               _c_psi * _c_theta * _s_phi) -             \
                    LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                         \
                               _c_theta * _s_phi * _s_psi) +             \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                          \
                    d6 * _c_phi * _c_theta) -                              \
               _c_phi * _c_theta *                                         \
                   (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                         \
                               _s_phi * _s_psi * _s_theta) -             \
                    LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                         \
                               _c_psi * _s_phi * _s_theta) +             \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                          \
                    d6 * _c_phi * _s_theta)) +                             \
          _c_phi * _c_theta *                                              \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) - \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_phi * _c_theta *                                         \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _c_phi * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *              \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _s_theta *                                              \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi * _s_theta *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +  \
          _c_phi * _s_theta *                                              \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) + \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) -      \
                           LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                           LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                   \
                           d6 * _c_phi * _c_theta) -                       \
               _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +        \
                                        LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *      \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -                    \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                           LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                   \
                           d6 * _c_phi * _s_theta) -                       \
               _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +        \
                                        LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *      \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_s_phi *                                                      \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _c_phi * _c_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +       \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_s_phi *                                                      \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _c_phi * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) +      \
      _acceleration(X) *(                                                                 \
          _massLinks(LINK_PEDAL) * (_s_theta *                                     \
                            (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -    \
                                        _s_phi * _s_psi *      \
                                            _s_theta) -                \
                             LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +    \
                                        _c_psi * _s_phi *      \
                                            _s_theta) +                \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +     \
                             _c_phi * _s_theta * d6) +   \
                        _c_theta *                                     \
                            (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -    \
                                        _c_psi * _c_theta *    \
                                            _s_phi) -                  \
                             LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +    \
                                        _c_theta * _s_phi *    \
                                            _s_psi) +                  \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +     \
                             _c_phi * _c_theta * d6)) +  \
          _massLinks(LINK_PEDAL) * (_c_theta *                                     \
                            (_c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -     \
                                                 LINKS_COM[LINK_ROLL][1] * _s_phi) +    \
                             LINKS_COM[LINK_ROLL][2] * _s_theta) +                      \
                        _s_theta *                                     \
                            (_s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -     \
                                                 LINKS_COM[LINK_ROLL][1] * _s_phi) -    \
                             LINKS_COM[LINK_ROLL][2] * _c_theta)) +                     \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _s_theta *                                              \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi * _s_theta *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) + \
      _acceleration(ROLL) *(                                                              \
          _c_theta *                                                         \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) - \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) + \
          _c_theta *                                                         \
              (_c_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][ZZ] * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ] * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ] * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) +   \
               _s_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ] * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY] * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY] * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) -   \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ] * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX] * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY] * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi))) +  \
          _s_theta *                                                         \
              (_s_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][ZZ] * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ] * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ] * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) +   \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ] * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX] * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY] * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) -   \
               _c_theta * _s_phi *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ] * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY] * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY] * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi))) +  \
          _c_theta *                                                         \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) -    \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_theta *                                              \
                   (LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) - \
                    LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                    _c_phi * _s_theta * d6) +            \
               _c_theta *                                              \
                   (LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) - \
                    LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _s_theta +                   \
                         _c_theta * _s_phi * _s_psi) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                    _c_phi * _c_theta * d6)) *           \
              (_c_theta * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) -    \
                             LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                 \
                             d6 * _c_phi * _c_theta) +                     \
               _s_theta * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                             LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                 \
                             d6 * _c_phi * _s_theta)) +                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _s_theta *                                              \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi * _s_theta *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *  \
              (_c_theta *                                                    \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +               \
               _s_theta *                                                    \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +              \
          _massLinks(LINK_PEDAL) * (_c_theta *                                     \
                            (_c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -     \
                                                 LINKS_COM[LINK_ROLL][1] * _s_phi) +    \
                             LINKS_COM[LINK_ROLL][2] * _s_theta) +                      \
                        _s_theta *                                     \
                            (_s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -     \
                                                 LINKS_COM[LINK_ROLL][1] * _s_phi) -    \
                             LINKS_COM[LINK_ROLL][2] * _c_theta)) *                     \
              (_c_theta *                                                    \
                   (LINKS_COM[LINK_ROLL][2] * _s_theta +                                      \
                    _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -    \
               _s_theta *                                                    \
                   (LINKS_COM[LINK_ROLL][2] * _c_theta -                                      \
                    _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi))) +   \
          _massLinks(LINK_PEDAL) * _c_theta * _c_theta *                         \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) *                    \
              (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                            \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                                 \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                                \
          _massLinks(LINK_PEDAL) * _s_theta * _s_theta *                         \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) *                    \
              (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                            \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                                 \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                                \
          _massLinks(LINK_PEDAL) * _c_theta * _c_theta *                         \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) *            \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +                        \
          _massLinks(LINK_PEDAL) * _s_theta * _s_theta *                         \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) *            \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) -                        \
          _massLinks(LINK_PEDAL) * _c_theta * _c_theta *                         \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (LINKS_COM[LINK_YAW][2] * _s_phi -                                              \
               _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -             \
          _massLinks(LINK_PEDAL) * _s_theta * _s_theta *                         \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (LINKS_COM[LINK_YAW][2] * _s_phi -                                              \
               _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))
/*========================================================================================================================================================================*/

#define COMP_INERTIA_EQ_YAW                                                    \
  _acceleration(YAW) *(                                                                   \
      _s_phi * (_s_phi *                                             \
                      (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_phi * _c_theta *        \
                                        (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                                    _s_phi * _c_phi * _c_psi -     \
                                    _c_phi * _s_theta *        \
                                        (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta)) +  \
                       LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_phi * _s_theta *        \
                                        (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                                    _c_phi * _c_theta *        \
                                        (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                                    _s_phi * _c_phi * _s_psi) +    \
                       LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_s_phi * _s_phi +                \
                                    _c_phi * _c_theta *        \
                                        _c_phi * _c_theta +                \
                                    _c_phi * _s_theta *        \
                                        _c_phi * _s_theta)) +              \
                  _c_phi * _s_psi *                            \
                      (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _c_theta *        \
                                        (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                                    _s_phi * _c_phi * _c_psi -     \
                                    _c_phi * _s_theta *        \
                                        (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta)) +  \
                       LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_phi * _s_theta *        \
                                        (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                                    _c_phi * _c_theta *        \
                                        (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                                    _s_phi * _c_phi * _s_psi) +    \
                       LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_s_phi * _s_phi +                \
                                    _c_phi * _c_theta *        \
                                        _c_phi * _c_theta +                \
                                    _c_phi * _s_theta *        \
                                        _c_phi * _s_theta)) +              \
                  _c_phi * _c_psi *                            \
                      (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_phi * _c_theta *        \
                                        (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                                    _s_phi * _c_phi * _c_psi -     \
                                    _c_phi * _s_theta *        \
                                        (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta)) +  \
                       LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _s_theta *        \
                                        (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                                    _c_phi * _c_theta *        \
                                        (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                                    _s_phi * _c_phi * _s_psi) +    \
                       LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_s_phi * _s_phi +                \
                                    _c_phi * _c_theta *        \
                                        _c_phi * _c_theta +                \
                                    _c_phi * _s_theta *        \
                                        _c_phi * _s_theta))) +             \
      _s_phi * (_s_phi *                                             \
                      (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_phi * _s_theta *          \
                                      (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                                  _c_phi * _c_theta *          \
                                      (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                                  _s_phi * _c_phi * _s_psi) -      \
                       LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_phi * _c_theta *          \
                                      (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                                  _s_phi * _c_phi * _c_psi -       \
                                  _c_phi * _s_theta *          \
                                      (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta)) +    \
                       LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_s_phi * _s_phi +                  \
                                  _c_phi * _c_theta *          \
                                      _c_phi * _c_theta +                  \
                                  _c_phi * _s_theta *          \
                                      _c_phi * _s_theta)) -                \
                  _c_phi * _c_psi *                            \
                      (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _s_theta *          \
                                      (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                                  _c_phi * _c_theta *          \
                                      (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                                  _s_phi * _c_phi * _s_psi) -      \
                       LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_phi * _c_theta *          \
                                      (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                                  _s_phi * _c_phi * _c_psi -       \
                                  _c_phi * _s_theta *          \
                                      (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta)) +    \
                       LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_s_phi * _s_phi +                  \
                                  _c_phi * _c_theta *          \
                                      _c_phi * _c_theta +                  \
                                  _c_phi * _s_theta *          \
                                      _c_phi * _s_theta)) +                \
                  _c_phi * _s_psi *                            \
                      (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_phi * _s_theta *          \
                                      (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                                  _c_phi * _c_theta *          \
                                      (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                                  _s_phi * _c_phi * _s_psi) -      \
                       LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _c_theta *          \
                                      (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                                  _s_phi * _c_phi * _c_psi -       \
                                  _c_phi * _s_theta *          \
                                      (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta)) +    \
                       LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_s_phi * _s_phi +                  \
                                  _c_phi * _c_theta *          \
                                      _c_phi * _c_theta +                  \
                                  _c_phi * _s_theta *          \
                                      _c_phi * _s_theta))) +               \
      _massLinks(LINK_PEDAL) *                                                             \
          (_s_phi * (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                         \
                       _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                       _c_theta * _s_phi *                                 \
                           (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -            \
           _c_phi * _c_theta *                                             \
               (LINKS_COM[LINK_YAW][2] * _s_phi -                                             \
                _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *           \
          (_s_phi *                                                    \
               (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                    \
                _s_theta *                                             \
                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +        \
                _c_theta * _s_phi *                            \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
           _c_phi * _c_theta *                                 \
               (_c_phi *                                               \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -        \
                LINKS_COM[LINK_YAW][2] * _s_phi)) +                                     \
      _massLinks(LINK_PEDAL) * (_c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) -        \
                         LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                     \
                         d6 * _c_phi * _c_theta) -                         \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                     \
                         d6 * _c_phi * _s_theta)) *                        \
          (_c_phi * _s_theta *                                 \
               (LINKS_COM[LINK_PEDAL][1] *                                                      \
                    (_s_psi * _s_theta -                       \
                     _c_psi * _c_theta * _s_phi) -     \
                LINKS_COM[LINK_PEDAL][0] *                                                      \
                    (_c_psi * _s_theta +                       \
                     _c_theta * _s_phi * _s_psi) +     \
                LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                _c_phi * _c_theta * d6) -                \
           _c_phi * _c_theta *                                 \
               (LINKS_COM[LINK_PEDAL][0] *                                                      \
                    (_c_psi * _c_theta -                       \
                     _s_phi * _s_psi * _s_theta) -     \
                LINKS_COM[LINK_PEDAL][1] *                                                      \
                    (_c_theta * _s_psi +                       \
                     _c_psi * _s_phi * _s_theta) +     \
                LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                _c_phi * _s_theta * d6)) +               \
      _massLinks(LINK_PEDAL) * (_c_phi * _c_theta *                                    \
                        (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                       \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                       \
                         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *         \
          (_c_phi * _c_theta *                                 \
               (_c_theta *                                             \
                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +        \
                LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                    \
                _s_phi * _s_theta *                            \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -       \
           _c_phi * _s_theta *                                 \
               (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                    \
                _s_theta *                                             \
                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +        \
                _c_theta * _s_phi *                            \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +      \
      _c_phi * _c_theta *                                                  \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) -                                \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) +                                \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta))) +                               \
      _massLinks(LINK_PEDAL) *                                                             \
          (_s_phi * (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                       LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                         \
                       _s_phi * _s_theta *                                 \
                           (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -            \
           _c_phi * _s_theta *                                             \
               (LINKS_COM[LINK_YAW][2] * _s_phi -                                             \
                _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *           \
          (_s_phi *                                                    \
               (_c_theta *                                             \
                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +        \
                LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                    \
                _s_phi * _s_theta *                            \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
           _c_phi * _s_theta *                                 \
               (_c_phi *                                               \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -        \
                LINKS_COM[LINK_YAW][2] * _s_phi)) +                                     \
      _massLinks(LINK_PEDAL) *                                                             \
          (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -    \
                                        _c_psi * _c_theta *    \
                                            _s_phi) -                  \
                             LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +    \
                                        _c_theta * _s_phi *    \
                                            _s_psi) +                  \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +     \
                             _c_phi * _c_theta * d6) -   \
           _c_phi * _c_theta *                                 \
               (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +         \
                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                    \
                LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *                  \
          (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) -          \
                       LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                       LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                       \
                       d6 * _c_phi * _c_theta) -                           \
           _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +       \
                                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +            \
                                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +          \
      _c_phi * _s_theta *                                                  \
          ((_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) -                                \
           (_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) +                                \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta))) -                               \
      _c_phi * _c_theta *                                                  \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) -                                  \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta))) +                                 \
      _c_phi * _s_theta *                                                  \
          ((_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           (_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta))) +                                 \
      _massLinks(LINK_PEDAL) *                                                             \
          (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -    \
                                        _s_phi * _s_psi *      \
                                            _s_theta) -                \
                             LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +    \
                                        _c_psi * _s_phi *      \
                                            _s_theta) +                \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +     \
                             _c_phi * _s_theta * d6) -   \
           _c_phi * _s_theta *                                 \
               (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +         \
                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                    \
                LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *                  \
          (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                       LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta) +          \
                       LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                       \
                       d6 * _c_phi * _s_theta) -                           \
           _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +       \
                                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +            \
                                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))) -         \
      _acceleration(Y) *(                                                                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _c_phi * _c_theta *                             \
                   (_c_phi *                                           \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -    \
                    LINKS_COM[LINK_YAW][2] * _s_phi)) +                                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) - \
                    LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                               _c_theta * _s_phi *             \
                                   _s_psi) +                           \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                    _c_phi * _c_theta * d6) -            \
               _c_phi * _c_theta *                             \
                   (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +     \
                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                \
                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))) +             \
      _acceleration(X) *(                                                                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_phi * _c_theta *                             \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi *                                           \
                        _s_theta *                                     \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -   \
               _c_phi *                                                \
                   _s_theta *                                          \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) -  \
          _massLinks(LINK_PEDAL) * (_c_phi * _s_theta *                    \
                            (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -    \
                                        _c_psi * _c_theta *    \
                                            _s_phi) -                  \
                             LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +    \
                                        _c_theta * _s_phi *    \
                                            _s_psi) +                  \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +     \
                             _c_phi * _c_theta * d6) -   \
                        _c_phi * _c_theta *                    \
                            (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -    \
                                        _s_phi * _s_psi *      \
                                            _s_theta) -                \
                             LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +    \
                                        _c_psi * _s_phi *      \
                                            _s_theta) +                \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +     \
                             _c_phi * _s_theta * d6))) + \
      _acceleration(PITCH) *(                                                             \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_phi * _c_theta *                \
                                (_s_psi * _s_theta -                       \
                                 _c_psi * _c_theta * _s_phi) +           \
                            _s_phi * _c_phi * _c_psi -             \
                            _c_phi * _s_theta *                \
                                (_c_theta * _s_psi +                       \
                                 _c_psi * _s_phi * _s_theta)) +          \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_phi * _s_theta *                \
                                (_c_psi * _c_theta -                       \
                                 _s_phi * _s_psi * _s_theta) -           \
                            _c_phi * _c_theta *                \
                                (_c_psi * _s_theta +                       \
                                 _c_theta * _s_phi * _s_psi) +           \
                            _s_phi * _c_phi * _s_psi) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_s_phi * _s_phi +                        \
                            _c_phi * _c_theta * _c_phi *     \
                                _c_theta +                                   \
                            _c_phi * _s_theta * _c_phi *     \
                                _s_theta)) +                                 \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_phi * _s_theta *                  \
                              (_c_psi * _c_theta -                         \
                               _s_phi * _s_psi * _s_theta) -             \
                          _c_phi * _c_theta *                  \
                              (_c_psi * _s_theta +                         \
                               _c_theta * _s_phi * _s_psi) +             \
                          _s_phi * _c_phi * _s_psi) -              \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_phi * _c_theta *                  \
                              (_s_psi * _s_theta -                         \
                               _c_psi * _c_theta * _s_phi) +             \
                          _s_phi * _c_phi * _c_psi -               \
                          _c_phi * _s_theta *                  \
                              (_c_theta * _s_psi +                         \
                               _c_psi * _s_phi * _s_theta)) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_s_phi * _s_phi +                          \
                          _c_phi * _c_theta * _c_phi *       \
                              _c_theta +                                     \
                          _c_phi * _s_theta * _c_phi *       \
                              _s_theta)) +                                   \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _c_theta *                \
                                (_s_psi * _s_theta -                       \
                                 _c_psi * _c_theta * _s_phi) +           \
                            _s_phi * _c_phi * _c_psi -             \
                            _c_phi * _s_theta *                \
                                (_c_theta * _s_psi +                       \
                                 _c_psi * _s_phi * _s_theta)) +          \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_phi * _s_theta *                \
                                (_c_psi * _c_theta -                       \
                                 _s_phi * _s_psi * _s_theta) -           \
                            _c_phi * _c_theta *                \
                                (_c_psi * _s_theta +                       \
                                 _c_theta * _s_phi * _s_psi) +           \
                            _s_phi * _c_phi * _s_psi) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_s_phi * _s_phi +                        \
                            _c_phi * _c_theta * _c_phi *     \
                                _c_theta +                                   \
                            _c_phi * _s_theta * _c_phi *     \
                                _s_theta)) -                                 \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _s_theta *                  \
                              (_c_psi * _c_theta -                         \
                               _s_phi * _s_psi * _s_theta) -             \
                          _c_phi * _c_theta *                  \
                              (_c_psi * _s_theta +                         \
                               _c_theta * _s_phi * _s_psi) +             \
                          _s_phi * _c_phi * _s_psi) -              \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_phi * _c_theta *                  \
                              (_s_psi * _s_theta -                         \
                               _c_psi * _c_theta * _s_phi) +             \
                          _s_phi * _c_phi * _c_psi -               \
                          _c_phi * _s_theta *                  \
                              (_c_theta * _s_psi +                         \
                               _c_psi * _s_phi * _s_theta)) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_s_phi * _s_phi +                          \
                          _c_phi * _c_theta * _c_phi *       \
                              _c_theta +                                     \
                          _c_phi * _s_theta * _c_phi *       \
                              _s_theta)) +                                   \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_phi * _s_theta *                  \
                              (_c_psi * _c_theta -                         \
                               _s_phi * _s_psi * _s_theta) -             \
                          _c_phi * _c_theta *                  \
                              (_c_psi * _s_theta +                         \
                               _c_theta * _s_phi * _s_psi) +             \
                          _s_phi * _c_phi * _s_psi) -              \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _c_theta *                  \
                              (_s_psi * _s_theta -                         \
                               _c_psi * _c_theta * _s_phi) +             \
                          _s_phi * _c_phi * _c_psi -               \
                          _c_phi * _s_theta *                  \
                              (_c_theta * _s_psi +                         \
                               _c_psi * _s_phi * _s_theta)) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_s_phi * _s_phi +                          \
                          _c_phi * _c_theta * _c_phi *       \
                              _c_theta +                                     \
                          _c_phi * _s_theta * _c_phi *       \
                              _s_theta)) +                                   \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) - \
                    LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                               _c_theta * _s_phi *             \
                                   _s_psi) +                           \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                    _c_phi * _c_theta * d6) -            \
               _c_phi * _c_theta *                             \
                   (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +     \
                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                \
                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *              \
              (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                              \
                          _c_psi * _c_theta * _s_phi) -                  \
               LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                              \
                          _c_theta * _s_phi * _s_psi) +                  \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta + d6 * _c_phi * _c_theta) + \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) - \
                    LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                    _c_phi * _s_theta * d6) -            \
               _c_phi * _s_theta *                             \
                   (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +     \
                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                \
                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *              \
              (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                              \
                          _s_phi * _s_psi * _s_theta) -                  \
               LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                              \
                          _c_psi * _s_phi * _s_theta) +                  \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta + d6 * _c_phi * _s_theta) + \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _c_phi * _c_theta *                             \
                   (_c_phi *                                           \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -    \
                    LINKS_COM[LINK_YAW][2] * _s_phi)) *                                 \
              (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                                 \
               _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +            \
               _c_theta * _s_phi *                                         \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_s_phi *                                                \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi * _s_theta *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _c_phi * _s_theta *                             \
                   (_c_phi *                                           \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -    \
                    LINKS_COM[LINK_YAW][2] * _s_phi)) *                                 \
              (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +            \
               LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                                 \
               _s_phi * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +                    \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_phi * _c_theta *                \
                                (_s_psi * _s_theta -                       \
                                 _c_psi * _c_theta * _s_phi) +           \
                            _s_phi * _c_phi * _c_psi -             \
                            _c_phi * _s_theta *                \
                                (_c_theta * _s_psi +                       \
                                 _c_psi * _s_phi * _s_theta)) +          \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _s_theta *                \
                                (_c_psi * _c_theta -                       \
                                 _s_phi * _s_psi * _s_theta) -           \
                            _c_phi * _c_theta *                \
                                (_c_psi * _s_theta +                       \
                                 _c_theta * _s_phi * _s_psi) +           \
                            _s_phi * _c_phi * _s_psi) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_s_phi * _s_phi +                        \
                            _c_phi * _c_theta * _c_phi *     \
                                _c_theta +                                   \
                            _c_phi * _s_theta * _c_phi *     \
                                _s_theta))) -                                \
      _acceleration(ROLL) *(                                                              \
          _c_theta *                                                         \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) -                 \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) +                 \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta))) -                \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY] * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) -                 \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY] * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX] * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) +                 \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ] * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ] * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ] * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta))) +                \
          _c_theta *                                                         \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) +                              \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) +                              \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta))) +                             \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX] * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) +                              \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY] * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY] * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) -                              \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ] * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ] * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ] * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta))) +                             \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_phi * _s_theta *                             \
                   (LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) - \
                    LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                               _c_theta * _s_phi *             \
                                   _s_psi) +                           \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                    _c_phi * _c_theta * d6) -            \
               _c_phi * _c_theta *                             \
                   (LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) - \
                    LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                    _c_phi * _s_theta * d6)) *           \
              (_c_theta * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) -    \
                             LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                 \
                             d6 * _c_phi * _c_theta) +                     \
               _s_theta * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                             LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                             LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                 \
                             d6 * _c_phi * _s_theta)) -                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_c_theta *                                                    \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +               \
               _s_theta *                                                    \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) *              \
              (_c_phi * _c_theta *                             \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi * _s_theta *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -   \
               _c_phi * _s_theta *                             \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +  \
          _massLinks(LINK_PEDAL) * _s_theta *                                            \
              (LINKS_COM[LINK_YAW][2] * _s_phi -                                              \
               _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) *             \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                \
                    _s_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    _c_theta * _s_phi *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _c_phi * _c_theta *                             \
                   (_c_phi *                                           \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -    \
                    LINKS_COM[LINK_YAW][2] * _s_phi)) +                                 \
          _massLinks(LINK_PEDAL) * _s_theta *                                            \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) - \
                    LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _s_theta +                   \
                         _c_theta * _s_phi * _s_psi) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                    _c_phi * _c_theta * d6) -            \
               _c_phi * _c_theta *                             \
                   (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +     \
                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                \
                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *              \
              (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                            \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                                 \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -                                \
          _massLinks(LINK_PEDAL) * _c_theta *                                            \
              (LINKS_COM[LINK_YAW][2] * _s_phi -                                              \
               _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) *             \
              (_s_phi *                                                \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +    \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                \
                    _s_phi * _s_theta *                        \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
               _c_phi * _s_theta *                             \
                   (_c_phi *                                           \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -    \
                    LINKS_COM[LINK_YAW][2] * _s_phi)) -                                 \
          _massLinks(LINK_PEDAL) * _c_theta *                                            \
              (_s_phi *                                                \
                   (LINKS_COM[LINK_PEDAL][0] *                                                  \
                        (_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) - \
                    LINKS_COM[LINK_PEDAL][1] *                                                  \
                        (_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) + \
                    LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                    _c_phi * _s_theta * d6) -            \
               _c_phi * _s_theta *                             \
                   (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +     \
                    LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                \
                    LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) *              \
              (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                            \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi + LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))
#endif MACROS_COMPENSATION_INERTIA_H