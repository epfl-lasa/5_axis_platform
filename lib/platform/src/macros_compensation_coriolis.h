#ifndef MACROS_COMPENSATION_CORIOLIS_H
#define MACROS_COMPENSATION_CORIOLIS_H

#include "macros_compensation_inertia.h"

/************************MACROS FOR EQUATION COMPENSATION*******************************/
#define COMP_CORIOLIS_EQ_Y \
_speed(PITCH)                                          \
  *(_massLinks(LINK_PEDAL) *                                                               \
        (_speed(PITCH) *                                                           \
             (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +             \
              LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                                  \
              _s_phi * _s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) + \
         _speed(ROLL) *                                                            \
             (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                                  \
              _c_phi * _c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) + \
         _speed(YAW) * (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) +              \
    _massLinks(LINK_PEDAL) * (_speed(PITCH) * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -              \
                                          _s_phi * _s_psi * _s_theta) -  \
                               LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +              \
                                          _c_psi * _s_phi * _s_theta) +  \
                               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +               \
                               d6 * _c_phi * _s_theta) -                   \
                  _speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                             LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi)) +   \
                  _speed(ROLL) * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                \
                              d6 * _c_theta * _s_phi +                     \
                              LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +     \
                              LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) -   \
    _massLinks(LINK_PEDAL) *                                                               \
        (_speed(PITCH) * (LINKS_COM[LINK_ROLL][2] * _c_theta -                                    \
                      _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -  \
         _speed(ROLL) * _c_theta * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) +    \
    _massLinks(LINK_PEDAL) * _speed(PITCH) * (LINKS_COM[LINK_PITCH][1] * _c_theta + LINKS_COM[LINK_PITCH][0] * _s_theta)) -  \
      _speed(YAW) *(                                                               \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(YAW) *                                                       \
                   (_s_phi *                                                 \
                        (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) -       \
                    _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -   \
                                             LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) - \
               _speed(PITCH) *                                                     \
                   (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                                LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                                d6 * _c_phi * _s_theta) -                  \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                       \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +                     \
               _speed(ROLL) *                                                      \
                   (_c_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) - \
                                LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                                d6 * _c_phi * _c_theta) -                  \
                    _s_phi * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +              \
                                d6 * _c_theta * _s_phi +                   \
                                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +   \
                                LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi) -  \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -                       \
                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +                      \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                       \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))) -                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(YAW) *                                                       \
                   (_s_phi * (_s_theta *                                   \
                                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +    \
                                _c_theta * _s_phi *                        \
                                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +   \
                        _c_phi * _c_phi                                             \
                     * _c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +  \
               _speed(PITCH) *                                                     \
                   (_s_phi *                                                 \
                        (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                       \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi -                                    \
                         _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) -  \
               _speed(ROLL) *                                                      \
                   (_c_phi *                                                 \
                        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                       \
                         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _s_phi * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                \
                                _c_phi * _c_theta *                        \
                                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -   \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _c_phi +                                    \
                         _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi -                                    \
                         _c_phi *                                            \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))))) +       \
      _speed(ROLL) *(                                                              \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) * _c_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(ROLL) * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _s_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(ROLL) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +       \
               _speed(YAW) * _s_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) +       \
          _massLinks(LINK_PEDAL) * (_speed(PITCH) * _c_theta *                               \
                            (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +          \
                        _speed(ROLL) * _s_theta *                                \
                            (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)))

/*=====================================================================================================================================================================*/

#define COMP_CORIOLIS_EQ_X                                                     \
  _speed(YAW) *(                                                                   \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(YAW) *                                                           \
               (_c_phi * _c_theta *                                        \
                    (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -      \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +              \
                _c_phi * _s_theta *                                        \
                    (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) -             \
           _speed(ROLL) *                                                          \
               (_c_theta * _s_phi *                                        \
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _s_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _c_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                           \
                     _c_phi * _c_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +              \
                _c_phi * _c_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                           \
                     _c_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) -            \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(ROLL) * (_c_phi * _c_theta *                                 \
                           (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                  \
                            d6 * _s_phi * _s_theta +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) -      \
                       _s_phi * _s_theta *                                 \
                           (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) +                      \
                       _c_theta * _s_phi *                                 \
                           (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) -                      \
                       _c_phi * _s_theta *                                 \
                           (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                  \
                            d6 * _c_theta * _s_phi +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) +     \
           _speed(YAW) * (_c_phi * _c_theta *                                  \
                          (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta)) +     \
                      _c_phi * _s_theta *                                  \
                          (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                           LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi))))) -  \
      _speed(ROLL) *(                                                              \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(YAW) *                                                       \
                   (_c_theta *                                               \
                        (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -          \
                    _s_theta *                                               \
                        (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -  \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) +         \
               _speed(ROLL) *                                                      \
                   (_c_theta * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -              \
                                  _c_phi * _c_theta *                      \
                                      (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) + \
                    _s_theta *                                               \
                        (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                       \
                         _c_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) -        \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(YAW) *                                                       \
                   (_c_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) -       \
                    _s_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta))) -      \
               _speed(ROLL) *                                                      \
                   (_s_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                     \
                         d6 * _s_phi * _s_theta +                          \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +          \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) +         \
                    _c_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                     \
                         d6 * _c_theta * _s_phi +                          \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +          \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi))) +       \
          _massLinks(LINK_PEDAL) * _speed(ROLL) *                                              \
              (_c_theta * _c_theta                                                    \
                * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) + _s_theta * _s_theta      \
               * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)))

/*=====================================================================================================================================================================*/
#define COMP_CORIOLIS_EQ_PITCH                                                 \
  _speed(YAW) *(                                                                   \
      (_speed(ROLL) * _c_theta * _s_phi + _speed(PITCH) * _c_phi * _s_theta) * \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                               \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) -                               \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi)) +                              \
      (_speed(PITCH) * _c_phi * _c_theta - _speed(ROLL) * _s_phi * _s_theta) * \
          ((_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                               \
           (_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi) +                               \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi)) +                              \
      _s_phi *                                                               \
          (_s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _speed(ROLL) * _c_phi) -                  \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _speed(ROLL) * _c_phi) +                               \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _speed(ROLL) * _c_phi)) -                              \
      (_speed(ROLL) * _c_theta * _s_phi + _speed(PITCH) * _c_phi * _s_theta) * \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) -                             \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) +                             \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) +                            \
      (_speed(PITCH) * _c_phi * _c_theta - _speed(ROLL) * _s_phi * _s_theta) * \
          ((_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) -                             \
           (_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                             \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) +                            \
      _s_phi *                                                               \
          (_s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _speed(ROLL) * _c_phi) +                \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _speed(ROLL) * _c_phi) +                             \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _speed(ROLL) * _c_phi)) -                            \
      _c_phi * _c_theta *                                                  \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _speed(ROLL) * _c_phi) +                               \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _speed(ROLL) * _c_phi) -                               \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _speed(ROLL) * _c_phi)) +                              \
      _c_phi * _s_theta *                                                  \
          ((_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _speed(ROLL) * _c_phi) +                               \
           (_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _speed(ROLL) * _c_phi) +                               \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_speed(YAW) * _c_phi * _s_psi +                     \
                           _speed(ROLL) * _c_psi * _s_phi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_speed(YAW) * _c_phi * _c_psi -                     \
                           _speed(ROLL) * _s_phi * _s_psi) +                   \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _speed(ROLL) * _c_phi)) +                              \
      _speed(ROLL) * _c_phi *                                                    \
          (_s_phi *                                                    \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi) +                             \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                             \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +       \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi)) -                            \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(ROLL) *                                                          \
               (_s_phi * (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                  \
                            d6 * _s_phi * _s_theta +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) -      \
                _c_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) +                      \
                _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -       \
                                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) -      \
                _s_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) -     \
           _speed(PITCH) *                                                         \
               (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) -                      \
                _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +     \
           _speed(YAW) *                                                           \
               (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta)) +    \
                _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -       \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi))) *    \
          (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                      \
                      _s_phi * _s_psi * _s_theta) -    \
           LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                      \
                      _c_psi * _s_phi * _s_theta) +    \
           LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                       \
           _c_phi * _s_theta * d6) -                     \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(YAW) *                                                           \
               (_s_phi *                                                     \
                        (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +          \
                    _c_phi *_c_phi                                                 \
                * _c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +      \
           _speed(PITCH) *                                                         \
               (_s_phi *                                                     \
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _c_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) -      \
           _speed(ROLL) *                                                          \
               (_c_phi *                                                     \
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _s_phi * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                    \
                            _c_phi * _c_theta *                            \
                                (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -       \
                _c_phi * _c_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_phi +                                        \
                     _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
                _c_theta * _s_phi *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) *     \
          (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                         \
           _s_theta *                                                  \
               (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +             \
           _c_theta * _s_phi *                                 \
               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +            \
      _speed(ROLL) * _c_phi *                                                    \
          (_s_phi *                                                    \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _c_phi * _s_psi) -                               \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _s_psi) +                               \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _s_phi - LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * _c_phi * _c_psi +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * _c_phi * _s_psi)) +                              \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(ROLL) *                                                          \
               (_c_phi *                                                     \
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _s_phi * (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                    \
                            _c_phi * _s_theta *                            \
                                (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -       \
                _c_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_phi +                                        \
                     _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
                _s_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +      \
           _speed(YAW) *                                                           \
               (_s_phi *                                                     \
                        (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -  \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -          \
                    _c_phi * _c_phi                                                 \
                 * _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +      \
           _speed(PITCH) *                                                         \
               (_s_phi *                                                     \
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _c_phi * _c_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) *     \
          (_c_theta *                                                  \
               (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +             \
           LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                         \
           _s_phi * _s_theta *                                 \
               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +            \
      _c_phi * _c_theta *                                                  \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _speed(ROLL) * _c_phi) -                             \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _speed(ROLL) * _c_phi) +                             \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _speed(ROLL) * _c_phi)) +                            \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(YAW) *                                                           \
               (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi)) -    \
                _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -       \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) -     \
           _speed(PITCH) *                                                         \
               (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) -                      \
                _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +     \
           _speed(ROLL) *                                                          \
               (_c_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) -                      \
                _s_phi * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                  \
                            d6 * _c_theta * _s_phi +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi) -      \
                _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -       \
                                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +      \
                _c_theta * _s_phi * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))) *    \
          (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                      \
                      _c_psi * _c_theta * _s_phi) -    \
           LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                      \
                      _c_theta * _s_phi * _s_psi) +    \
           LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                       \
           _c_phi * _c_theta * d6) +                     \
      _c_phi * _s_theta *                                                  \
          ((_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _speed(ROLL) * _c_phi) -                             \
           (_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _speed(ROLL) * _c_phi) +                             \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_speed(YAW) * _c_phi * _c_psi -                   \
                             _speed(ROLL) * _s_phi * _s_psi) -                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_speed(YAW) * _c_phi * _s_psi +                   \
                             _speed(ROLL) * _c_psi * _s_phi) +                 \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _speed(ROLL) * _c_phi))) -                           \
      _speed(ROLL) *(                                                              \
          _c_theta * ((_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _speed(ROLL) * _c_phi) -                \
                        (_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _speed(ROLL) * _c_phi) +                \
                        _c_phi * _s_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _speed(ROLL) * _c_phi)) -               \
          _s_theta * ((_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _speed(ROLL) * _c_phi) -                \
                        (_c_psi * _s_theta +                   \
                         _c_theta * _s_phi * _s_psi) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _speed(ROLL) * _c_phi) +                \
                        _c_phi * _c_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_speed(YAW) * _c_phi * _c_psi -      \
                                          _speed(ROLL) * _s_phi * _s_psi) -    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_speed(YAW) * _c_phi * _s_psi +      \
                                          _speed(ROLL) * _c_psi * _s_phi) +    \
                             LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _speed(ROLL) * _c_phi)) +               \
          _c_theta * (_c_theta * (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * _speed(ROLL) * _s_phi -   \
                                            LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * _speed(ROLL) * _c_phi) -  \
                        _c_phi * _s_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _speed(ROLL) * _s_phi -                  \
                             LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _speed(ROLL) * _c_phi) +                 \
                        _s_phi * _s_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _speed(ROLL) * _s_phi -                  \
                             LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _speed(ROLL) * _c_phi)) +                \
          _s_theta * (_s_theta * (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * _speed(ROLL) * _s_phi -   \
                                            LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * _speed(ROLL) * _c_phi) +  \
                        _c_phi * _c_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _speed(ROLL) * _s_phi -                  \
                             LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _speed(ROLL) * _c_phi) -                 \
                        _c_theta * _s_phi *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _speed(ROLL) * _s_phi -                  \
                             LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _speed(ROLL) * _c_phi)) +                \
          _c_theta * ((_c_theta * _s_psi +                   \
                         _c_psi * _s_phi * _s_theta) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _speed(ROLL) * _c_phi) +                  \
                        (_c_psi * _c_theta -                   \
                         _s_phi * _s_psi * _s_theta) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _speed(ROLL) * _c_phi) +                  \
                        _c_phi * _s_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _speed(ROLL) * _c_phi)) +                 \
          _s_theta * ((_s_psi * _s_theta -                   \
                         _c_psi * _c_theta * _s_phi) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _speed(ROLL) * _c_phi) +                  \
                        (_c_psi * _s_theta +                   \
                         _c_theta * _s_phi * _s_psi) * \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _speed(ROLL) * _c_phi) -                  \
                        _c_phi * _c_theta *                    \
                            (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_speed(YAW) * _c_phi * _s_psi +        \
                                        _speed(ROLL) * _c_psi * _s_phi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_speed(YAW) * _c_phi * _c_psi -        \
                                        _speed(ROLL) * _s_phi * _s_psi) +      \
                             LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _speed(ROLL) * _c_phi)) +                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(ROLL) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +       \
               _speed(YAW) * _s_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) *       \
              (LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_s_psi * _s_theta -                        \
                    _c_psi * _c_theta * _s_phi) -      \
               LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _s_theta +                        \
                    _c_theta * _s_phi * _s_psi) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                   \
               _c_phi * _c_theta * d6) -                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(ROLL) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) -       \
               _speed(PITCH) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(YAW) * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) *       \
              (LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _c_theta -                        \
                    _s_phi * _s_psi * _s_theta) -      \
               LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_c_theta * _s_psi +                        \
                    _c_psi * _s_phi * _s_theta) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                   \
               _c_phi * _s_theta * d6) -                 \
          _speed(PITCH) * _c_theta *                                             \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) -                         \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) +                         \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) +                        \
          _massLinks(LINK_PEDAL) * (_c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -          \
                                            LINKS_COM[LINK_ROLL][1] * _s_phi) +         \
                        LINKS_COM[LINK_ROLL][2] * _s_theta) *                           \
              (_speed(PITCH) * _c_theta *                                        \
                   (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +                   \
               _speed(ROLL) * _s_theta *                                         \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -                  \
          _massLinks(LINK_PEDAL) * (_s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -          \
                                            LINKS_COM[LINK_ROLL][1] * _s_phi) -         \
                        LINKS_COM[LINK_ROLL][2] * _c_theta) *                           \
              (_speed(ROLL) * _c_theta *                                         \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) -                   \
               _speed(PITCH) * _s_theta *                                        \
                   (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) -                  \
          _speed(PITCH) * _s_theta *                                             \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * _c_phi * _s_psi) -                         \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * _c_phi * _s_psi) +                         \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _s_phi + LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _c_phi * _c_psi +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _c_phi * _s_psi)) -                        \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(ROLL) * _c_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -        \
               _speed(PITCH) * _s_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _c_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) *                    \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                     \
               _s_phi * _s_theta *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) * _c_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(ROLL) * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _s_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) *                    \
              (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                     \
               _s_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               _c_theta * _s_phi *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
          _speed(PITCH) * _c_theta *                                             \
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
          _speed(PITCH) * _c_theta *                                             \
              (_s_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * _s_phi) +               \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _s_phi) -               \
               _c_theta * _s_phi *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _s_phi)) -              \
          _speed(PITCH) * _s_theta *                                             \
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
          _speed(PITCH) * _s_theta *                                             \
              (_c_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * _s_phi) -               \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _s_phi) +               \
               _s_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _c_phi + LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _s_phi))) +             \
      _speed(PITCH) *(                                                             \
          _s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_speed(YAW) * _c_phi * _c_psi -       \
                                         _speed(ROLL) * _s_phi * _s_psi) -     \
                            LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_speed(YAW) * _c_phi * _s_psi +       \
                                         _speed(ROLL) * _c_psi * _s_phi) +     \
                            LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * _speed(ROLL) * _c_phi) -                 \
          _s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _speed(ROLL) * _s_phi -                   \
                            LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * _speed(ROLL) * _c_phi) +                  \
          _s_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) +       \
                            LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                            LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * _speed(ROLL) * _c_phi) -                   \
          _c_phi * (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * _speed(ROLL) * _s_phi -                   \
                            LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * _speed(ROLL) * _c_phi) +                  \
          _massLinks(LINK_PEDAL) * (_speed(PITCH) * (LINKS_COM[LINK_ROLL][2] * _s_theta +                     \
                                     _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -         \
                                                   LINKS_COM[LINK_ROLL][1] * _s_phi)) -       \
                        _speed(ROLL) * _s_theta *                                \
                            (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) *         \
              (_s_theta *                                              \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) -       \
               LINKS_COM[LINK_ROLL][2] * _c_theta) -                                    \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta)) -      \
               _speed(PITCH) * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) +                      \
               _speed(ROLL) * (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                   \
                           d6 * _s_phi * _s_theta +                        \
                           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +        \
                           LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta)) *      \
              (LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _c_theta -                        \
                    _s_phi * _s_psi * _s_theta) -      \
               LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_c_theta * _s_psi +                        \
                    _c_psi * _s_phi * _s_theta) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                   \
               _c_phi * _s_theta * d6) +                 \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _c_psi -                    \
                            _speed(ROLL) * _s_phi * _s_psi) -                  \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_speed(YAW) * _c_phi * _s_psi +                    \
                            _speed(ROLL) * _c_psi * _s_phi) +                  \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * _speed(ROLL) * _c_phi) +                              \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_speed(YAW) * _c_phi * _c_psi -                    \
                            _speed(ROLL) * _s_phi * _s_psi) -                  \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_speed(YAW) * _c_phi * _s_psi +                    \
                            _speed(ROLL) * _c_psi * _s_phi) +                  \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * _speed(ROLL) * _c_phi) -                              \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_speed(YAW) * _c_phi * _s_psi +                      \
                          _speed(ROLL) * _c_psi * _s_phi) +                    \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _c_psi -                      \
                          _speed(ROLL) * _s_phi * _s_psi) +                    \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * _speed(ROLL) * _c_phi) +                                \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_speed(YAW) * _c_phi * _s_psi +                      \
                          _speed(ROLL) * _c_psi * _s_phi) +                    \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_speed(YAW) * _c_phi * _c_psi -                      \
                          _speed(ROLL) * _s_phi * _s_psi) +                    \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * _speed(ROLL) * _c_phi) -                                \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) *                                                     \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +               \
               _speed(ROLL) * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                     \
                           _c_phi * _c_theta *                             \
                               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) + \
                          _c_theta * _s_phi *                              \
                              (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) *        \
              (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                     \
               _s_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               _c_theta * _s_phi *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -        \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) -                      \
               _speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi)) +      \
               _speed(ROLL) * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                   \
                           d6 * _c_theta * _s_phi +                        \
                           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +        \
                           LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) *      \
              (LINKS_COM[LINK_PEDAL][1] *                                                       \
                   (_s_psi * _s_theta -                        \
                    _c_psi * _c_theta * _s_phi) -      \
               LINKS_COM[LINK_PEDAL][0] *                                                       \
                   (_c_psi * _s_theta +                        \
                    _c_theta * _s_phi * _s_psi) +      \
               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                   \
               _c_phi * _c_theta * d6) +                 \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(PITCH) *                                                     \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _speed(ROLL) * (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                     \
                           _c_phi * _s_theta *                             \
                               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) - \
                          _s_phi * _s_theta *                              \
                              (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) *        \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +         \
               LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                     \
               _s_phi * _s_theta *                             \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
          _massLinks(LINK_PEDAL) * (_speed(PITCH) * (LINKS_COM[LINK_ROLL][2] * _c_theta -                     \
                                     _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -         \
                                                   LINKS_COM[LINK_ROLL][1] * _s_phi)) -       \
                        _speed(ROLL) * _c_theta *                                \
                            (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) *         \
              (_c_theta *                                              \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) +       \
               LINKS_COM[LINK_ROLL][2] * _s_theta) +                                    \
          _massLinks(LINK_PEDAL) * _speed(PITCH) *                                             \
              (LINKS_COM[LINK_PITCH][1] * _c_theta + LINKS_COM[LINK_PITCH][0] * _s_theta) *      \
              (LINKS_COM[LINK_PITCH][0] * _c_theta - LINKS_COM[LINK_PITCH][1] * _s_theta) -                  \
          _massLinks(LINK_PEDAL) * _speed(PITCH) *                                             \
              (LINKS_COM[LINK_PITCH][0] * _c_theta - LINKS_COM[LINK_PITCH][1] * _s_theta) *      \
              (LINKS_COM[LINK_PITCH][1] * _c_theta + LINKS_COM[LINK_PITCH][0] * _s_theta))

/*=====================================================================================================================================================================*/
#define COMP_CORIOLIS_EQ_ROLL                                                  \
  -_speed(PITCH) *(                                                                \
      _c_phi *                                                         \
          (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_speed(PITCH) * _c_theta * _s_theta -             \
                       _speed(PITCH) * _s_theta * _c_theta) +            \
           LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  *                                                          \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)) -      \
           LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  *                                                          \
               (_c_theta * (_speed(ROLL) * _c_phi * _s_theta +         \
                                    _speed(PITCH) * _c_theta * _s_phi) -       \
                _s_theta * (_speed(ROLL) * _c_phi * _c_theta -         \
                                    _speed(PITCH) * _s_phi * _s_theta))) -     \
      _s_phi *                                                         \
          (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_s_theta *                                     \
                            (_speed(YAW) * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                             _speed(PITCH) * (_c_theta * _s_psi +              \
                                          _c_psi * _s_phi * _s_theta) -  \
                             _speed(ROLL) * _c_phi * _c_psi * _c_theta) +    \
                        _c_theta *                                     \
                            (_speed(YAW) * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                             _speed(PITCH) * (_s_psi * _s_theta -              \
                                          _c_psi * _c_theta * _s_phi) +  \
                             _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +   \
           LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                                     \
                            (_speed(YAW) * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                             _speed(PITCH) * (_c_psi * _s_theta +              \
                                          _c_theta * _s_phi * _s_psi) +  \
                             _speed(ROLL) * _c_phi * _s_psi * _s_theta) -    \
                        _s_theta *                                     \
                            (_speed(PITCH) * (_c_psi * _c_theta -              \
                                          _s_phi * _s_psi * _s_theta) -  \
                             _speed(YAW) * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                             _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -   \
           LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  *                                                         \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
      _s_phi *                                                         \
          (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_theta *                                       \
                          (_speed(YAW) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                           _speed(PITCH) * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) -    \
                           _speed(ROLL) * _c_phi * _c_psi * _c_theta) +      \
                      _c_theta *                                       \
                          (_speed(YAW) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                           _speed(PITCH) * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                           _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -     \
           LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                                       \
                          (_speed(YAW) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           _speed(PITCH) * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                           _speed(ROLL) * _c_phi * _s_psi * _s_theta) -      \
                      _s_theta *                                       \
                          (_speed(PITCH) * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                           _speed(YAW) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                           _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +     \
           LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  *                                                           \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
      _s_phi *                                                         \
          (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_speed(PITCH) * _c_theta * _s_theta -             \
                       _speed(PITCH) * _s_theta * _c_theta) +            \
           LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  *                                                          \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)) -      \
           LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  *                                                          \
               (_c_theta * (_speed(ROLL) * _c_phi * _s_theta +         \
                                    _speed(PITCH) * _c_theta * _s_phi) -       \
                _s_theta * (_speed(ROLL) * _c_phi * _c_theta -         \
                                    _speed(PITCH) * _s_phi * _s_theta))) -     \
      _c_phi * _c_psi *                                        \
          (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_s_theta *                                     \
                            (_speed(YAW) * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                             _speed(PITCH) * (_c_theta * _s_psi +              \
                                          _c_psi * _s_phi * _s_theta) -  \
                             _speed(ROLL) * _c_phi * _c_psi * _c_theta) +    \
                        _c_theta *                                     \
                            (_speed(YAW) * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                             _speed(PITCH) * (_s_psi * _s_theta -              \
                                          _c_psi * _c_theta * _s_phi) +  \
                             _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +   \
           LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                                     \
                            (_speed(YAW) * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                             _speed(PITCH) * (_c_psi * _s_theta +              \
                                          _c_theta * _s_phi * _s_psi) +  \
                             _speed(ROLL) * _c_phi * _s_psi * _s_theta) -    \
                        _s_theta *                                     \
                            (_speed(PITCH) * (_c_psi * _c_theta -              \
                                          _s_phi * _s_psi * _s_theta) -  \
                             _speed(YAW) * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                             _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -   \
           LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                         \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) -     \
      _c_phi * _s_psi *                                        \
          (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_s_theta *                                     \
                            (_speed(YAW) * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                             _speed(PITCH) * (_c_theta * _s_psi +              \
                                          _c_psi * _s_phi * _s_theta) -  \
                             _speed(ROLL) * _c_phi * _c_psi * _c_theta) +    \
                        _c_theta *                                     \
                            (_speed(YAW) * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                             _speed(PITCH) * (_s_psi * _s_theta -              \
                                          _c_psi * _c_theta * _s_phi) +  \
                             _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +   \
           LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_theta *                                     \
                            (_speed(YAW) * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) +    \
                             _speed(PITCH) * (_c_psi * _s_theta +              \
                                          _c_theta * _s_phi * _s_psi) +  \
                             _speed(ROLL) * _c_phi * _s_psi * _s_theta) -    \
                        _s_theta *                                     \
                            (_speed(PITCH) * (_c_psi * _c_theta -              \
                                          _s_phi * _s_psi * _s_theta) -  \
                             _speed(YAW) * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                             _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -   \
           LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                         \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) -     \
      _c_phi * _c_psi *                                        \
          (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_s_theta *                                       \
                          (_speed(YAW) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                           _speed(PITCH) * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) -    \
                           _speed(ROLL) * _c_phi * _c_psi * _c_theta) +      \
                      _c_theta *                                       \
                          (_speed(YAW) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                           _speed(PITCH) * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                           _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -     \
           LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                                       \
                          (_speed(YAW) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           _speed(PITCH) * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                           _speed(ROLL) * _c_phi * _s_psi * _s_theta) -      \
                      _s_theta *                                       \
                          (_speed(PITCH) * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                           _speed(YAW) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                           _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +     \
           LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                           \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
      _c_phi * _s_psi *                                        \
          (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_s_theta *                                       \
                          (_speed(YAW) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                           _speed(PITCH) * (_c_theta * _s_psi +                \
                                        _c_psi * _s_phi * _s_theta) -    \
                           _speed(ROLL) * _c_phi * _c_psi * _c_theta) +      \
                      _c_theta *                                       \
                          (_speed(YAW) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                           _speed(PITCH) * (_s_psi * _s_theta -                \
                                        _c_psi * _c_theta * _s_phi) +    \
                           _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -     \
           LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_theta *                                       \
                          (_speed(YAW) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           _speed(PITCH) * (_c_psi * _s_theta +                \
                                        _c_theta * _s_phi * _s_psi) +    \
                           _speed(ROLL) * _c_phi * _s_psi * _s_theta) -      \
                      _s_theta *                                       \
                          (_speed(PITCH) * (_c_psi * _c_theta -                \
                                        _s_phi * _s_psi * _s_theta) -    \
                           _speed(YAW) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                           _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +     \
           LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                           \
               (_c_theta * (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                _s_theta * (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
      _massLinks(LINK_PEDAL) * _c_theta *                                          \
          (_speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                       \
                                 _c_psi * _s_phi * _s_theta) +           \
                      LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                       \
                                 _s_phi * _s_psi * _s_theta)) -          \
           _speed(PITCH) * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                     \
                                   _c_psi * _c_theta * _s_phi) -         \
                        LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                     \
                                   _c_theta * _s_phi * _s_psi) +         \
                        LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                      \
                        d6 * _c_phi * _c_theta) +                          \
           _speed(ROLL) *                                                          \
               (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta + d6 * _s_phi * _s_theta + \
                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +                   \
                LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta)) *                 \
          (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +              \
           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                         \
           LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -                        \
      _massLinks(LINK_PEDAL) * _c_theta *                                          \
          (_speed(PITCH) *                                                         \
               (LINKS_COM[LINK_ROLL][2] * _s_theta +                                          \
                _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -        \
           _speed(ROLL) * _s_theta * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) *  \
          (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +                \
      _massLinks(LINK_PEDAL) * _s_theta *                                          \
          (_speed(PITCH) *                                                         \
               (LINKS_COM[LINK_ROLL][2] * _c_theta -                                          \
                _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi)) -        \
           _speed(ROLL) * _c_theta * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) *  \
          (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +                \
      _massLinks(LINK_PEDAL) * _s_theta *                                          \
          (_c_phi *                                                    \
               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -             \
           LINKS_COM[LINK_YAW][2] * _s_phi) *                                           \
          (_speed(PITCH) * (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +   \
                        LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                        \
                        _s_phi * _s_theta *                                \
                            (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +           \
           _speed(ROLL) * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                         \
                       _c_phi * _c_theta *                                 \
                           (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +            \
           _speed(YAW) * (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +     \
                      _c_theta * _s_phi *                                  \
                          (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) +            \
      _massLinks(LINK_PEDAL) * _c_theta *                                          \
          (_c_phi *                                                    \
               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -             \
           LINKS_COM[LINK_YAW][2] * _s_phi) *                                           \
          (_speed(PITCH) * (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                        \
                        _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +   \
                        _c_theta * _s_phi *                                \
                            (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -           \
           _speed(ROLL) * (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                         \
                       _c_phi * _s_theta *                                 \
                           (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +            \
           _speed(YAW) * (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -     \
                      _s_phi * _s_theta *                                  \
                          (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) -            \
      _massLinks(LINK_PEDAL) * _s_theta *                                          \
          (_speed(PITCH) * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                     \
                                   _s_phi * _s_psi * _s_theta) -         \
                        LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                     \
                                   _c_psi * _s_phi * _s_theta) +         \
                        LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                      \
                        d6 * _c_phi * _s_theta) -                          \
           _speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                       \
                                 _c_psi * _c_theta * _s_phi) +           \
                      LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                       \
                                 _c_theta * _s_phi * _s_psi)) +          \
           _speed(ROLL) *                                                          \
               (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi + d6 * _c_theta * _s_phi + \
                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +                   \
                LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) *                 \
          (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +              \
           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                         \
           LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) -                       \
      _speed(ROLL) *(                                                              \
          _c_theta *                                                         \
              (_c_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][ZZ]  * (_speed(PITCH) * _c_theta * _s_theta -    \
                                _speed(PITCH) * _s_theta * _c_theta) +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_c_theta *                             \
                                    (_speed(PITCH) * _c_phi * _c_theta -       \
                                     _speed(ROLL) * _s_phi * _s_theta) +       \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_theta * _s_phi +        \
                                     _speed(PITCH) * _c_phi * _s_theta)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_c_theta *                             \
                                    (_speed(ROLL) * _c_phi * _s_theta +        \
                                     _speed(PITCH) * _c_theta * _s_phi) -      \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_phi * _c_theta -        \
                                     _speed(PITCH) * _s_phi * _s_theta))) +    \
               _s_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_speed(PITCH) * _c_theta * _s_theta -    \
                                _speed(PITCH) * _s_theta * _c_theta) +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta *                             \
                                    (_speed(PITCH) * _c_phi * _c_theta -       \
                                     _speed(ROLL) * _s_phi * _s_theta) +       \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_theta * _s_phi +        \
                                     _speed(PITCH) * _c_phi * _s_theta)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * (_c_theta *                             \
                                    (_speed(ROLL) * _c_phi * _s_theta +        \
                                     _speed(PITCH) * _c_theta * _s_phi) -      \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_phi * _c_theta -        \
                                     _speed(PITCH) * _s_phi * _s_theta))) -    \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_speed(PITCH) * _c_theta * _s_theta -    \
                                _speed(PITCH) * _s_theta * _c_theta) +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * (_c_theta *                             \
                                    (_speed(PITCH) * _c_phi * _c_theta -       \
                                     _speed(ROLL) * _s_phi * _s_theta) +       \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_theta * _s_phi +        \
                                     _speed(PITCH) * _c_phi * _s_theta)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta *                             \
                                    (_speed(ROLL) * _c_phi * _s_theta +        \
                                     _speed(PITCH) * _c_theta * _s_phi) -      \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_phi * _c_theta -        \
                                     _speed(PITCH) * _s_phi * _s_theta)))) -   \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) -     \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)))) -    \
          _c_theta *                                                         \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)))) +    \
          _s_theta *                                                         \
              (_s_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][ZZ]  * (_speed(PITCH) * _c_theta * _s_theta -    \
                                _speed(PITCH) * _s_theta * _c_theta) +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_c_theta *                             \
                                    (_speed(PITCH) * _c_phi * _c_theta -       \
                                     _speed(ROLL) * _s_phi * _s_theta) +       \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_theta * _s_phi +        \
                                     _speed(PITCH) * _c_phi * _s_theta)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_c_theta *                             \
                                    (_speed(ROLL) * _c_phi * _s_theta +        \
                                     _speed(PITCH) * _c_theta * _s_phi) -      \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_phi * _c_theta -        \
                                     _speed(PITCH) * _s_phi * _s_theta))) +    \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_speed(PITCH) * _c_theta * _s_theta -    \
                                _speed(PITCH) * _s_theta * _c_theta) +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * (_c_theta *                             \
                                    (_speed(PITCH) * _c_phi * _c_theta -       \
                                     _speed(ROLL) * _s_phi * _s_theta) +       \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_theta * _s_phi +        \
                                     _speed(PITCH) * _c_phi * _s_theta)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta *                             \
                                    (_speed(ROLL) * _c_phi * _s_theta +        \
                                     _speed(PITCH) * _c_theta * _s_phi) -      \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_phi * _c_theta -        \
                                     _speed(PITCH) * _s_phi * _s_theta))) -    \
               _c_theta * _s_phi *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_speed(PITCH) * _c_theta * _s_theta -    \
                                _speed(PITCH) * _s_theta * _c_theta) +   \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta *                             \
                                    (_speed(PITCH) * _c_phi * _c_theta -       \
                                     _speed(ROLL) * _s_phi * _s_theta) +       \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_theta * _s_phi +        \
                                     _speed(PITCH) * _c_phi * _s_theta)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * (_c_theta *                             \
                                    (_speed(ROLL) * _c_phi * _s_theta +        \
                                     _speed(PITCH) * _c_theta * _s_phi) -      \
                                _s_theta *                             \
                                    (_speed(ROLL) * _c_phi * _c_theta -        \
                                     _speed(PITCH) * _s_phi * _s_theta)))) +   \
          _c_theta *                                                         \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) -   \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) +   \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta)))) -  \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) -   \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) +   \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta)))) +  \
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
              (_speed(YAW) *                                                       \
                   (_c_theta *                                               \
                        (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -          \
                    _s_theta *                                               \
                        (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -  \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) +         \
               _speed(ROLL) *                                                      \
                   (_c_theta * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -              \
                                  _c_phi * _c_theta *                      \
                                      (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) + \
                    _s_theta *                                               \
                        (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                       \
                         _c_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) +        \
          _speed(PITCH) * _c_theta *                                             \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) + \
          _speed(PITCH) * _s_theta *                                             \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) - \
          _speed(PITCH) * _c_theta *                                             \
              (_s_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][ZZ]  * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) +   \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) -   \
               _c_theta * _s_phi *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi))) +  \
          _speed(PITCH) * _s_theta *                                             \
              (_c_theta *                                              \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][ZZ]  * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) +   \
               _s_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YZ]  * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][YY]  * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi)) -   \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XZ]  * (_c_theta * _c_theta +                \
                                _s_theta * _s_theta) -               \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XX]  * (_c_theta * _c_phi * _s_theta -     \
                                _s_theta * _c_phi * _c_theta) +    \
                    LINKS_MOMENT_OF_INERTIAS[LINK_ROLL][XY]  * (_c_theta * _s_phi * _s_theta -     \
                                _s_theta * _c_theta * _s_phi))) -  \
          _speed(PITCH) * _c_theta *                                             \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) -    \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _speed(PITCH) * _s_theta *                                             \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) -   \
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
              (_speed(YAW) *                                                       \
                   (_c_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) -       \
                    _s_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta))) -      \
               _speed(ROLL) *                                                      \
                   (_s_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                     \
                         d6 * _s_phi * _s_theta +                          \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +          \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) +         \
                    _c_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                     \
                         d6 * _c_theta * _s_phi +                          \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +          \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi))) -       \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_speed(ROLL) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) -       \
               _speed(PITCH) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(YAW) * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) *       \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_speed(PITCH) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(ROLL) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +       \
               _speed(YAW) * _s_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) *       \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                    \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_speed(ROLL) * _c_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -        \
               _speed(PITCH) * _s_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _c_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_speed(PITCH) * _c_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(ROLL) * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _s_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +                    \
          _massLinks(LINK_PEDAL) * _speed(ROLL) *                                              \
              (_c_theta *                                              \
                   (_c_theta *                                         \
                        (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) +  \
                    LINKS_COM[LINK_ROLL][2] * _s_theta) +                               \
               _s_theta *                                              \
                   (_s_theta *                                         \
                        (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) -  \
                    LINKS_COM[LINK_ROLL][2] * _c_theta)) *                              \
              (_c_theta * _c_theta                                                   \
               * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) + _s_theta * _s_theta      \
               * (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) -                  \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) *            \
              (_speed(ROLL) * _c_theta *                                         \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi) -                   \
               _speed(PITCH) * _s_theta *                                        \
                   (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi)) -                  \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) *            \
              (_speed(PITCH) * _c_theta *                                        \
                   (LINKS_COM[LINK_ROLL][1] * _c_phi + LINKS_COM[LINK_ROLL][0] * _s_phi) +                   \
               _speed(ROLL) * _s_theta *                                         \
                   (LINKS_COM[LINK_ROLL][0] * _c_phi - LINKS_COM[LINK_ROLL][1] * _s_phi))) -                 \
      _speed(YAW) *(                                                               \
          (_speed(PITCH) * _c_phi * _c_theta -                                 \
           _speed(ROLL) * _s_phi * _s_theta) *                                 \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _s_phi *                                                           \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) -     \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)))) -    \
          (_speed(ROLL) * _c_theta * _s_phi +                                  \
           _speed(PITCH) * _c_phi * _s_theta) *                                \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) + \
          (_speed(PITCH) * _c_phi * _c_theta -                                 \
           _speed(ROLL) * _s_phi * _s_theta) *                                 \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) -  \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) - \
          _s_phi *                                                           \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) +   \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) +   \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta)))) +  \
          (_speed(ROLL) * _c_theta * _s_phi +                                  \
           _speed(PITCH) * _c_phi * _s_theta) *                                \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) -    \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) +   \
          _c_phi * _s_theta *                                              \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)))) -    \
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
              (_speed(YAW) *                                                       \
                   (_c_phi * _c_theta *                                    \
                        (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -  \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +          \
                    _c_phi * _s_theta *                                    \
                        (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) -         \
               _speed(ROLL) *                                                      \
                   (_c_theta * _s_phi *                                    \
                        (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                       \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                       \
                         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                       \
                         _c_phi * _c_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +          \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                       \
                         _c_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) +        \
          _speed(ROLL) * _c_phi *                                                \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta)) +  \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_theta *                            \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)) -     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_theta *                            \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta * _c_phi * _s_theta -    \
                                 _s_theta * _c_phi * _c_theta))) + \
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
              (_speed(ROLL) * (_c_phi * _c_theta *                             \
                               (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +              \
                                d6 * _s_phi * _s_theta +                   \
                                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +   \
                                LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) -  \
                           _s_phi * _s_theta *                             \
                               (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) - \
                                LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                                d6 * _c_phi * _c_theta) +                  \
                           _c_theta * _s_phi *                             \
                               (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                                LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                                d6 * _c_phi * _s_theta) -                  \
                           _c_phi * _s_theta *                             \
                               (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +              \
                                d6 * _c_theta * _s_phi +                   \
                                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +   \
                                LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) + \
               _speed(YAW) *                                                       \
                   (_c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta)) +       \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)))) -     \
          _c_phi * _c_theta *                                              \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) -   \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) +   \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta)))) +  \
          _speed(ROLL) * _c_phi *                                                \
              (_s_phi *                                                \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) -    \
               _c_phi * _c_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta)) +    \
               _c_phi * _s_psi *                               \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_theta *                              \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_theta *                              \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) +        \
                               _s_theta *                              \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta * _c_phi * _s_theta -      \
                               _s_theta * _c_phi * _c_theta))) -   \
          _c_phi * _s_theta *                                              \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) -   \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta))) +   \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_theta *                            \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) +      \
                                 _s_theta *                            \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta)))) -  \
          _c_phi * _c_theta *                                              \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) +     \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta))) -     \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_theta *                                    \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_theta *                                    \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_theta *                                    \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta) -   \
                         _s_theta *                                    \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_theta *                              \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) +        \
                               _s_theta *                              \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta)))) +    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_speed(YAW) *                                                       \
                   (_s_phi *                                                 \
                        (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) -       \
                    _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -   \
                                             LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) - \
               _speed(PITCH) *                                                     \
                   (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                                LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                                d6 * _c_phi * _s_theta) -                  \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                       \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +                     \
               _speed(ROLL) *                                                      \
                   (_c_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) - \
                                LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                                d6 * _c_phi * _c_theta) -                  \
                    _s_phi * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +              \
                                d6 * _c_theta * _s_phi +                   \
                                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +   \
                                LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi) -  \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -                       \
                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +                      \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                       \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))) *                    \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                    \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_speed(ROLL) *                                                      \
                   (_s_phi * (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +              \
                                d6 * _s_phi * _s_theta +                   \
                                LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +   \
                                LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) -  \
                    _c_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                                LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +              \
                                d6 * _c_phi * _s_theta) +                  \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -                       \
                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) -                      \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                       \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) -                     \
               _speed(PITCH) *                                                     \
                   (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) - \
                                LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                                LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +              \
                                d6 * _c_phi * _c_theta) -                  \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +                  \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                       \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +                     \
               _speed(YAW) * (_s_phi *                                           \
                              (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +              \
                                          _c_psi * _s_phi * _s_theta) +  \
                               LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -              \
                                          _s_phi * _s_psi * _s_theta)) + \
                          _c_phi * _s_theta *                              \
                              (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -                 \
                               LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi))) *              \
              (_s_phi * d6 + LINKS_COM[LINK_PEDAL][2] * _s_phi +          \
               LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +                     \
               LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +                    \
          _massLinks(LINK_PEDAL) * _s_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_speed(YAW) *                                                       \
                   (_s_phi * (_s_theta *                                   \
                                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +    \
                                _c_theta * _s_phi *                        \
                                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +   \
                        _c_phi * _c_phi                                             \
                    * _c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +  \
               _speed(PITCH) *                                                     \
                   (_s_phi *                                                 \
                        (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                       \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi -                                    \
                         _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) -  \
               _speed(ROLL) *                                                      \
                   (_c_phi *                                                 \
                        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                       \
                         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _s_phi * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                \
                                _c_phi * _c_theta *                        \
                                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -   \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _c_phi +                                    \
                         _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi -                                    \
                         _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) + \
          _massLinks(LINK_PEDAL) * _c_theta *                                      \
              (_c_phi *                                                \
                   (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -         \
               LINKS_COM[LINK_YAW][2] * _s_phi) *                                       \
              (_speed(ROLL) *                                                      \
                   (_c_phi *                                                 \
                        (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                       \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _s_phi * (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                \
                                _c_phi * _s_theta *                        \
                                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -   \
                    _c_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _c_phi +                                    \
                         _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +   \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi -                                    \
                         _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +  \
               _speed(YAW) *                                                       \
                   (_s_phi * (_c_theta *                                   \
                                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -    \
                                _s_phi * _s_theta *                        \
                                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -   \
                        _c_phi * _c_phi                                             \
                     * _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +  \
               _speed(PITCH) *                                                     \
                   (_s_phi *                                                 \
                        (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                       \
                         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -          \
                    _c_phi * _c_theta *                                    \
                        (LINKS_COM[LINK_YAW][2] * _s_phi -                                    \
                         _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))))

/*=====================================================================================================================================================================*/
#define COMP_CORIOLIS_EQ_YAW                                                   \
  _speed(YAW) *(                                                                   \
      (_speed(PITCH) * _c_phi * _c_theta - _speed(ROLL) * _s_phi * _s_theta) * \
          ((_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) -                                \
           (_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) +                                \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta))) -                               \
      (_speed(ROLL) * _c_theta * _s_phi + _speed(PITCH) * _c_phi * _s_theta) * \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) -                                \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) +                                \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta))) -                               \
      _s_phi *                                                               \
          (_s_phi *                                                    \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta)))) +    \
      (_speed(ROLL) * _c_theta * _s_phi + _speed(PITCH) * _c_phi * _s_theta) * \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) -                                  \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta))) +                                 \
      (_speed(PITCH) * _c_phi * _c_theta - _speed(ROLL) * _s_phi * _s_theta) * \
          ((_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           (_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta))) +                                 \
      _s_phi *                                                               \
          (_s_phi *                                                    \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_phi * (_speed(YAW) * _c_phi * _s_psi +   \
                                             _speed(ROLL) * _c_psi * _s_phi) - \
                           _c_phi * _c_theta *                 \
                               (_speed(YAW) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                                _speed(PITCH) *                                    \
                                    (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) -       \
                                _speed(ROLL) * _c_phi * _c_psi * _c_theta) + \
                           _c_phi * _s_theta *                 \
                               (_speed(YAW) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                                _speed(PITCH) *                                    \
                                    (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                                _speed(ROLL) * _c_phi * _c_psi *               \
                                    _s_theta)) +                             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -     \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta)))) +    \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(ROLL) * (_c_phi * _c_theta *                                 \
                           (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                  \
                            d6 * _s_phi * _s_theta +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) -      \
                       _s_phi * _s_theta *                                 \
                           (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) +                      \
                       _c_theta * _s_phi *                                 \
                           (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) -                      \
                       _c_phi * _s_theta *                                 \
                           (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                  \
                            d6 * _c_theta * _s_phi +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) +     \
           _speed(YAW) * (_c_phi * _c_theta *                                  \
                          (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) +      \
                           LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta)) +     \
                      _c_phi * _s_theta *                                  \
                          (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                           LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi)))) *   \
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
                _c_phi * _s_theta * d6)) -               \
      _c_phi * _c_theta *                                                  \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -     \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta)))) +    \
      _speed(ROLL) * _c_phi *                                                    \
          (_s_phi *                                                    \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) -                                  \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta)) +                                  \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_phi * _s_theta *                 \
                               (_c_psi * _c_theta -                        \
                                _s_phi * _s_psi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_c_psi * _s_theta +                        \
                                _c_theta * _s_phi * _s_psi) +            \
                           _s_phi * _c_phi * _s_psi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _c_theta *                 \
                               (_s_psi * _s_theta -                        \
                                _c_psi * _c_theta * _s_phi) +            \
                           _s_phi * _c_phi * _c_psi -              \
                           _c_phi * _s_theta *                 \
                               (_c_theta * _s_psi +                        \
                                _c_psi * _s_phi * _s_theta)) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_s_phi * _s_phi +                         \
                           _c_phi * _c_theta * _c_phi *      \
                               _c_theta +                                    \
                           _c_phi * _s_theta * _c_phi *      \
                               _s_theta))) -                                 \
      _c_phi * _s_theta *                                                  \
          ((_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -     \
           (_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                    \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -      \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_phi * _s_theta *               \
                                 (_speed(PITCH) * _c_phi * _c_theta -          \
                                  _speed(ROLL) * _s_phi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_speed(ROLL) * _c_theta * _s_phi +           \
                                  _speed(PITCH) * _c_phi * _s_theta) +         \
                             _speed(ROLL) * _s_phi * _c_phi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                    \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta)))) -    \
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
          (_speed(ROLL) *                                                          \
               (_s_phi * (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                  \
                            d6 * _s_phi * _s_theta +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) -      \
                _c_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) +                      \
                _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -       \
                                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) -      \
                _s_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) -     \
           _speed(PITCH) *                                                         \
               (_s_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) -                      \
                _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +     \
           _speed(YAW) *                                                           \
               (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta)) +    \
                _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -       \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi))) -    \
      _c_phi * _c_theta *                                                  \
          ((_s_psi * _s_theta -                                \
            _c_psi * _c_theta * _s_phi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           (_c_psi * _s_theta +                                \
            _c_theta * _s_phi * _s_psi) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -     \
           _c_phi * _c_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta)))) +    \
      _c_phi * _s_theta *                                                  \
          ((_c_theta * _s_psi +                                \
            _c_psi * _s_phi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           (_c_psi * _c_theta -                                \
            _s_phi * _s_psi * _s_theta) *              \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +     \
           _c_phi * _s_theta *                                 \
               (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                      \
                    (_s_phi * (_speed(YAW) * _c_phi * _s_psi +         \
                                       _speed(ROLL) * _c_psi * _s_phi) -       \
                     _c_phi * _c_theta *                       \
                         (_speed(YAW) * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi) +       \
                          _speed(PITCH) * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) -     \
                          _speed(ROLL) * _c_phi * _c_psi * _c_theta) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta) -       \
                          _speed(PITCH) * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                          _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +      \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_phi * _s_theta *                 \
                               (_speed(PITCH) * _c_phi * _c_theta -            \
                                _speed(ROLL) * _s_phi * _s_theta) -            \
                           _c_phi * _c_theta *                 \
                               (_speed(ROLL) * _c_theta * _s_phi +             \
                                _speed(PITCH) * _c_phi * _s_theta) +           \
                           _speed(ROLL) * _s_phi * _c_phi) -             \
                LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                      \
                    (_c_phi * _c_theta *                       \
                         (_speed(PITCH) * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                          _speed(YAW) * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          _speed(ROLL) * _c_phi * _c_theta * _s_psi) -       \
                     _s_phi * (_speed(YAW) * _c_phi * _c_psi -         \
                                       _speed(ROLL) * _s_phi * _s_psi) +       \
                     _c_phi * _s_theta *                       \
                         (_speed(YAW) * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          _speed(PITCH) * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                          _speed(ROLL) * _c_phi * _s_psi * _s_theta)))) -    \
      _massLinks(LINK_PEDAL) *                                                             \
          (_s_phi *                                                    \
               (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                    \
                _s_theta *                                             \
                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +        \
                _c_theta * _s_phi *                            \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
           _c_phi * _c_theta *                                 \
               (_c_phi *                                               \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -        \
                LINKS_COM[LINK_YAW][2] * _s_phi)) *                                     \
          (_speed(YAW) *                                                           \
               (_s_phi *                                                     \
                        (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +          \
                    _c_phi * _c_phi                                                 \
                * _c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +      \
           _speed(PITCH) *                                                         \
               (_s_phi *                                                     \
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _c_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) -      \
           _speed(ROLL) *                                                          \
               (_c_phi *                                                     \
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _s_phi * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                    \
                            _c_phi * _c_theta *                            \
                                (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -       \
                _c_phi * _c_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_phi +                                        \
                     _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
                _c_theta * _s_phi *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) +     \
      _massLinks(LINK_PEDAL) *                                                             \
          (_speed(YAW) *                                                           \
               (_c_phi * _c_theta *                                        \
                    (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -      \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +              \
                _c_phi * _s_theta *                                        \
                    (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) -             \
           _speed(ROLL) *                                                          \
               (_c_theta * _s_phi *                                        \
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _s_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _c_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                           \
                     _c_phi * _c_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +              \
                _c_phi * _c_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                           \
                     _c_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) *            \
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
          (_speed(YAW) *                                                           \
               (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) +     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi)) -    \
                _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -       \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) -     \
           _speed(PITCH) *                                                         \
               (_s_phi * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) -                      \
                _c_phi * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi)) +     \
           _speed(ROLL) *                                                          \
               (_c_phi * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) -                      \
                _s_phi * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                  \
                            d6 * _c_theta * _s_phi +                       \
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +       \
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi) -      \
                _c_phi * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -       \
                                         LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +      \
                _c_theta * _s_phi * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi))) +    \
      _speed(ROLL) * _c_phi *                                                    \
          (_s_phi *                                                    \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) +                                \
           _c_phi * _s_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta)) +                                \
           _c_phi * _c_psi *                                   \
               (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_phi * _c_theta *               \
                                 (_s_psi * _s_theta -                      \
                                  _c_psi * _c_theta * _s_phi) +          \
                             _s_phi * _c_phi * _c_psi -            \
                             _c_phi * _s_theta *               \
                                 (_c_theta * _s_psi +                      \
                                  _c_psi * _s_phi * _s_theta)) +         \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _s_theta *               \
                                 (_c_psi * _c_theta -                      \
                                  _s_phi * _s_psi * _s_theta) -          \
                             _c_phi * _c_theta *               \
                                 (_c_psi * _s_theta +                      \
                                  _c_theta * _s_phi * _s_psi) +          \
                             _s_phi * _c_phi * _s_psi) +           \
                LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_s_phi * _s_phi +                       \
                             _c_phi * _c_theta * _c_phi *    \
                                 _c_theta +                                  \
                             _c_phi * _s_theta * _c_phi *    \
                                 _s_theta))) +                               \
      _massLinks(LINK_PEDAL) *                                                             \
          (_s_phi *                                                    \
               (_c_theta *                                             \
                    (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +        \
                LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                    \
                _s_phi * _s_theta *                            \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
           _c_phi * _s_theta *                                 \
               (_c_phi *                                               \
                    (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -        \
                LINKS_COM[LINK_YAW][2] * _s_phi)) *                                     \
          (_speed(ROLL) *                                                          \
               (_c_phi *                                                     \
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                           \
                     _s_phi * _s_theta *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _s_phi * (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                    \
                            _c_phi * _s_theta *                            \
                                (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -       \
                _c_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _c_phi +                                        \
                     _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +       \
                _s_phi * _s_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))) +      \
           _speed(YAW) *                                                           \
               (_s_phi *                                                     \
                        (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -  \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -          \
                    _c_phi * _c_phi                                                 \
                 * _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) +      \
           _speed(PITCH) *                                                         \
               (_s_phi *                                                     \
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                           \
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +      \
                     _c_theta * _s_phi *                                   \
                         (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -              \
                _c_phi * _c_theta *                                        \
                    (LINKS_COM[LINK_YAW][2] * _s_phi -                                        \
                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi))))) -    \
      _speed(ROLL) *(                                                              \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _s_theta *           \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta) +     \
                                 _speed(ROLL) * _s_phi * _c_phi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) - \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *           \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta) +     \
                                 _speed(ROLL) * _s_phi * _c_phi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) + \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_phi * _s_theta *           \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta) +     \
                                 _speed(ROLL) * _s_phi * _c_phi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi *                 \
                                  _s_theta)))) -                             \
          _c_theta *                                                         \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *           \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta) +     \
                                 _speed(ROLL) * _s_phi * _c_phi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) - \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _s_theta *           \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta) +     \
                                 _speed(ROLL) * _s_phi * _c_phi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) + \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_phi * _s_theta *           \
                                     (_speed(PITCH) * _c_phi * _c_theta -      \
                                      _speed(ROLL) * _s_phi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_speed(ROLL) * _c_theta * _s_phi +       \
                                      _speed(PITCH) * _c_phi * _s_theta) +     \
                                 _speed(ROLL) * _s_phi * _c_phi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi *                 \
                                  _s_theta)))) +                             \
          _c_theta *                                                         \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _s_theta *             \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta) +       \
                               _speed(ROLL) * _s_phi * _c_phi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) + \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *             \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta) +       \
                               _speed(ROLL) * _s_phi * _c_phi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) + \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_phi * _s_theta *             \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta) +       \
                               _speed(ROLL) * _s_phi * _c_phi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi *                 \
                                  _s_theta)))) +                             \
          _s_theta *                                                         \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                  \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _s_theta *             \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta) +       \
                               _speed(ROLL) * _s_phi * _c_phi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) + \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                  \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *             \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta) +       \
                               _speed(ROLL) * _s_phi * _c_phi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                  \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi * _s_theta))) - \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                  \
                        (_s_phi * (_speed(YAW) * _c_phi * _s_psi +     \
                                           _speed(ROLL) * _c_psi * _s_phi) -   \
                         _c_phi * _c_theta *                   \
                             (_speed(YAW) * (_c_psi * _s_theta +               \
                                         _c_theta * _s_phi * _s_psi) +   \
                              _speed(PITCH) * (_c_theta * _s_psi +             \
                                           _c_psi * _s_phi * _s_theta) - \
                              _speed(ROLL) * _c_phi * _c_psi * _c_theta) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_psi * _c_theta -               \
                                         _s_phi * _s_psi * _s_theta) -   \
                              _speed(PITCH) * (_s_psi * _s_theta -             \
                                           _c_psi * _c_theta * _s_phi) + \
                              _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +  \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_phi * _s_theta *             \
                                   (_speed(PITCH) * _c_phi * _c_theta -        \
                                    _speed(ROLL) * _s_phi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_speed(ROLL) * _c_theta * _s_phi +         \
                                    _speed(PITCH) * _c_phi * _s_theta) +       \
                               _speed(ROLL) * _s_phi * _c_phi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                  \
                        (_c_phi * _c_theta *                   \
                             (_speed(PITCH) * (_c_psi * _c_theta -             \
                                           _s_phi * _s_psi * _s_theta) - \
                              _speed(YAW) * (_s_psi * _s_theta -               \
                                         _c_psi * _c_theta * _s_phi) +   \
                              _speed(ROLL) * _c_phi * _c_theta * _s_psi) -   \
                         _s_phi * (_speed(YAW) * _c_phi * _c_psi -     \
                                           _speed(ROLL) * _s_phi * _s_psi) +   \
                         _c_phi * _s_theta *                   \
                             (_speed(YAW) * (_c_theta * _s_psi +               \
                                         _c_psi * _s_phi * _s_theta) +   \
                              _speed(PITCH) * (_c_psi * _s_theta +             \
                                           _c_theta * _s_phi * _s_psi) + \
                              _speed(ROLL) * _c_phi * _s_psi *                 \
                                  _s_theta)))) -                             \
          _speed(PITCH) * _s_theta *                                             \
              ((_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) -                 \
               (_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) +                 \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta))) +                \
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
              (_speed(PITCH) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(ROLL) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) +       \
               _speed(YAW) * _s_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) +       \
          _speed(PITCH) * _c_theta *                                             \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) +                              \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) -                              \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_s_phi * _s_phi +                     \
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
              (_speed(YAW) *                                                       \
                   (_c_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi)) -       \
                    _s_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta))) -      \
               _speed(ROLL) *                                                      \
                   (_s_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                     \
                         d6 * _s_phi * _s_theta +                          \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +          \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta) +         \
                    _c_theta *                                               \
                        (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                     \
                         d6 * _c_theta * _s_phi +                          \
                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +          \
                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi))) -       \
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
              (_speed(ROLL) * _c_theta * (LINKS_COM[LINK_PEDAL][2] * _c_phi + d6 * _c_phi -   \
                                        LINKS_COM[LINK_PEDAL][1] * _c_psi * _s_phi -        \
                                        LINKS_COM[LINK_PEDAL][0] * _s_phi * _s_psi) -       \
               _speed(PITCH) * _s_theta * (LINKS_COM[LINK_PEDAL][2] * _s_phi + d6 * _s_phi +  \
                                         LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +       \
                                         LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) +      \
               _speed(YAW) * _c_theta * (LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_psi -         \
                                       LINKS_COM[LINK_PEDAL][1] * _c_phi * _s_psi)) -       \
          _speed(PITCH) * _s_theta *                                             \
              ((_c_theta * _s_psi +                            \
                _c_psi * _s_phi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) +                              \
               (_c_psi * _c_theta -                            \
                _s_phi * _s_psi * _s_theta) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta)) +                              \
               _c_phi * _s_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *             \
                                   (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                               _c_phi * _c_theta *             \
                                   (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                               _s_phi * _c_phi * _s_psi) -         \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _c_theta *             \
                                   (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                               _s_phi * _c_phi * _c_psi -          \
                               _c_phi * _s_theta *             \
                                   (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta)) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_s_phi * _s_phi +                     \
                               _c_phi * _c_theta * _c_phi *  \
                                   _c_theta +                                \
                               _c_phi * _s_theta * _c_phi *  \
                                   _s_theta))) +                             \
          _massLinks(LINK_PEDAL) *                                                         \
              (_speed(YAW) *                                                       \
                   (_c_theta *                                               \
                        (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) +  \
                         _c_theta * _s_phi *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -          \
                    _s_theta *                                               \
                        (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -  \
                         _s_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) +         \
               _speed(ROLL) *                                                      \
                   (_c_theta * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -              \
                                  _c_phi * _c_theta *                      \
                                      (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) + \
                    _s_theta *                                               \
                        (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                       \
                         _c_phi * _s_theta *                               \
                             (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)))) *        \
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
              (_speed(PITCH) * _c_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(ROLL) * _s_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _s_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -                    \
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
              (_speed(ROLL) * _c_theta *                                         \
                   (LINKS_COM[LINK_YAW][2] * _c_phi +                                         \
                    _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -        \
               _speed(PITCH) * _s_theta *                                        \
                   (LINKS_COM[LINK_YAW][2] * _s_phi -                                         \
                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * _c_phi * _c_theta *                               \
                   (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi)) -                    \
          _speed(PITCH) * _c_theta *                                             \
              ((_s_psi * _s_theta -                            \
                _c_psi * _c_theta * _s_phi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) -                 \
               (_c_psi * _s_theta +                            \
                _c_theta * _s_phi * _s_psi) *          \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)) +                 \
               _c_phi * _c_theta *                             \
                   (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _c_theta *           \
                                     (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                                 _s_phi * _c_phi * _c_psi -        \
                                 _c_phi * _s_theta *           \
                                     (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta)) +     \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *           \
                                     (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                                 _c_phi * _c_theta *           \
                                     (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                                 _s_phi * _c_phi * _s_psi) +       \
                    LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_s_phi * _s_phi +                   \
                                 _c_phi * _c_theta *           \
                                     _c_phi * _c_theta +                   \
                                 _c_phi * _s_theta *           \
                                     _c_phi * _s_theta)))) -               \
      _speed(PITCH) *(                                                             \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  *                                                     \
                   (_s_phi * (_speed(YAW) * _c_phi * _s_psi +          \
                                      _speed(ROLL) * _c_psi * _s_phi) -        \
                    _c_phi * _c_theta *                        \
                        (_speed(YAW) * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         _speed(PITCH) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) -      \
                         _speed(ROLL) * _c_phi * _c_psi * _c_theta) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         _speed(PITCH) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                         _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -       \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][ZZ]  * (_c_phi * _s_theta *                \
                                (_speed(PITCH) * _c_phi * _c_theta -           \
                                 _speed(ROLL) * _s_phi * _s_theta) -           \
                            _c_phi * _c_theta *                \
                                (_speed(ROLL) * _c_theta * _s_phi +            \
                                 _speed(PITCH) * _c_phi * _s_theta) +          \
                            _speed(ROLL) * _s_phi * _c_phi) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  *                                                     \
                   (_c_phi * _c_theta *                        \
                        (_speed(PITCH) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                         _speed(YAW) * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         _speed(ROLL) * _c_phi * _c_theta * _s_psi) -        \
                    _s_phi * (_speed(YAW) * _c_phi * _c_psi -          \
                                      _speed(ROLL) * _s_phi * _s_psi) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         _speed(PITCH) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                         _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -      \
          _s_phi *                                                     \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  *                                                       \
                   (_s_phi * (_speed(YAW) * _c_phi * _s_psi +          \
                                      _speed(ROLL) * _c_psi * _s_phi) -        \
                    _c_phi * _c_theta *                        \
                        (_speed(YAW) * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         _speed(PITCH) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) -      \
                         _speed(ROLL) * _c_phi * _c_psi * _c_theta) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         _speed(PITCH) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                         _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +       \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][ZZ]  * (_c_phi * _s_theta *                  \
                              (_speed(PITCH) * _c_phi * _c_theta -             \
                               _speed(ROLL) * _s_phi * _s_theta) -             \
                          _c_phi * _c_theta *                  \
                              (_speed(ROLL) * _c_theta * _s_phi +              \
                               _speed(PITCH) * _c_phi * _s_theta) +            \
                          _speed(ROLL) * _s_phi * _c_phi) -              \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  *                                                       \
                   (_c_phi * _c_theta *                        \
                        (_speed(PITCH) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                         _speed(YAW) * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         _speed(ROLL) * _c_phi * _c_theta * _s_psi) -        \
                    _s_phi * (_speed(YAW) * _c_phi * _c_psi -          \
                                      _speed(ROLL) * _s_phi * _s_psi) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         _speed(PITCH) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                         _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -      \
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
              (_speed(PITCH) *                                                     \
                   (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -                            \
                    _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    _c_theta * _s_phi *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) -               \
               _speed(ROLL) * (LINKS_COM[LINK_YAW][2] * _s_phi * _s_theta -                     \
                           _c_phi * _s_theta *                             \
                               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * (_c_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) - \
                          _s_phi * _s_theta *                              \
                              (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))) +        \
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
              (_speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_c_theta * _s_psi +                   \
                                     _c_psi * _s_phi * _s_theta) +       \
                          LINKS_COM[LINK_PEDAL][1] * (_c_psi * _c_theta -                   \
                                     _s_phi * _s_psi * _s_theta)) -      \
               _speed(PITCH) * (LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta -                 \
                                       _c_psi * _c_theta * _s_phi) -     \
                            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta +                 \
                                       _c_theta * _s_phi * _s_psi) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta +                  \
                            d6 * _c_phi * _c_theta) +                      \
               _speed(ROLL) * (LINKS_COM[LINK_PEDAL][2] * _s_phi * _s_theta +                   \
                           d6 * _s_phi * _s_theta +                        \
                           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _s_theta +        \
                           LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi * _s_theta)) +      \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YY]  *                                                     \
                   (_s_phi * (_speed(YAW) * _c_phi * _s_psi +          \
                                      _speed(ROLL) * _c_psi * _s_phi) -        \
                    _c_phi * _c_theta *                        \
                        (_speed(YAW) * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         _speed(PITCH) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) -      \
                         _speed(ROLL) * _c_phi * _c_psi * _c_theta) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         _speed(PITCH) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                         _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -       \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][YZ]  * (_c_phi * _s_theta *                \
                                (_speed(PITCH) * _c_phi * _c_theta -           \
                                 _speed(ROLL) * _s_phi * _s_theta) -           \
                            _c_phi * _c_theta *                \
                                (_speed(ROLL) * _c_theta * _s_phi +            \
                                 _speed(PITCH) * _c_phi * _s_theta) +          \
                            _speed(ROLL) * _s_phi * _c_phi) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                     \
                   (_c_phi * _c_theta *                        \
                        (_speed(PITCH) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                         _speed(YAW) * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         _speed(ROLL) * _c_phi * _c_theta * _s_psi) -        \
                    _s_phi * (_speed(YAW) * _c_phi * _c_psi -          \
                                      _speed(ROLL) * _s_phi * _s_psi) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         _speed(PITCH) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                         _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +      \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XY]  *                                                     \
                   (_s_phi * (_speed(YAW) * _c_phi * _s_psi +          \
                                      _speed(ROLL) * _c_psi * _s_phi) -        \
                    _c_phi * _c_theta *                        \
                        (_speed(YAW) * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         _speed(PITCH) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) -      \
                         _speed(ROLL) * _c_phi * _c_psi * _c_theta) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         _speed(PITCH) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                         _speed(ROLL) * _c_phi * _c_psi * _s_theta)) -       \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XZ]  * (_c_phi * _s_theta *                \
                                (_speed(PITCH) * _c_phi * _c_theta -           \
                                 _speed(ROLL) * _s_phi * _s_theta) -           \
                            _c_phi * _c_theta *                \
                                (_speed(ROLL) * _c_theta * _s_phi +            \
                                 _speed(PITCH) * _c_phi * _s_theta) +          \
                            _speed(ROLL) * _s_phi * _c_phi) +            \
               LINKS_MOMENT_OF_INERTIAS[LINK_PEDAL][XX]  *                                                     \
                   (_c_phi * _c_theta *                        \
                        (_speed(PITCH) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                         _speed(YAW) * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         _speed(ROLL) * _c_phi * _c_theta * _s_psi) -        \
                    _s_phi * (_speed(YAW) * _c_phi * _c_psi -          \
                                      _speed(ROLL) * _s_phi * _s_psi) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         _speed(PITCH) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                         _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +      \
          _c_phi * _c_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XX]  *                                                       \
                   (_s_phi * (_speed(YAW) * _c_phi * _s_psi +          \
                                      _speed(ROLL) * _c_psi * _s_phi) -        \
                    _c_phi * _c_theta *                        \
                        (_speed(YAW) * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         _speed(PITCH) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) -      \
                         _speed(ROLL) * _c_phi * _c_psi * _c_theta) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         _speed(PITCH) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                         _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +       \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XZ]  * (_c_phi * _s_theta *                  \
                              (_speed(PITCH) * _c_phi * _c_theta -             \
                               _speed(ROLL) * _s_phi * _s_theta) -             \
                          _c_phi * _c_theta *                  \
                              (_speed(ROLL) * _c_theta * _s_phi +              \
                               _speed(PITCH) * _c_phi * _s_theta) +            \
                          _speed(ROLL) * _s_phi * _c_phi) -              \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                       \
                   (_c_phi * _c_theta *                        \
                        (_speed(PITCH) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                         _speed(YAW) * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         _speed(ROLL) * _c_phi * _c_theta * _s_psi) -        \
                    _s_phi * (_speed(YAW) * _c_phi * _c_psi -          \
                                      _speed(ROLL) * _s_phi * _s_psi) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         _speed(PITCH) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                         _speed(ROLL) * _c_phi * _s_psi * _s_theta))) -      \
          _c_phi * _s_psi *                                    \
              (LINKS_MOMENT_OF_INERTIAS[LINK_YAW][XY]  *                                                       \
                   (_s_phi * (_speed(YAW) * _c_phi * _s_psi +          \
                                      _speed(ROLL) * _c_psi * _s_phi) -        \
                    _c_phi * _c_theta *                        \
                        (_speed(YAW) * (_c_psi * _s_theta +                    \
                                    _c_theta * _s_phi * _s_psi) +        \
                         _speed(PITCH) * (_c_theta * _s_psi +                  \
                                      _c_psi * _s_phi * _s_theta) -      \
                         _speed(ROLL) * _c_phi * _c_psi * _c_theta) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_psi * _c_theta -                    \
                                    _s_phi * _s_psi * _s_theta) -        \
                         _speed(PITCH) * (_s_psi * _s_theta -                  \
                                      _c_psi * _c_theta * _s_phi) +      \
                         _speed(ROLL) * _c_phi * _c_psi * _s_theta)) +       \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YZ]  * (_c_phi * _s_theta *                  \
                              (_speed(PITCH) * _c_phi * _c_theta -             \
                               _speed(ROLL) * _s_phi * _s_theta) -             \
                          _c_phi * _c_theta *                  \
                              (_speed(ROLL) * _c_theta * _s_phi +              \
                               _speed(PITCH) * _c_phi * _s_theta) +            \
                          _speed(ROLL) * _s_phi * _c_phi) -              \
               LINKS_MOMENT_OF_INERTIAS[LINK_YAW][YY]  *                                                       \
                   (_c_phi * _c_theta *                        \
                        (_speed(PITCH) * (_c_psi * _c_theta -                  \
                                      _s_phi * _s_psi * _s_theta) -      \
                         _speed(YAW) * (_s_psi * _s_theta -                    \
                                    _c_psi * _c_theta * _s_phi) +        \
                         _speed(ROLL) * _c_phi * _c_theta * _s_psi) -        \
                    _s_phi * (_speed(YAW) * _c_phi * _c_psi -          \
                                      _speed(ROLL) * _s_phi * _s_psi) +        \
                    _c_phi * _s_theta *                        \
                        (_speed(YAW) * (_c_theta * _s_psi +                    \
                                    _c_psi * _s_phi * _s_theta) +        \
                         _speed(PITCH) * (_c_psi * _s_theta +                  \
                                      _c_theta * _s_phi * _s_psi) +      \
                         _speed(ROLL) * _c_phi * _s_psi * _s_theta))) +      \
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
              (_speed(PITCH) * (LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta -                 \
                                       _s_phi * _s_psi * _s_theta) -     \
                            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi +                 \
                                       _c_psi * _s_phi * _s_theta) +     \
                            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta +                  \
                            d6 * _c_phi * _s_theta) -                      \
               _speed(YAW) * (LINKS_COM[LINK_PEDAL][0] * (_s_psi * _s_theta -                   \
                                     _c_psi * _c_theta * _s_phi) +       \
                          LINKS_COM[LINK_PEDAL][1] * (_c_psi * _s_theta +                   \
                                     _c_theta * _s_phi * _s_psi)) +      \
               _speed(ROLL) * (LINKS_COM[LINK_PEDAL][2] * _c_theta * _s_phi +                   \
                           d6 * _c_theta * _s_phi +                        \
                           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi * _c_theta +        \
                           LINKS_COM[LINK_PEDAL][0] * _c_phi * _c_theta * _s_psi)) +      \
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
              (_speed(PITCH) *                                                     \
                   (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +       \
                    LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +                            \
                    _s_phi * _s_theta *                                    \
                        (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +               \
               _speed(ROLL) * (LINKS_COM[LINK_YAW][2] * _c_theta * _s_phi -                     \
                           _c_phi * _c_theta *                             \
                               (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi)) +        \
               _speed(YAW) * (_s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) + \
                          _c_theta * _s_phi *                              \
                              (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi))))

#endif MACROS_COMPENSATION_CORIOLIS_H