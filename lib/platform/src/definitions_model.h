#ifndef DEFINITIONS_MODEL_H
#define DEFINITIONS_MODEL_H

#include "definitions_main.h"
#include "definitions_hardware.h"

//! Structure

enum link_chain {
  LINK_BASE,
  LINK_Y,
  LINK_X,
  LINK_PITCH,
  LINK_ROLL,
  LINK_YAW,
  LINK_PEDAL,
  NB_LINKS
};

enum frame_chain {
  FRAME_BASE,
  FRAME_Y,
  FRAME_X,
  FRAME_Z,
  FRAME_PITCH,
  FRAME_ROLL,
  FRAME_YAW,
  FRAME_FS,
  FRAME_PEDAL,
  FRAME_EPOINT,
  NB_FRAMES
};

//! Geometric Parameters
const float r3 = 0.305f;
#if (PRESENCE_FORCE_SENSOR)
const float d6 = 0.1326f;
//! Links Inertial Parameters
const float LINKS_COM[NB_LINKS]
                            [3] = {{0.06506077f, 0.23761375f, 0.06685000f},
                                   {0.02817123f, -0.07585496f, -0.00969366f},
                                   {-0.15754146f, 0.02306281f, 0.06946538f},
                                   {-0.00593072f, -0.00000124f, -0.00156324f},
                                   {0.03719409f, -0.00002562f, -0.00002217f},
                                   {
                                       -0.00151943f, -0.00177412f, 0.10396420f,
                                   },
                                   {-0.03259637f, 0.00014150f, 0.02254560f}};

const float LINKS_MASS[NB_LINKS] = {
    6.20696344f, 1.86377694f, 3.45202011f, 6.40922346f,
    1.17868723f, 0.42803941f, 0.37200000f};

const float LINKS_MOMENT_OF_INERTIAS[NB_LINKS][9] = {
    {0.16517950f, 0.01377293f, -0.00179536f, 0.01377293f, 0.15434303f,
     0.01782405f, -0.00179536f, 0.01782405f,
     0.29648455f} // LINK X  W.R.T COM_BASE
    ,
    {0.00343968f, 0.00007319f, 0.00001718f, 0.00007319f, 0.06820557f,
     0.00085107f, 0.00001718f, 0.00085107f, 0.06938224f} // LINK Y W.R.T COM_Y
    ,
    {0.02316770f, -0.00205135f, -0.00581720f, -0.00205135f, 0.02061438f,
     0.00627358f, -0.00581720f, 0.00627358f, 0.02484097f} // LINK X W.R.T COM_X
    ,
    {0.10886020f, 0.00001273f, 0.00088705f, 0.00001273f, 0.02644452f,
     0.00000809f, 0.00088705f, 0.00000809f,
     0.09345014f} // LINK_PITCH W.R.T COM_PITCH
    ,
    {0.00153790, -0.00000106, 0.00003310, -0.00000106, 0.00213092, 0.00000026,
     0.00003310, 0.00000026, 0.00126543} // LINK_ROLL W.R.T COM_ROLL
    ,
    {0.00031947f, -0.00002243f, -0.00000878f, -0.00002243f, 0.00032862f,
     0.00000745f, -0.00000878f, 0.00000745f,
     0.00027962f} // LINK_YAW W.R.T COM_YAW
    ,
    {0.00045218f, -0.00001660f, 0.00011638f, -0.00001660f, 0.00284161f,
     -0.00000098f, 0.00011638f, -0.00000098f,
     0.00296033f} // LINK_PEDAL W.R.T IN COM_FS
};

#else
const float d6 = 0.0786f;
//! Links Inertial Parameters

const float LINKS_COM[NB_LINKS][NB_CART_AXIS] = {
    {0.06506077f, 0.23761375f, 0.06685000f},
    {0.02817123f, -0.07585496f, -0.00969366f},
    {-0.15754146f, 0.02306281f, 0.06946538f},
    {-0.00593072f, -0.00000124f, -0.00156324f},
    {0.03719409f, -0.00002562f, -0.00002217f},
    {0.00000000f, 0.00000000f, 0.07495233f},
    {-0.03295545f, 0.00021567f, 0.02639184f}};

const float LINKS_MASS[NB_LINKS] = {
    6.20696344f, 1.86377694f, 3.45202011f, 6.40922346f,
    1.17868723f, 0.09770847f, 0.37200000f};

const float mod_inertia_pitch =
    0.09345014f - 0.09345014f - 0.00213092f - 0.00004184f -
    (0.00335913f + LINKS_MASS[6] * d6 * d6) + 0.2521f;

const float mod_inertia_roll = 0.00126543f - 0.00126543f - 0.00003916f -
                                      (0.00049690f + LINKS_MASS[6] * d6 * d6) +
                                      0.1831f;

const float mod_inertia_yaw =
    0.00000699f - 0.00000699f - 0.00351089f + 0.1867f;

const float LINKS_MOMENT_OF_INERTIAS[NB_LINKS][NB_CART_AXIS *
                                                      NB_CART_AXIS] = {
    {0.16517950f, 0.01377293f, -0.00179536f, 0.01377293f, 0.15434303f,
     0.01782405f, -0.00179536f, 0.01782405f,
     0.29648455f} // LINK BASE  W.R.T COM_BASE
    ,
    {0.00343968f, 0.00007319f, 0.00001718f, 0.00007319f, 0.06820557f,
     0.00085107f, 0.00001718f, 0.00085107f, 0.06938224f} // LINK Y W.R.T COM_Y
    ,
    {0.02316770f, -0.00205135f, -0.00581720f, -0.00205135f, 0.02061438f,
     0.00627358f, -0.00581720f, 0.00627358f, 0.02484097f} // LINK X W.R.T COM_X
    ,
    {0.10886020f, 0.00001273f, 0.00088705f, 0.00001273f, 0.02644452f,
     0.00000809f, 0.00088705f, 0.00000809f,
     mod_inertia_pitch} // LINK_PITCH W.R.T COM_PITCH
    ,
    {0.00153790f, -0.00000106f, 0.00003310f, -0.00000106f, 0.00213092f,
     0.00000026f, 0.00003310f, 0.00000026f,
     mod_inertia_roll} // LINK_ROLL W.R.T COM_ROLL
    ,
    {0.00004184f, -0.00000023f, 0.00000000f, -0.00000023f, 0.00003916f,
     0.00000000f, 0.00000000f, 0.00000000f,
     mod_inertia_yaw} // LINK_YAW W.R.T COM_YAW
    ,
    {0.00049690f, -0.00001989f, 0.00016983f, -0.00001989f, 0.00335913f,
     -0.00000136f, 0.00016983f, -0.00000136f,
     0.00351089f} // LINK_PEDAL W.R.T IN COM_FS
};

#endif
const float d7 = 0.023f;
const float r8 = 0.209f;

#endif // DEFINITIONS_MODEL_H