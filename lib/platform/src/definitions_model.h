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
    
    
    const float r3 = 0.305f;
    const float d6_no_fs = 0.0786f;
    const float d6 = 0.1366f;
    const float d7 = 0.023f;
    const float r8 = 0.209f;

    const float LINKS_MASS[NB_LINKS] = {6.20696344f, 1.86377694f, 3.59020320f,
                                        6.40922346f, 1.17868723f, 0.46082997f,
                                        0.30000000f}; //0.37200000f

    // const float LINKS_MASS[NB_LINKS] = {6.20696344f, 1.86377694f, 3.59020320f,
    //                                     6.40922346f, 1.17868723f, 0.39114648f,
    //                                     0.30000000f}; //0.37200000f                                        

#if (PLATFORM_ID == RIGHT_PLATFORM)
    //! Geometric Parameters
        //! Links Inertial Parameters

        const float LINKS_COM[NB_LINKS][NB_CART_AXIS] = {
            {0.06378538f, 0.23761375f, 0.06685000f},    // Base
            {0.02817123f, -0.07585496f, -0.00969366f},  // Y
            {-0.15354810f, 0.02479498f, 0.06831312f},        // X
            {-0.00593072f, -0.00000124f, -0.00156324f}, // PITCH
            { 0.03719409f, -0.00002562f, -0.00002217f},  // ROLL
            { 0.00004367f, -0.00000225f,  0.10098165f},   // YAW-ALU
            // { 0.00002617f, -0.00000265f,	0.10239610f},   // YAW
            {-0.03198766f, 0.00021421f, 0.02133190f}};  // PEDAL


        const float IZZ_pitch_raw_no_fs = 0.09345014f + PITCH_REDUCTION_R*PITCH_REDUCTION_R*PITCH_ROTOR_INERTIA; //! Yes, its the same
        const float IZZ_roll_raw_no_fs = 0.00126543f + ROLL_YAW_REDUCTION_R*ROLL_YAW_REDUCTION_R*ROLL_YAW_ROTOR_INERTIA;  //! Yes, its the same
        // const float IZZ_yaw_raw_fs = 0.00017310f + ROLL_YAW_REDUCTION_R*ROLL_YAW_REDUCTION_R*ROLL_YAW_ROTOR_INERTIA;
        const float IZZ_yaw_raw_fs = 0.00022433f + ROLL_YAW_REDUCTION_R*ROLL_YAW_REDUCTION_R*ROLL_YAW_ROTOR_INERTIA; // ALU

        const float LINKS_MOMENT_OF_INERTIAS[NB_LINKS][NB_CART_AXIS * NB_CART_AXIS] = {
            { 0.16517950f, 0.01377594f, -0.00180234f, 
              0.01377594f, 0.15491411f,  0.01782405f, 
             -0.00180234f, 0.01782405f,  0.01782405f} // LINK BASE  W.R.T COM_BASE
            ,
            {0.00343968f, 0.00002371f, 0.00000365f,
             0.00002371f, 0.06755732f, 0.00085107f, 
             0.00000365f, 0.00085107f, 0.06873399f} // LINK Y W.R.T COM_Y
            ,
            { 0.02391576f,-0.00245204f,-0.00647808f,
             -0.00245204f, 0.02208247f, 0.00612089f,
             -0.00647808f, 0.00612089f, 0.02573858f}  // LINK X W.R.T COM_X
            ,
            {0.10886020f, 0.00001273f, 0.00088705f, 
             0.00001273f, 0.02644452f, 0.00000809f, 
             0.00088705f, 0.00000809f,  IZZ_pitch_raw_no_fs} // LINK_PITCH W.R.T COM_PITCH
            ,
            {0.00153790f, -0.00000106f, 0.00003310f, 
            -0.00000106f,  0.00213092f, 0.00000026f, 
             0.00003310f,  0.00000026f, IZZ_roll_raw_no_fs} // LINK_ROLL W.R.T COM_ROLL
            ,
            // {0.00028064f, 0.00000000f, 0.00000015f,
            //  0.00000000f, 0.00027286f, 0.00000000f,
            //  0.00000015f, 0.00000000f, IZZ_yaw_raw_fs,
            // } // LINK_YAW W.R.T COM_YAW
            { 0.00031468f,	0.00000000f, 0.00000015f,
              0.00000000f,	0.00030286f, -0.00000001f,
              0.00000015f,	-0.00000001f, IZZ_yaw_raw_fs,
            } // LINK_YAW W.R.T COM_YAW ALU
            ,
            { 0.00044045f , -0.00001582f , 0.00012683f, 
             -0.00001582f ,  0.00271438f ,-0.00000105f, 
              0.00012683f , -0.00000105f , 0.00282256f} // LINK_PEDAL W.R.T IN COM_FS
        };
#else
    //! Links Inertial Parameters

        const float LINKS_COM[NB_LINKS][NB_CART_AXIS] = {
            { -0.06393533f,  0.23761375f, 0.06685000f},    // Base
            { -0.02817123f, -0.07585496f, 0.00969366f},  // Y
            { -0.15354810f, -0.02479498f,-0.06831312f},  // X
            { -0.00593072f, -0.00000124f,-0.00156324f}, // PITCH
            {  0.03719409f, -0.00002562f,-0.00002217f},  // ROLL
            { -0.00004367f, 0.00000225f,  0.10098165f},   // YAW-ALU
            // { -0.00002617f,  0.00000265f, 0.10239610f},   // YAW
            { -0.03198766f,  0.00021421f, 0.02133190f}};  // PEDAL

        const float IZZ_pitch_raw_no_fs = 0.09345014f + PITCH_REDUCTION_R*PITCH_REDUCTION_R*PITCH_ROTOR_INERTIA; //! Yes, its the same
        const float IZZ_roll_raw_no_fs = 0.00126543f + ROLL_YAW_REDUCTION_R*ROLL_YAW_REDUCTION_R*ROLL_YAW_ROTOR_INERTIA;  //! Yes, its the same
        // const float IZZ_yaw_raw_fs = 0.00017310f + ROLL_YAW_REDUCTION_R*ROLL_YAW_REDUCTION_R*ROLL_YAW_ROTOR_INERTIA;
        const float IZZ_yaw_raw_fs = 0.00022433f + ROLL_YAW_REDUCTION_R*ROLL_YAW_REDUCTION_R*ROLL_YAW_ROTOR_INERTIA; // ALU


        const float LINKS_MOMENT_OF_INERTIAS[NB_LINKS][NB_CART_AXIS * NB_CART_AXIS] = {
            {  0.16517950f, -0.01377594f,  0.00180234f,
              -0.01377594f,  0.15491411f,  0.01782405f,
               0.00180234f,  0.01782405f,  0.29705563f} // LINK BASE  W.R.T COM_BASE
            ,
            { 0.00343968f, -0.00002371f,  0.00000365f,
             -0.00002371f,  0.06755732f, -0.00085107f, 
              0.00000365f, -0.00085107f, -0.06873399f} // LINK Y W.R.T COM_Y
            ,
            { 0.02391576f, 0.00245204f,	0.00647808f,
              0.00245204f, 0.02208247f,	0.00612089f,
              0.00647808f, 0.00612089f,	0.02573858f}  // LINK X W.R.T COM_X
            ,
            { 0.10886020f, -0.00001273f, -0.00088705f, 
             -0.00001273f,  0.02644452f,  0.00000809f, 
             -0.00088705f,  0.00000809f,  IZZ_pitch_raw_no_fs} // LINK_PITCH W.R.T COM_PITCH
            ,
            {0.00153790f, -0.00000106f, 0.00003310f, 
            -0.00000106f, 0.00213092f,  0.00000026f, 
             0.00003310f, 0.00000026f,   IZZ_roll_raw_no_fs} // LINK_ROLL W.R.T COM_ROLL
            ,
            // { 0.00028064f, 0.00000000f, -0.00000015f,
            //   0.00000000f, 0.00027286f,  0.00000000f,
            //  -0.00000015f, 0.00000000f,  IZZ_yaw_raw_fs,
            // } // LINK_YAW W.R.T COM_YAW
            { 0.00031468f,	0.00000000f, -0.00000015f,
              0.00000000f,	0.00030286f, 0.00000001f,
              -0.00000015f,	0.00000001f, IZZ_yaw_raw_fs,
            } // LINK_YAW W.R.T COM_YAW ALU
            ,
            { 0.00044045f , -0.00001582f , 0.00012683f, 
             -0.00001582f ,  0.00271438f ,-0.00000105f, 
              0.00012683f , -0.00000105f , 0.00282256f} // LINK_PEDAL W.R.T IN COM_FS
        };

#endif

#endif // DEFINITIONS_MODEL_H