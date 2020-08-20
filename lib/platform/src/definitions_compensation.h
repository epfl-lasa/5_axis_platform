#ifndef DEFINITIONS_COMPENSATION_H
#define DEFINITIONS_COMPENSATION_H

#include "definitions_main.h"
#include "definitions_control.h"
#include "LP_Filter.h"
#include "MatLP_Filter.h"
#include "macros_compensation_inertia.h"
#include "macros_compensation_coriolis.h"

//! Type of compensation. Matricial: computes matrices of the dynamic model, Equations: Hard-Coded equations pre-computed from Matlab
#define COMP_MATRICIAL 0
#define COMP_EQUATIONS 1
#define COMPENSATION_TYPE COMP_EQUATIONS

//! Exact Model Compensation | Inverse Dynamics
enum compensationComp {
  COMP_GRAVITY,
  COMP_VISC_FRICTION,
  COMP_INERTIA,
  COMP_CORIOLIS,
  COMP_FORCE_SENSOR,
  COMP_DRY_FRICTION,
  NB_COMPENSATION_COMP
};

//! Dry friction
#define NB_STICTION_AXIS 2
#define NB_SIGN_COMP 2
#define NEG_FRIC 0
#define POS_FRIC 1
#define MID_FRIC 3

const int COMPENSATION_COMP[] = {1, 0, 1, 1, 0, 1}; // gravity, viscous, inertia, coriolis, force sensor, dry

const float SPEED_THRESHOLD[NB_AXIS] = {0.010f, 0.010f, 0.09f * DEG_TO_RAD,
                                        0.12f * DEG_TO_RAD, 0.12f * DEG_TO_RAD};
                                        
//! Viscous Friction

float const VISCOUS_K[NB_AXIS] = {0.0f, 0.0f, 0.8f*46.0734f * DEG_TO_RAD,
                                  0.0f * DEG_TO_RAD, 0.0f * DEG_TO_RAD}; //62.3174f

// float const EMPIRICAL_INERTIA_K[NB_AXIS] = {13.6178f,13.7704f, 0.2521f,
//                                   0.1831f, 0.1867f};

//! Effort Limits

float const VISC_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{0.0f, 0.0f , -0.6f, -0.5f, -0.5f}, {0.0f, 0.0f, 0.5f, 0.5f, 0.5f}};
float const GRAVITY_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{0.0f, 0.0f, -2.0f, -2.0f, -2.0f}, {0.0f, 0.0f, 2.0f, 2.0f, 2.0f}};
float const INERTIA_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-10.0f,-10.0f,-0.5f,-0.5f,-0.5f}, {10.0f, 10.0f, 0.5f, 0.5f, 0.5f}};
float const CORIOLIS_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-10.0f,-10.0f,-1.0f,-1.0f,-1.0f}, {10.0f, 10.0f, 1.0f, 1.0f, 1.0f}};
float const DRY_EFFORT_LIMS[NB_SIGN_COMP][NB_LIMS][NB_AXIS] = {{{-16.0498f,-8.55883f,0.0f,0.0f,0.0f}, { -3.10896f, -1.47001f, 0.0f, 0.0f, 0.0f}},
                                                              {{1.90903f,0.875992f,0.0f,0.0f,0.0f},{15.5236f, 6.60670f, 0.0f, 0.0f, 0.0f}}};
float const FS_EFFORT_LIMS[NB_LIMS][NB_AXIS] =  {{-17.0f, -15.0f , -2.0f, -2.0f, -2.0f}, {17.0f, 15.0f, 2.0f, 2.0f,2.0f}};                                                             


//! Coriolis

#define CORIOLIS_KRONECKER 0
#define CORIOLIS_TEMPORAL 1
#define CORIOLIS_DEV_STRATEGY CORIOLIS_TEMPORAL


// RIGHT_PLATFORM

// Variables of Quadratic Regression - DRY FRICTION

#if (PLATFORM_ID==RIGHT_PLATFORM)
const float BETAS_QR_Y[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = 
{
    {1.6298f, 0.1197f, -0.1719f, -0.0223f, 0.1983f, //! BETAS NEG_FRIC Y
        -0.9100f, 0.0202f, 0.0566f, -0.1002f, -0.1577f},
    {0.0843f, 0.1095f, 0.1773f, -0.1787f, -0.0708f, //! BETAS POS_FRIC Y
        1.4363f, 0.0936f, -0.0854f, 0.0247f, 0.1376f},
};

const float MEAN_QR_Y[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = 
{
    {0.0613f, 0.0094f, 0.0325f, -0.0148f, 0.0062f, //! MEANS NEG_FRIC Y
        0.0045f,  0.0022f, 0.0307f, 0.0304f, 0.0533f},
    {0.0613f, 0.0094f, 0.0325f, -0.0148f, 0.0062f, //! MEANS POS_FRIC Y
        0.0045f, 0.0022f, 0.0307f, 0.0304f, 0.0533f}
};

const float STDINV_QR_Y[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = 
{
    {35.7143f, 21.8818f, 5.7870f, 5.7372f, 4.3178f, //! MEANS NEG_FRIC Y
        370.3704f, 416.6667f, 32.4675f, 40.1606f, 23.4742f},
    {25.3807f, 21.8818f, 5.7971f, 5.7372f, 4.3178f, //! MEANS POS_FRIC Y
        555.5556f, 416.6667f, 32.6797f, 40.1606f, 23.4742f}
};

const float BIAS_QR_Y[NB_SIGN_COMP] = {-8.18028f,10.5657f};


const float BETAS_QR_X[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = 
{
    {0.5090f, 0.1000f, -0.1961f, -0.2133f, 0.6077f, //! BETAS NEG_FRIC X
        0.3717f, -0.3332f, 0.1320f, -0.2567f, -0.0420},
    {-0.3858f, 1.6894f, -0.0671f, -0.1247f, -0.4456f, //! BETAS POS_FRIC X
        -0.4280f, 0.7966f, -0.0831f, -0.1320f, 0.3302f},
};

const float MEAN_QR_X[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = 
{
    {-0.0143f, -0.0139f, 0.0623f, -0.0613f, 0.0552f, //! MEANS NEG_FRIC X
    0.0014f, 0.0011f, 0.0305f, 0.0251f, 0.0312},
    {-0.0143f, -0.0147f, 0.0623f, -0.0613f, 0.0552f, //! MEANS POS_FRIC X
     0.0014f, 0.0011f, 0.0305f, 0.0251f, 0.0312   }
};

const float STDINV_QR_X[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = 
{
    {28.5714f, 33.6700f, 6.1275f, 6.8306f, 5.9595f, //! STD NEG_FRIC X
     666.6667f, 833.3333f, 38.1679f, 37.3134f, 28.0112f},
    {28.5714f, 33.3333f, 6.1275f, 6.8306f, 5.9595f, //! STD POS_FRIC X
     666.6667f, 833.3333f, 38.1679f, 37.3134f, 28.0112f}
};

const float BIAS_QR_X[NB_SIGN_COMP] = {-4.64060f,3.17075f};

#else

// TODO: UPDATE THIS MODEL CORRECTLY FOR THE LEFT PLATFORM

const float BETAS_QR_Y[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = {
    {1.6298f, 0.1197f, -0.1719f, -0.0223f, 0.1983f, //! BETAS NEG_FRIC Y
     -0.9100f, 0.0202f, 0.0566f, -0.1002f, -0.1577f},
    {0.0843f, 0.1095f, 0.1773f, -0.1787f, -0.0708f, //! BETAS POS_FRIC Y
     1.4363f, 0.0936f, -0.0854f, 0.0247f, 0.1376f},
};

const float MEAN_QR_Y[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = {
    {0.0613f, 0.0094f, 0.0325f, -0.0148f, 0.0062f, //! MEANS NEG_FRIC Y
     0.0045f, 0.0022f, 0.0307f, 0.0304f, 0.0533f},
    {0.0613f, 0.0094f, 0.0325f, -0.0148f, 0.0062f, //! MEANS POS_FRIC Y
     0.0045f, 0.0022f, 0.0307f, 0.0304f, 0.0533f}};

const float STDINV_QR_Y[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = {
    {35.7143f, 21.8818f, 5.7870f, 5.7372f, 4.3178f, //! MEANS NEG_FRIC Y
     370.3704f, 416.6667f, 32.4675f, 40.1606f, 23.4742f},
    {25.3807f, 21.8818f, 5.7971f, 5.7372f, 4.3178f, //! MEANS POS_FRIC Y
     555.5556f, 416.6667f, 32.6797f, 40.1606f, 23.4742f}};

const float BIAS_QR_Y[NB_SIGN_COMP] = {-8.18028f, 10.5657f};

const float BETAS_QR_X[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = {
    {0.5090f, 0.1000f, -0.1961f, -0.2133f, 0.6077f, //! BETAS NEG_FRIC X
     0.3717f, -0.3332f, 0.1320f, -0.2567f, -0.0420},
    {-0.3858f, 1.6894f, -0.0671f, -0.1247f, -0.4456f, //! BETAS POS_FRIC X
     -0.4280f, 0.7966f, -0.0831f, -0.1320f, 0.3302f},
};

const float MEAN_QR_X[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = {
    {-0.0143f, -0.0139f, 0.0623f, -0.0613f, 0.0552f, //! MEANS NEG_FRIC X
     0.0014f, 0.0011f, 0.0305f, 0.0251f, 0.0312},
    {-0.0143f, -0.0147f, 0.0623f, -0.0613f, 0.0552f, //! MEANS POS_FRIC X
     0.0014f, 0.0011f, 0.0305f, 0.0251f, 0.0312}};

const float STDINV_QR_X[NB_SIGN_COMP][NB_AXIS + NB_AXIS] = {
    {28.5714f, 33.6700f, 6.1275f, 6.8306f, 5.9595f, //! STD NEG_FRIC X
     666.6667f, 833.3333f, 38.1679f, 37.3134f, 28.0112f},
    {28.5714f, 33.3333f, 6.1275f, 6.8306f, 5.9595f, //! STD POS_FRIC X
     666.6667f, 833.3333f, 38.1679f, 37.3134f, 28.0112f}};

const float BIAS_QR_X[NB_SIGN_COMP] = {-4.63060f, 3.17075f};

#endif


#endif // DEFINITIONS_COMPENSATION_H