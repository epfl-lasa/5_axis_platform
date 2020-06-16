#ifndef DEFINITIONS_SECURITY_H
#define DEFINITIONS_SECURITY_H

#include "definitions_main.h"
#include "definitions_model.h"
#include "definitions_hardware.h"

//! Security Variables

const float WS_LIMITS[NB_AXIS] = {Y_RANGE / 2.0, X_RANGE / 2.0,
                                  PITCH_RANGE / 2.0f, ROLL_RANGE / 2.0f,
                                  YAW_RANGE / 2.0f};

const float WS_RANGE[NB_AXIS] = {Y_RANGE, X_RANGE, PITCH_RANGE, ROLL_RANGE,
                                 YAW_RANGE};

const float C_WS_LIMIT_Y = (Y_RANGE / 2.0f) * 0.8f;
const float C_WS_LIMIT_X = (X_RANGE / 2.0f) * 0.8f;
const float C_WS_LIMIT_PITCH = (PITCH_RANGE / 2.0f) * 0.8f;
const float C_WS_LIMIT_ROLL = (ROLL_RANGE / 2.0f) * 1.0f;
const float C_WS_LIMIT_YAW = (YAW_RANGE / 2.0f) * 0.8f;

const float C_WS_LIMITS[NB_AXIS] = {C_WS_LIMIT_Y, C_WS_LIMIT_X,
                                    C_WS_LIMIT_PITCH, C_WS_LIMIT_ROLL,
                                    C_WS_LIMIT_YAW};

// const float ADC_EFFORT_BIAS_RIGHT[NB_AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0};

const float EFFORT_LIMIT_DEFAULT_Y =
    (TORQUE_CONSTANT_Y / 1000.0f) * (MAX_CURRENT_Y)*Y_TRANSMISSION;
const float EFFORT_LIMIT_DEFAULT_X =
    (TORQUE_CONSTANT_X / 1000.0f) * (MAX_CURRENT_X)*X_TRANSMISSION;
const float EFFORT_LIMIT_DEFAULT_PITCH= (TORQUE_CONSTANT_PITCH_ROLL_YAW/1000.0f)*
    (MAX_CURRENT_PITCH_ROLL_YAW) * PITCH_REDUCTION_R;
const float EFFORT_LIMIT_DEFAULT_ROLL= (TORQUE_CONSTANT_PITCH_ROLL_YAW/1000.0f)*
    (MAX_CURRENT_PITCH_ROLL_YAW) * ROLL_YAW_REDUCTION_R;
const float EFFORT_LIMIT_DEFAULT_YAW= (TORQUE_CONSTANT_PITCH_ROLL_YAW/1000.0f) *
    (MAX_CURRENT_PITCH_ROLL_YAW) * ROLL_YAW_REDUCTION_R;

const float MAX_EFFORT[] = {EFFORT_LIMIT_DEFAULT_Y ,EFFORT_LIMIT_DEFAULT_X, 
                            EFFORT_LIMIT_DEFAULT_PITCH,EFFORT_LIMIT_DEFAULT_ROLL, 
                            EFFORT_LIMIT_DEFAULT_YAW};


const float SAFETY_MAX_EFFORTS[NB_AXIS] = { EFFORT_LIMIT_DEFAULT_Y, EFFORT_LIMIT_DEFAULT_X,
                                            0.5f * EFFORT_LIMIT_DEFAULT_PITCH , 0.5f * EFFORT_LIMIT_DEFAULT_ROLL, 0.5f * EFFORT_LIMIT_DEFAULT_YAW};

const float SAFETY_MAX_INIT[] = {SAFETY_MAX_EFFORTS[Axis::Y],
                                    SAFETY_MAX_EFFORTS[Axis::X],
                                    0.4f * SAFETY_MAX_EFFORTS[Axis::PITCH],
                                    0.4f * SAFETY_MAX_EFFORTS[Axis::ROLL],
                                    0.4f * SAFETY_MAX_EFFORTS[Axis::YAW]};

const float SPEED_LIMIT_Y = Y_RANGE / 0.5;
const float SPEED_LIMIT_X = X_RANGE / 0.5; //! s^-1
const float SPEED_LIMIT_PITCH = PITCH_RANGE / 0.5;
const float SPEED_LIMIT_ROLL = ROLL_RANGE / 0.5;
const float SPEED_LIMIT_YAW = YAW_RANGE / 0.5;

#endif // DEFINITIONS_SECURITY_H