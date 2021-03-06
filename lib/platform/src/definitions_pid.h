#ifndef DEFINITIONS_PID_H
#define DEFINITIONS_PID_H

#include "definitions_main.h"
#include "definitions_control.h"

#define KP 0
#define KI 1
#define KD 2

//! Filters
const float POS_PID_FILTER_GAINS[NB_AXIS] = {0.6f, 0.6f, 0.6f, 0.6f, 0.6f};
const float VEL_PID_FILTER_GAINS[NB_AXIS] = {0.6f, 0.6f, 0.6f, 0.6f, 0.6f};
const float FS_PID_FILTER_GAINS[NB_AXIS] = {0.85f, 0.85f, 0.8f, 0.8f, 0.8f};
const float ALPHA_RCM_PID_FILTER = 0.8f;

//! List of Gains
//******************************HOMING********************************

    #if (PLATFORM_ID==LEFT_PLATFORM_ID) //! TODO: Set for the left platform

        const float SPEED_D_HOMING_Y = 1.0f * 0.3f;                                                 //[m/s]
        const float SPEED_D_HOMING_X = -1.0f * 0.3f;                                                 //[m/s]
        const float SPEED_D_HOMING_PITCH  = 200 * DEG_TO_RAD * 0.5f;                              //[deg/s             
    #else 
        
        const float SPEED_D_HOMING_Y = 1.0f * 0.3f;                                                 //[m/s]
        const float SPEED_D_HOMING_X = 1.0f * 0.3f;                                                 //[m/s]                    
        const float SPEED_D_HOMING_PITCH  = -200 * DEG_TO_RAD * 0.5f;                              //[deg/s]
    #endif

        const float HOMING_KP_SPEED_Y = 1000.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m]   
        const float HOMING_KP_SPEED_X = 1000.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m]
        const float HOMING_KP_SPEED_PITCH = 1000.0f * SCALE_GAINS_ANGULAR_SPEED;           //[Nm.s/deg]

        const float HOMING_KI_SPEED_Y = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m.s]     
        const float HOMING_KI_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m.s]
        const float HOMING_KI_SPEED_PITCH = 1000.0f * SCALE_GAINS_ANGULAR_SPEED;            //[N.s/deg.s]    

const float SPEED_PID_GAINS_DEFAULT[3][NB_AXIS] = {
            {HOMING_KP_SPEED_Y, HOMING_KP_SPEED_X, HOMING_KP_SPEED_PITCH,
             0.0f, 0.0f},
            {HOMING_KI_SPEED_Y, HOMING_KI_SPEED_X, HOMING_KI_SPEED_PITCH,
             0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};


const float FS_PID_GAINS_DEFAULT[3][NB_AXIS] = {
            {2.0f, 1.0f, 2.0f,2.0f, 3.0f},
            {2.0f, 1.0f, 2.0f,2.0f, 3.0f},
            {0.0f, 0.0f, 0.0f,0.0f, 0.0f}}; // Y, X, PITCH, ROLL, YAW

const float RCM_PID_GAINS_DEFAULT[3] = {0.0f, 0.0f, 0.0f};

const float RCM_POS_DEFAULT[NB_CART_AXIS] = {0.0f, 0.0f, 0.5446f};
//*************************GOTO******************************************************

#if (PLATFORM_ID == LEFT_PLATFORM_ID)

const float GT_KP_POSITION_Y = 5000.0f * SCALE_GAINS_LINEAR_POSITION; //[N/m]
const float GT_KP_POSITION_X = 5000.0f * SCALE_GAINS_LINEAR_POSITION; //[N/m]
const float GT_KP_POSITION_PITCH =    10000.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm/deg]
const float GT_KP_POSITION_ROLL =    5000.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm/deg]
const float GT_KP_POSITION_YAW =    5000.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm/deg]

const float GT_KI_POSITION_Y = 10000.0f * SCALE_GAINS_LINEAR_POSITION; //[N/m.s]
const float GT_KI_POSITION_X = 10000.0f * SCALE_GAINS_LINEAR_POSITION; //[N/m.s]
const float GT_KI_POSITION_PITCH =    10000.0f * SCALE_GAINS_ANGULAR_POSITION; //[N/deg.s]
const float GT_KI_POSITION_ROLL =    10000.0f * SCALE_GAINS_ANGULAR_POSITION; //[N/deg.s]
const float GT_KI_POSITION_YAW =    10000.0f * SCALE_GAINS_ANGULAR_POSITION; //[N/deg.s]

const float GT_KD_POSITION_Y = 10.0f * SCALE_GAINS_LINEAR_POSITION; //[N.s/m]
const float GT_KD_POSITION_X = 10.0f * SCALE_GAINS_LINEAR_POSITION; //[N.s/m]
const float GT_KD_POSITION_PITCH =    30.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm.s/deg]
const float GT_KD_POSITION_ROLL =    35.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm.s/deg]
const float GT_KD_POSITION_YAW =    35.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm.s/deg]

#else //! TODO TUNE FOR RIGHT_PLATFORM_ID     

        const float GT_KP_POSITION_Y =  5000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m]
        const float GT_KP_POSITION_X =  5000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m]
        const float GT_KP_POSITION_PITCH =  10000.0f * SCALE_GAINS_ANGULAR_POSITION;              //[Nm/deg]
        const float GT_KP_POSITION_ROLL =  5000.0f * SCALE_GAINS_ANGULAR_POSITION;               //[Nm/deg]
        const float GT_KP_POSITION_YAW = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;                 //[Nm/deg]
            
        const float GT_KI_POSITION_Y =  10000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m.s]
        const float GT_KI_POSITION_X =  10000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m.s]
        const float GT_KI_POSITION_PITCH =  10000.0f * SCALE_GAINS_ANGULAR_POSITION;              //[N/deg.s]   
        const float GT_KI_POSITION_ROLL =  10000.0f * SCALE_GAINS_ANGULAR_POSITION;               //[N/deg.s]   
        const float GT_KI_POSITION_YAW = 10000.0f * SCALE_GAINS_ANGULAR_POSITION;                 //[N/deg.s]
            
        const float GT_KD_POSITION_Y =  10.0f * SCALE_GAINS_LINEAR_POSITION;                      //[N.s/m]
        const float GT_KD_POSITION_X =  10.0f * SCALE_GAINS_LINEAR_POSITION;                      //[N.s/m]                
        const float GT_KD_POSITION_PITCH =  30.0f * SCALE_GAINS_ANGULAR_POSITION;                 //[Nm.s/deg]
        const float GT_KD_POSITION_ROLL =  35.0f * SCALE_GAINS_ANGULAR_POSITION;                  //[Nm.s/deg]
        const float GT_KD_POSITION_YAW = 35.0f * SCALE_GAINS_ANGULAR_POSITION;                    //[Nm.s/deg]
#endif
const float POS_PID_GAINS_DEFAULT[3][NB_AXIS] = {
            {GT_KP_POSITION_Y, GT_KP_POSITION_X, GT_KP_POSITION_PITCH,
             GT_KP_POSITION_ROLL, GT_KP_POSITION_YAW},
            {GT_KI_POSITION_Y, GT_KI_POSITION_X, GT_KI_POSITION_PITCH,
             GT_KI_POSITION_ROLL, GT_KI_POSITION_YAW},
            {GT_KD_POSITION_Y, GT_KD_POSITION_X, GT_KD_POSITION_PITCH,
             GT_KD_POSITION_ROLL, GT_KD_POSITION_YAW}};
        //*************************W_CONSTRAINS****AKA.VIRTUAL_WALLS****************************

#if (PLATFORM_ID == LEFT_PLATFORM_ID)

        const float C_WS_KP_POSITION_Y =  1000.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m]
        const float C_WS_KP_POSITION_X =  1000.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m]                              
        const float C_WS_KP_POSITION_PITCH =  1000.0f * SCALE_GAINS_ANGULAR_POSITION;       //[Nm/deg]
        const float C_WS_KP_POSITION_ROLL =  5000.0f * SCALE_GAINS_ANGULAR_POSITION;        //[Nm/deg]
        const float C_WS_KP_POSITION_YAW = 1000.0f * SCALE_GAINS_ANGULAR_POSITION;          //[Nm/deg]

        const float C_WS_KD_POSITION_Y =  1.0f* SCALE_GAINS_LINEAR_POSITION;              //[N.s/m]
        const float C_WS_KD_POSITION_X =  0.0f* SCALE_GAINS_LINEAR_POSITION;              //[N.s/m]                
        const float C_WS_KD_POSITION_PITCH =  5.0f* SCALE_GAINS_ANGULAR_POSITION;          //[Nm.s/deg]
        const float C_WS_KD_POSITION_ROLL =  10.0f* SCALE_GAINS_ANGULAR_POSITION;           //[Nm.s/deg]
        const float C_WS_KD_POSITION_YAW = 5.0f* SCALE_GAINS_ANGULAR_POSITION;             //[Nm.s/deg]

           //***************************SHOULD-BE-ZERO****************/ 
                const float C_WS_KI_POSITION_Y =  0.0f * SCALE_GAINS_LINEAR_POSITION;      //[N/m.s]
                const float C_WS_KI_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;      //[N/m.s]                
                const float C_WS_KI_POSITION_PITCH =  0.0f * SCALE_GAINS_ANGULAR_POSITION;  //[Nm/deg.s]
                const float C_WS_KI_POSITION_ROLL =  0.0f * SCALE_GAINS_ANGULAR_POSITION;   //[Nm/deg.s]
                const float C_WS_KI_POSITION_YAW = 0.0f * SCALE_GAINS_ANGULAR_POSITION;     //[Nm/deg.s]


    #else

        const float C_WS_KP_POSITION_Y = 1000.0f * SCALE_GAINS_LINEAR_POSITION; //[N/m]
        const float C_WS_KP_POSITION_X = 1000.0f * SCALE_GAINS_LINEAR_POSITION;            //[N/m]                              
        const float C_WS_KP_POSITION_PITCH = 1000.0f * SCALE_GAINS_ANGULAR_POSITION;        //[Nm/deg]
        const float C_WS_KP_POSITION_ROLL = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;         //[Nm/deg]
        const float C_WS_KP_POSITION_YAW =1000.0f * SCALE_GAINS_ANGULAR_POSITION;           //[Nm/deg]

        const float C_WS_KD_POSITION_Y =  1.0f * SCALE_GAINS_LINEAR_POSITION;               //[N.s/m]
        const float C_WS_KD_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;               //[N.s/m]                
        const float C_WS_KD_POSITION_PITCH =  5.0f * SCALE_GAINS_ANGULAR_POSITION;           //[Nm.s/deg]
        const float C_WS_KD_POSITION_ROLL =  10.0f * SCALE_GAINS_ANGULAR_POSITION;            //[Nm.s/deg]
        const float C_WS_KD_POSITION_YAW = 5.0f * SCALE_GAINS_ANGULAR_POSITION;              //[Nm.s/deg]

        //***************************SHOULD-BE-ZERO****************/ 

            const float C_WS_KI_POSITION_Y =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m.s]
            const float C_WS_KI_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m.s]                
            const float C_WS_KI_POSITION_PITCH =  0.0f * SCALE_GAINS_ANGULAR_POSITION;       //[Nm/deg.s]
            const float C_WS_KI_POSITION_ROLL =  0.0f * SCALE_GAINS_ANGULAR_POSITION;        //[Nm/deg.s]
            const float C_WS_KI_POSITION_YAW = 0.0f * SCALE_GAINS_ANGULAR_POSITION;          //[Nm/deg.s]

    
    #endif
const float C_WS_PID_GAINS[3][NB_AXIS] = {
            {C_WS_KP_POSITION_Y, C_WS_KP_POSITION_X, C_WS_KP_POSITION_PITCH,
             C_WS_KP_POSITION_ROLL, C_WS_KP_POSITION_YAW},
            {C_WS_KI_POSITION_Y, C_WS_KI_POSITION_X, C_WS_KI_POSITION_PITCH,
             C_WS_KI_POSITION_ROLL, C_WS_KI_POSITION_YAW},
            {C_WS_KD_POSITION_Y, C_WS_KD_POSITION_X, C_WS_KD_POSITION_PITCH,
             C_WS_KD_POSITION_ROLL, C_WS_KD_POSITION_YAW}};


#endif //DEFINITIONS_PID_H
