#ifndef DEFINITIONS_PID_H
#define DEFINITIONS_PID_H

#include "definitions_main.h"
#include "definitions_control.h"


//! Filters
const float POS_PID_FILTER_GAINS[NB_AXIS] = {0.5f, 0.5f, 0.8f, 0.8f, 0.8f};
const float VEL_PID_FILTER_GAINS[NB_AXIS] = {0.5f, 0.5f, 0.8f, 0.8f, 0.8f};

//! List of Gains
//******************************HOMING********************************

    #if (PLATFORM_ID==LEFT_PLATFORM) //! TODO: Set for the left platform

        #define SPEED_D_HOMING_X -1.0                                                //[m/s]                         
        #define SPEED_D_HOMING_Y 1.0                                                 //[deg/s]
        const float SPEED_D_HOMING_PITCH = 300.0 * DEG_TO_RAD;                              //[deg/s]                             
        
        const float KP_HOMING_SPEED_Y = 2000.0f * SCALE_GAINS_LINEAR_SPEED;                  //[N.s/m]     
        const float KP_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                  //[N.s/m]     
        const float KP_HOMING_SPEED_PITCH = 1700.0f * SCALE_GAINS_ANGULAR_SPEED;             //[Nm.s/deg]     

        const float KI_HOMING_SPEED_Y = 2000.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m.s]     
        const float KI_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m.s]     
        const float KI_HOMING_SPEED_PITCH = 850.0f * SCALE_GAINS_ANGULAR_SPEED;             //[N.s/deg.s]     

    #else 

        #define SPEED_D_HOMING_Y 1.0                                                 //[m/s]
        #define SPEED_D_HOMING_X 1.0                                                 //[m/s]                    
        const float SPEED_D_HOMING_PITCH  = -300 * DEG_TO_RAD;                              //[deg/s]

        const float KP_HOMING_SPEED_Y = 1500.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m]   
        const float KP_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m]
        const float KP_HOMING_SPEED_PITCH = 1700.0f * SCALE_GAINS_ANGULAR_SPEED;           //[Nm.s/deg]

        const float KI_HOMING_SPEED_Y = 1000.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m.s]     
        const float KI_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m.s]
        const float KI_HOMING_SPEED_PITCH = 850.0f * SCALE_GAINS_ANGULAR_SPEED;            //[N.s/deg.s]    
    #endif


//*************************GOTO******************************************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        const float GT_KP_POSITION_Y = 5000.0f * SCALE_GAINS_LINEAR_POSITION;       //[N/m]
        const float GT_KP_POSITION_X = 5000.0f * SCALE_GAINS_LINEAR_POSITION;       //[N/m]
        const float GT_KP_POSITION_PITCH = 10000.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm/deg]
        const float GT_KP_POSITION_ROLL = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;   //[Nm/deg]
        const float GT_KP_POSITION_YAW = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;    //[Nm/deg]

        const float GT_KI_POSITION_Y = 5000.0f * SCALE_GAINS_LINEAR_POSITION;     //[N/m.s]
        const float GT_KI_POSITION_X = 5000.0f * SCALE_GAINS_LINEAR_POSITION;     //[N/m.s]
        const float GT_KI_POSITION_PITCH = 5000.0f * SCALE_GAINS_ANGULAR_POSITION; //[N/deg.s]
        const float GT_KI_POSITION_ROLL = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;  //[N/deg.s]
        const float GT_KI_POSITION_YAW = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;   //[N/deg.s]

        const float GT_KD_POSITION_Y = 10.0f * SCALE_GAINS_LINEAR_POSITION;      //[N.s/m]
        const float GT_KD_POSITION_X = 10.0f * SCALE_GAINS_LINEAR_POSITION;      //[N.s/m]
        const float GT_KD_POSITION_PITCH = 45.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm.s/deg]
        const float GT_KD_POSITION_ROLL = 35.0f * SCALE_GAINS_ANGULAR_POSITION;  //[Nm.s/deg]
        const float GT_KD_POSITION_YAW = 35.0f * SCALE_GAINS_ANGULAR_POSITION;   //[Nm.s/deg]

#else //! TODO TUNE FOR RIGHT_PLATFORM     

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

        //*************************W_CONSTRAINS****AKA.VIRTUAL_WALLS****************************

#if (PLATFORM_ID == LEFT_PLATFORM)

        const float C_WS_KP_POSITION_Y =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m]
        const float C_WS_KP_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m]                              
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

        const float C_WS_KP_POSITION_Y = 0.0f * SCALE_GAINS_LINEAR_POSITION;            //[N/m]
        const float C_WS_KP_POSITION_X = 0.0f * SCALE_GAINS_LINEAR_POSITION;            //[N/m]                              
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



#endif //DEFINITIONS_PID_H
