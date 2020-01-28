#ifndef DEFINITIONS_2_HH
#define DEFINITIONS_2_HH

#include <../../5_axis_platform/lib/platform/src/definitions.h>


#define SCALE_GAINS_LINEAR_POSITION 1
#define SCALE_GAINS_LINEAR_SPEED 1e-2f   
#define SCALE_GAINS_ANGULAR_POSITION 1e-4f
#define SCALE_GAINS_ANGULAR_SPEED 1e-5f


//******************************CONSTRUCTOR*****************************
    #define PLATFORM_SUBSCRIBER_NAME_LEFT "/FI_Input/Left"
    #define PLATFORM_PUBLISHER_NAME_LEFT "/FI_Output/Left"
    #define SERVICE_CHANGE_STATE_NAME_LEFT "update_left_state"
    #define SERVICE_CHANGE_CTRL_NAME_LEFT "update_left_controller"
    
    #define PLATFORM_SUBSCRIBER_NAME_RIGHT "/FI_Input/Right"
    #define PLATFORM_PUBLISHER_NAME_RIGHT "/FI_Output/Right"
    #define SERVICE_CHANGE_STATE_NAME_RIGHT "update_right_state"
    #define SERVICE_CHANGE_CTRL_NAME_RIGHT "update_right_controller"


    #if (PLATFORM_ID == LEFT_PLATFORM) 
        #define PLATFORM_SUBSCRIBER_NAME PLATFORM_SUBSCRIBER_NAME_LEFT
        #define PLATFORM_PUBLISHER_NAME PLATFORM_PUBLISHER_NAME_LEFT
        #define SERVICE_CHANGE_STATE_NAME SERVICE_CHANGE_STATE_NAME_LEFT
        #define SERVICE_CHANGE_CTRL_NAME SERVICE_CHANGE_CTRL_NAME_LEFT
    #else
        #define PLATFORM_SUBSCRIBER_NAME PLATFORM_SUBSCRIBER_NAME_RIGHT
        #define PLATFORM_PUBLISHER_NAME PLATFORM_PUBLISHER_NAME_RIGHT
        #define SERVICE_CHANGE_STATE_NAME SERVICE_CHANGE_STATE_NAME_RIGHT
        #define SERVICE_CHANGE_CTRL_NAME SERVICE_CHANGE_CTRL_NAME_RIGHT
    #endif



//******************************HOMING********************************

    #if (PLATFORM_ID==LEFT_PLATFORM) //! TODO: Set for the left platform

        #define SPEED_D_HOMING_X -1.0                                                //[m/s]                         
        #define SPEED_D_HOMING_Y 1.0                                                 //[deg/s]
        #define SPEED_D_HOMING_PITCH 300.0                                           //[deg/s]                             
        
        const float KP_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                  //[N.s/m]     
        const float KP_HOMING_SPEED_Y = 2000.0f * SCALE_GAINS_LINEAR_SPEED;                  //[N.s/m]     
        const float KP_HOMING_SPEED_PITCH = 1700.0f * SCALE_GAINS_ANGULAR_SPEED;             //[Nm.s/deg]     

        const float KI_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m.s]     
        const float KI_HOMING_SPEED_Y = 2000.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m.s]     
        const float KI_HOMING_SPEED_PITCH = 850.0f * SCALE_GAINS_ANGULAR_SPEED;             //[N.s/deg.s]     

    #else 

        #define SPEED_D_HOMING_X 1.0                                                 //[m/s]                    
        #define SPEED_D_HOMING_Y 1.0                                                 //[deg/s]
        #define SPEED_D_HOMING_PITCH -300                                            //[deg/s]

        const float KP_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                 //[N.s/m]
        const float KP_HOMING_SPEED_Y = 1500.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m]   
        const float KP_HOMING_SPEED_PITCH = 1700.0f * SCALE_GAINS_ANGULAR_SPEED;           //[Nm.s/deg]

        const float KI_HOMING_SPEED_X = 2500.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m.s]
        const float KI_HOMING_SPEED_Y = 1000.0f * SCALE_GAINS_LINEAR_SPEED;                //[N.s/m.s]     
        const float KI_HOMING_SPEED_PITCH = 850.0f * SCALE_GAINS_ANGULAR_SPEED;            //[N.s/deg.s]    
    #endif


//*************************GOTO******************************************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        const float GT_KP_POSITION_X = 5000.0f * SCALE_GAINS_LINEAR_POSITION;       //[N/m]
        const float GT_KP_POSITION_Y = 5000.0f * SCALE_GAINS_LINEAR_POSITION;       //[N/m]
        const float GT_KP_POSITION_PITCH = 10000.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm/deg]
        const float GT_KP_POSITION_ROLL = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;   //[Nm/deg]
        const float GT_KP_POSITION_YAW = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;    //[Nm/deg]

        const float GT_KI_POSITION_X = 5000.0f * SCALE_GAINS_LINEAR_POSITION;     //[N/m.s]
        const float GT_KI_POSITION_Y = 5000.0f * SCALE_GAINS_LINEAR_POSITION;     //[N/m.s]
        const float GT_KI_POSITION_PITCH = 5000.0f * SCALE_GAINS_ANGULAR_POSITION; //[N/deg.s]
        const float GT_KI_POSITION_ROLL = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;  //[N/deg.s]
        const float GT_KI_POSITION_YAW = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;   //[N/deg.s]

        const float GT_KD_POSITION_X = 10.0f * SCALE_GAINS_LINEAR_POSITION;      //[N.s/m]
        const float GT_KD_POSITION_Y = 10.0f * SCALE_GAINS_LINEAR_POSITION;      //[N.s/m]
        const float GT_KD_POSITION_PITCH = 45.0f * SCALE_GAINS_ANGULAR_POSITION; //[Nm.s/deg]
        const float GT_KD_POSITION_ROLL = 35.0f * SCALE_GAINS_ANGULAR_POSITION;  //[Nm.s/deg]
        const float GT_KD_POSITION_YAW = 35.0f * SCALE_GAINS_ANGULAR_POSITION;   //[Nm.s/deg]

#else //! TODO TUNE FOR RIGHT_PLATFORM     

        const float GT_KP_POSITION_X =  5000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m]
        const float GT_KP_POSITION_Y =  5000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m]
        const float GT_KP_POSITION_PITCH =  10000.0f * SCALE_GAINS_ANGULAR_POSITION;              //[Nm/deg]
        const float GT_KP_POSITION_ROLL =  5000.0f * SCALE_GAINS_ANGULAR_POSITION;               //[Nm/deg]
        const float GT_KP_POSITION_YAW = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;                 //[Nm/deg]
            
        const float GT_KI_POSITION_X =  10000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m.s]
        const float GT_KI_POSITION_Y =  10000.0f * SCALE_GAINS_LINEAR_POSITION;                   //[N/m.s]
        const float GT_KI_POSITION_PITCH =  10000.0f * SCALE_GAINS_ANGULAR_POSITION;              //[N/deg.s]   
        const float GT_KI_POSITION_ROLL =  10000.0f * SCALE_GAINS_ANGULAR_POSITION;               //[N/deg.s]   
        const float GT_KI_POSITION_YAW = 10000.0f * SCALE_GAINS_ANGULAR_POSITION;                 //[N/deg.s]
            
        const float GT_KD_POSITION_X =  10.0f * SCALE_GAINS_LINEAR_POSITION;                      //[N.s/m]                
        const float GT_KD_POSITION_Y =  10.0f * SCALE_GAINS_LINEAR_POSITION;                      //[N.s/m]
        const float GT_KD_POSITION_PITCH =  30.0f * SCALE_GAINS_ANGULAR_POSITION;                 //[Nm.s/deg]
        const float GT_KD_POSITION_ROLL =  35.0f * SCALE_GAINS_ANGULAR_POSITION;                  //[Nm.s/deg]
        const float GT_KD_POSITION_YAW = 35.0f * SCALE_GAINS_ANGULAR_POSITION;                    //[Nm.s/deg]

#endif

        //*************************W_CONSTRAINS****AKA.VIRTUAL_WALLS****************************

#if (PLATFORM_ID == LEFT_PLATFORM)

        const float C_WS_KP_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m]                              
        const float C_WS_KP_POSITION_Y =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m]
        const float C_WS_KP_POSITION_PITCH =  1000.0f * SCALE_GAINS_ANGULAR_POSITION;       //[Nm/deg]
        const float C_WS_KP_POSITION_ROLL =  5000.0f * SCALE_GAINS_ANGULAR_POSITION;        //[Nm/deg]
        const float C_WS_KP_POSITION_YAW = 1000.0f * SCALE_GAINS_ANGULAR_POSITION;          //[Nm/deg]

        const float C_WS_KD_POSITION_X =  0.0f* SCALE_GAINS_LINEAR_POSITION;              //[N.s/m]                
        const float C_WS_KD_POSITION_Y =  1.0f* SCALE_GAINS_LINEAR_POSITION;              //[N.s/m]
        const float C_WS_KD_POSITION_PITCH =  5.0f* SCALE_GAINS_ANGULAR_POSITION;          //[Nm.s/deg]
        const float C_WS_KD_POSITION_ROLL =  10.0f* SCALE_GAINS_ANGULAR_POSITION;           //[Nm.s/deg]
        const float C_WS_KD_POSITION_YAW = 5.0f* SCALE_GAINS_ANGULAR_POSITION;             //[Nm.s/deg]

           //***************************SHOULD-BE-ZERO****************/ 
                const float C_WS_KI_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;      //[N/m.s]                
                const float C_WS_KI_POSITION_Y =  0.0f * SCALE_GAINS_LINEAR_POSITION;      //[N/m.s]
                const float C_WS_KI_POSITION_PITCH =  0.0f * SCALE_GAINS_ANGULAR_POSITION;  //[Nm/deg.s]
                const float C_WS_KI_POSITION_ROLL =  0.0f * SCALE_GAINS_ANGULAR_POSITION;   //[Nm/deg.s]
                const float C_WS_KI_POSITION_YAW = 0.0f * SCALE_GAINS_ANGULAR_POSITION;     //[Nm/deg.s]


    #else           

        const float C_WS_KP_POSITION_X = 0.0f * SCALE_GAINS_LINEAR_POSITION;            //[N/m]                              
        const float C_WS_KP_POSITION_Y = 0.0f * SCALE_GAINS_LINEAR_POSITION;            //[N/m]
        const float C_WS_KP_POSITION_PITCH = 1000.0f * SCALE_GAINS_ANGULAR_POSITION;        //[Nm/deg]
        const float C_WS_KP_POSITION_ROLL = 5000.0f * SCALE_GAINS_ANGULAR_POSITION;         //[Nm/deg]
        const float C_WS_KP_POSITION_YAW =1000.0f * SCALE_GAINS_ANGULAR_POSITION;           //[Nm/deg]

        const float C_WS_KD_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;               //[N.s/m]                
        const float C_WS_KD_POSITION_Y =  1.0f * SCALE_GAINS_LINEAR_POSITION;               //[N.s/m]
        const float C_WS_KD_POSITION_PITCH =  5.0f * SCALE_GAINS_ANGULAR_POSITION;           //[Nm.s/deg]
        const float C_WS_KD_POSITION_ROLL =  10.0f * SCALE_GAINS_ANGULAR_POSITION;            //[Nm.s/deg]
        const float C_WS_KD_POSITION_YAW = 5.0f * SCALE_GAINS_ANGULAR_POSITION;              //[Nm.s/deg]

        //***************************SHOULD-BE-ZERO****************/ 

            const float C_WS_KI_POSITION_X =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m.s]                
            const float C_WS_KI_POSITION_Y =  0.0f * SCALE_GAINS_LINEAR_POSITION;           //[N/m.s]
            const float C_WS_KI_POSITION_PITCH =  0.0f * SCALE_GAINS_ANGULAR_POSITION;       //[Nm/deg.s]
            const float C_WS_KI_POSITION_ROLL =  0.0f * SCALE_GAINS_ANGULAR_POSITION;        //[Nm/deg.s]
            const float C_WS_KI_POSITION_YAW = 0.0f * SCALE_GAINS_ANGULAR_POSITION;          //[Nm/deg.s]

    
    #endif

//*************************MOTION_DAMPING****AKA.SUPRESS_TREMOR****************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        const float MOTION_DAMPING_KP_SPEED_X =  800.0f * SCALE_GAINS_LINEAR_SPEED;              //[N/m.s]                
        const float MOTION_DAMPING_KP_SPEED_Y =  800.0f * SCALE_GAINS_LINEAR_SPEED;              //[N/m.s]
        const float MOTION_DAMPING_KP_SPEED_PITCH =  170.0f * SCALE_GAINS_ANGULAR_SPEED;         //[Nm/deg.s]
        const float MOTION_DAMPING_KP_SPEED_ROLL =  170.0f * SCALE_GAINS_ANGULAR_SPEED;          //[Nm/deg.s]
        const float MOTION_DAMPING_KP_SPEED_YAW = 170.0f * SCALE_GAINS_ANGULAR_SPEED;            //[Nm/deg.s]            

           //***************************SHOULD-BE-ZERO****************/             

            const float MOTION_DAMPING_KI_SPEED_X =  0.0f * SCALE_GAINS_LINEAR_SPEED;             //[N.s/m.s]
            const float MOTION_DAMPING_KI_SPEED_Y =  0.0f * SCALE_GAINS_LINEAR_SPEED;             //[N.s/m.s]
            const float MOTION_DAMPING_KI_SPEED_PITCH =  0.0f * SCALE_GAINS_ANGULAR_SPEED;        //[N.s/deg.s]
            const float MOTION_DAMPING_KI_SPEED_ROLL =  0.0f * SCALE_GAINS_ANGULAR_SPEED;         //[N.s/deg.s]
            const float MOTION_DAMPING_KI_SPEED_YAW = 0.0f * SCALE_GAINS_ANGULAR_SPEED;           //[N.s/deg.s]

            const float MOTION_DAMPING_KD_SPEED_X  = 0.0f* SCALE_GAINS_LINEAR_SPEED;               //[N.s/m]
            const float MOTION_DAMPING_KD_SPEED_Y  = 0.0f* SCALE_GAINS_ANGULAR_SPEED;              //[Nm.s/deg]
            const float MOTION_DAMPING_KD_SPEED_PITCH  = 0.0f* SCALE_GAINS_ANGULAR_SPEED;          //[Nm.s/deg]
            const float MOTION_DAMPING_KD_SPEED_ROLL  = 0.0f* SCALE_GAINS_ANGULAR_SPEED;           //[Nm.s/deg]
            const float MOTION_DAMPING_KD_SPEED_YAW = 0.0f* SCALE_GAINS_LINEAR_SPEED;              //[N.s/m] 

    #else 
    
        const float MOTION_DAMPING_KP_SPEED_X  =  800.0f * SCALE_GAINS_LINEAR_SPEED;        //[N/m.s]   
        const float MOTION_DAMPING_KP_SPEED_Y  =  800.0f * SCALE_GAINS_LINEAR_SPEED;        //[N/m.s]
        const float MOTION_DAMPING_KP_SPEED_PITCH  =  170.0f * SCALE_GAINS_ANGULAR_SPEED;   //[Nm/deg.s]
        const float MOTION_DAMPING_KP_SPEED_ROLL  =  170.0f * SCALE_GAINS_ANGULAR_SPEED;    //[Nm/deg.s]
        const float MOTION_DAMPING_KP_SPEED_YAW  = 170.0f * SCALE_GAINS_ANGULAR_SPEED;      //[Nm/deg.s]

        //***************************SHOULD-BE-ZERO****************/ 

            const float MOTION_DAMPING_KI_SPEED_X   = 0.0f * SCALE_GAINS_LINEAR_SPEED;       //[N.s/m.s]
            const float MOTION_DAMPING_KI_SPEED_Y  =  0.0f * SCALE_GAINS_LINEAR_SPEED;       //[N.s/m.s]
            const float MOTION_DAMPING_KI_SPEED_PITCH  =  0.0f * SCALE_GAINS_ANGULAR_SPEED;  //[N.s/deg.s]
            const float MOTION_DAMPING_KI_SPEED_YAW  = 0.0f * SCALE_GAINS_ANGULAR_SPEED;     //[N.s/deg.s]
            const float MOTION_DAMPING_KI_SPEED_ROLL   = 0.0f * SCALE_GAINS_ANGULAR_SPEED;   //[N.s/deg.s]
              
            const float MOTION_DAMPING_KD_SPEED_X  =  0.0f * SCALE_GAINS_LINEAR_SPEED;        //[N.s/m] 
            const float MOTION_DAMPING_KD_SPEED_YAW  = 0.0f * SCALE_GAINS_LINEAR_SPEED;       //[N.s/m]
            const float MOTION_DAMPING_KD_SPEED_Y   = 0.0f * SCALE_GAINS_ANGULAR_SPEED;       //[Nm.s/deg]
            const float MOTION_DAMPING_KD_SPEED_PITCH  =  0.0f * SCALE_GAINS_ANGULAR_SPEED;   //[Nm.s/deg]
            const float MOTION_DAMPING_KD_SPEED_ROLL   = 0.0f * SCALE_GAINS_ANGULAR_SPEED;    //[Nm.s/deg]


    #endif



#endif //DEFINITIONS_2_HH
