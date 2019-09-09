#ifndef DEFINITIONS_2_HH
#define DEFINITIONS_2_HH

#include <definitions.h>
#include <Platform.h>

//******************************CONSTRUCTOR*****************************

    #if (PLATFORM_ID == LEFT_PLATFORM) 
        #define SUBSCRIBER_NAME "/FI_Input/Left"
        #define PUBLISHER_NAME "/FI_Output/Left"
        #define SERVICE_CHANGE_STATE_NAME "update_left_state"
        #define SERVICE_CHANGE_CTRL_NAME "update_left_controller"
    #else
        #define SUBSCRIBER_NAME "/FI_Input/Right"
        #define PUBLISHER_NAME "/FI_Output/Right"
        #define SERVICE_CHANGE_STATE_NAME "update_right_state"
        #define SERVICE_CHANGE_CTRL_NAME "update_right_controller"
    #endif



//******************************HOMING********************************

    #if (PLATFORM_ID==LEFT_PLATFORM) //! TODO: Set for the left platform

        #define TWIST_D_HOMING_X -2.5 // m/s
        #define TWIST_D_HOMING_Y 2.5 // m/s
        #define TWIST_D_HOMING_PITCH 300; // °/s
        
        #define KP_HOMING_TWIST_X 2500.0f*0.01f
        #define KI_HOMING_TWIST_X 2500.0f*0.01f 
        #define KP_HOMING_TWIST_Y 2000.0f*0.01f
        #define KI_HOMING_TWIST_Y 2000.0f*0.01f //
        #define KP_HOMING_TWIST_PITCH 10000.0f * PI / 180.0f * 1e-4f //
        #define KI_HOMING_TWIST_PITCH 5000.0f * PI / 180.0f * 1e-4f //   

    #else 

        #define TWIST_D_HOMING_X 2.5 // m/s
        #define TWIST_D_HOMING_Y 2.5 // m/s
        #define TWIST_D_HOMING_PITCH -300 // °/s

        #define KP_HOMING_TWIST_X 2500.0f*0.01f
        #define KI_HOMING_TWIST_X 2500.0f*0.01f 
        #define KP_HOMING_TWIST_Y 1500.0f*0.01f
        #define KI_HOMING_TWIST_Y 1000.0f*0.01f //
        #define KP_HOMING_TWIST_PITCH 10000.0f * PI / 180.0f * 1e-4f //
        #define KI_HOMING_TWIST_PITCH 5000.0f * PI / 180.0f * 1e-4f // 
    #endif


//*************************GOTO********************************


    #if (PLATFORM_ID==LEFT_PLATFORM)
    
        #define GT_KP_POSE_X  2000.0f
        #define GT_KD_POSE_X  1.0f
        #define GT_KI_POSE_X  2000.0f //Ki->0.0f
        #define GT_KP_POSE_Y  2500.0f
        #define GT_KD_POSE_Y  0.5f
        #define GT_KI_POSE_Y  2500.0f //Ki->0.0f
        #define GT_KP_POSE_PITCH  3000.0f * PI / 180.0f * 0.01f //2000.0
        #define GT_KD_POSE_PITCH  5.0f * PI / 180.0f * 0.01f // 5.0
        #define GT_KI_POSE_PITCH  5000.0f * PI / 180.0f * 0.01f // 1000.0 
        #define GT_KP_POSE_ROLL  2500.0f * PI / 180.0f * 0.01f
        #define GT_KD_POSE_ROLL  10.0f * PI / 180.0f * 0.01f
        #define GT_KI_POSE_ROLL  1000.0f * PI / 180.0f * 0.01f 
        #define GT_KP_POSE_YAW 2500.0f * PI / 180.0f * 0.01f
        #define GT_KD_POSE_YAW 10.0f * PI / 180.0f * 0.01f
        #define GT_KI_POSE_YAW 1000.0f * PI / 180.0f * 0.01f 

    #else  //! TODO TUNE FOR RIGHT_PLATFORM
    
        #define GT_KP_POSE_X  1000.0f
        #define GT_KD_POSE_X  1.0f
        #define GT_KI_POSE_X  1000.0f //Ki->0.0f
        #define GT_KP_POSE_Y  2500.0f
        #define GT_KD_POSE_Y  0.5f
        #define GT_KI_POSE_Y  1000.0f //Ki->0.0f
        #define GT_KP_POSE_PITCH  2500.0f * PI / 180.0f * 0.01f //2000.0
        #define GT_KD_POSE_PITCH  5.0f * PI / 180.0f * 0.01f // 5.0
        #define GT_KI_POSE_PITCH  2500.0f * PI / 180.0f * 0.01f // 1000.0 
        #define GT_KP_POSE_ROLL  2500.0f * PI / 180.0f * 0.01f
        #define GT_KD_POSE_ROLL  10.0f * PI / 180.0f * 0.01f
        #define GT_KI_POSE_ROLL  1000.0f * PI / 180.0f * 0.01f 
        #define GT_KP_POSE_YAW 2500.0f * PI / 180.0f * 0.01f
        #define GT_KD_POSE_YAW 10.0f * PI / 180.0f * 0.01f
        #define GT_KI_POSE_YAW 1000.0f * PI / 180.0f * 0.01f 

    #endif

//*************************W_CONSTRAINS****AKA.VIRTUAL_WALLS****************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        #define C_WS_KP_POSE_X  2000.0f
        #define C_WS_KD_POSE_X  1.0f
        #define C_WS_KI_POSE_X  2000.0f 
        #define C_WS_KP_POSE_Y  2500.0f
        #define C_WS_KD_POSE_Y  0.5f
        #define C_WS_KI_POSE_Y  0.0f 
        #define C_WS_KP_POSE_PITCH  2000.0f * PI / 180.0f * 0.01f //2000.0
        #define C_WS_KD_POSE_PITCH  5.0f * PI / 180.0f * 0.01f // 5.0
        #define C_WS_KI_POSE_PITCH  0.0f
        #define C_WS_KP_POSE_ROLL  2000.0f * PI / 180.0f * 0.01f
        #define C_WS_KD_POSE_ROLL  10.0f * PI / 180.0f * 0.01f
        #define C_WS_KI_POSE_ROLL  0.0f 
        #define C_WS_KP_POSE_YAW 2000.0f * PI / 180.0f * 0.01f
        #define C_WS_KD_POSE_YAW 10.0f * PI / 180.0f * 0.01f
        #define C_WS_KI_POSE_YAW 0.0f 

    #else  //! TODO TUNE FOR RIGHT_PLATFORM
    
        #define C_WS_KP_POSE_X  1000.0f
        #define C_WS_KD_POSE_X  1.0f
        #define C_WS_KI_POSE_X  1000.0f //Ki->0.0f
        #define C_WS_KP_POSE_Y  2500.0f
        #define C_WS_KD_POSE_Y  0.5f
        #define C_WS_KI_POSE_Y  1000.0f //Ki->0.0f
        #define C_WS_KP_POSE_PITCH  2500.0f * PI / 180.0f * 0.01f //2000.0
        #define C_WS_KD_POSE_PITCH  5.0f * PI / 180.0f * 0.01f // 5.0
        #define C_WS_KI_POSE_PITCH  2500.0f * PI / 180.0f * 0.01f // 1000.0 
        #define C_WS_KP_POSE_ROLL  2500.0f * PI / 180.0f * 0.01f
        #define C_WS_KD_POSE_ROLL  10.0f * PI / 180.0f * 0.01f
        #define C_WS_KI_POSE_ROLL  1000.0f * PI / 180.0f * 0.01f 
        #define C_WS_KP_POSE_YAW 2500.0f * PI / 180.0f * 0.01f
        #define C_WS_KD_POSE_YAW 10.0f * PI / 180.0f * 0.01f
        #define C_WS_KI_POSE_YAW 1000.0f * PI / 180.0f * 0.01f 

    #endif

//*************************MOTION_DAMPING****AKA.SUPRESS_TREMOR****************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        #define MOTION_DAMPING_KP_TWIST_X  2500.0f*0.01f
        #define MOTION_DAMPING_KP_TWIST_Y  2000.0f*0.01f
        #define MOTION_DAMPING_KP_TWIST_PITCH  10000.0f * PI / 180.0f * 1e-4f
        #define MOTION_DAMPING_KP_TWIST_ROLL  10000.0f * PI / 180.0f * 1e-4f
        #define MOTION_DAMPING_KP_TWIST_YAW 10000.0f * PI / 180.0f * 1e-4f

        #define MOTION_DAMPING_KD_TWIST_YAW 0.0f
        #define MOTION_DAMPING_KI_TWIST_YAW 0.0f 
        #define MOTION_DAMPING_KD_TWIST_X  0.0f
        #define MOTION_DAMPING_KI_TWIST_X  0.0f
        #define MOTION_DAMPING_KD_TWIST_Y  0.0f
        #define MOTION_DAMPING_KI_TWIST_Y  0.0f 
        #define MOTION_DAMPING_KD_TWIST_PITCH  0.0f
        #define MOTION_DAMPING_KI_TWIST_PITCH  0.0f
        #define MOTION_DAMPING_KD_TWIST_ROLL  0.0f
        #define MOTION_DAMPING_KI_TWIST_ROLL  0.0f 

    #else 
    
        #define MOTION_DAMPING_KP_TWIST_X  1000.0f*0.01f
        #define MOTION_DAMPING_KP_TWIST_Y  1000.0f*0.01f
        #define MOTION_DAMPING_KP_TWIST_PITCH  1000.0f * PI / 180.0f * 1e-4f
        #define MOTION_DAMPING_KP_TWIST_ROLL  1000.0f * PI / 180.0f * 1e-4f
        #define MOTION_DAMPING_KP_TWIST_YAW 1000.0f * PI / 180.0f * 1e-4f

        #define MOTION_DAMPING_KD_TWIST_YAW 0.0f
        #define MOTION_DAMPING_KI_TWIST_YAW 0.0f 
        #define MOTION_DAMPING_KD_TWIST_X  0.0f
        #define MOTION_DAMPING_KI_TWIST_X  0.0f
        #define MOTION_DAMPING_KD_TWIST_Y  0.0f
        #define MOTION_DAMPING_KI_TWIST_Y  0.0f 
        #define MOTION_DAMPING_KD_TWIST_PITCH  0.0f
        #define MOTION_DAMPING_KI_TWIST_PITCH  0.0f
        #define MOTION_DAMPING_KD_TWIST_ROLL  0.0f
        #define MOTION_DAMPING_KI_TWIST_ROLL  0.0f 

    #endif



#endif //DEFINITIONS_2_HH
