#ifndef DEFINITIONS_ROS_H
#define DEFINITIONS_ROS_H

#include "definitions_main.h"

const int rosAxis[] = {X, Y, PITCH, ROLL, YAW}; //! This is because the first
                                                //! joint is Y and then X in the
                                                //! real platform and in the
                                                //! model... but in ros is X

enum FootInput_Category { MSG_POSITION, MSG_SPEED, MSG_TORQUE};

#define NB_FI_CATEGORY 3 //! Category of the information of the foot input message

//******************************NAMES*****************************
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


#endif // DEFINITIONS_ROS_H