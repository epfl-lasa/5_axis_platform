#ifndef DEFINITIONS_ROS_H
#define DEFINITIONS_ROS_H

#include "definitions_main.h"


const int rosAxis[] = {X, Y, PITCH, ROLL, YAW}; //! This is because the first
                                                //! joint is Y and then X in the
                                                //! real platform and in the
                                                //! model... but in ros is X

enum FootInput_Category { MSG_POSITION, MSG_SPEED, MSG_TORQUE};

enum Param_Category {PID_POS_C, PID_VEL_C, PID_FS_C, PID_SOFT_LIMITS_C, COMPENSATION_C, ALL};

#define NB_FI_CATEGORY 3 //! Category of the information of the foot input message

//******************************NAMES*****************************
    #define PLATFORM_SUBSCRIBER_NAME_LEFT "/FI_Input/Left"
    #define FORCE_SUBSCRIBER_NAME_LEFT "/left/rokubimini0/force/"
    #define PLATFORM_PUBLISHER_NAME_LEFT "/FI_Output/Left"
    #define SERVICE_CHANGE_STATE_NAME_LEFT "/update_left_state"
    #define SERVICE_CHANGE_CTRL_NAME_LEFT "/update_left_controller"
    
    #define PARAM_P_POS_LEFT "/left/p_pos"
    #define PARAM_I_POS_LEFT "/left/i_pos"
    #define PARAM_D_POS_LEFT "/left/d_pos"
    #define PARAM_WALL_POINT_LEFT_MIN "/left/v_wall/point_min"
    #define PARAM_WALL_POINT_LEFT_MAX "/left/v_wall/point_max"
    #define PARAM_P_WALL_LEFT "/left/v_wall/p_pos"
    #define PARAM_I_WALL_LEFT "/left/v_wall/i_pos"
    #define PARAM_D_WALL_LEFT "/left/v_wall/d_pos"
    #define PARAM_P_SPEED_LEFT "/left/p_vel"
    #define PARAM_I_SPEED_LEFT "/left/i_vel"
    #define PARAM_D_SPEED_LEFT "/left/d_vel"
    #define PARAM_P_FS_LEFT "/left/p_fs"
    #define PARAM_I_FS_LEFT "/left/i_fs"
    #define PARAM_D_FS_LEFT "/left/d_fs"
    #define PARAM_P_RCM_LEFT "/left/p_rcm"
    #define PARAM_I_RCM_LEFT "/left/i_rcm"
    #define PARAM_D_RCM_LEFT "/left/d_rcm"
    #define PARAM_POS_RCM_LEFT "/left/pos_rcm"
    #define PARAM_COMPENSATION_LEFT "/left/comp_"
    
    #define PLATFORM_SUBSCRIBER_NAME_RIGHT "/FI_Input/Right"
    #define FORCE_SUBSCRIBER_NAME_RIGHT "/right/rokubimini0/force"
    #define PLATFORM_PUBLISHER_NAME_RIGHT "/FI_Output/Right"
    #define SERVICE_CHANGE_STATE_NAME_RIGHT "/update_right_state"
    #define SERVICE_CHANGE_CTRL_NAME_RIGHT "/update_right_controller"
    #define PARAM_P_POS_RIGHT "/right/p_pos"
    #define PARAM_I_POS_RIGHT "/right/i_pos"
    #define PARAM_D_POS_RIGHT "/right/d_pos"
    #define PARAM_WALL_POINT_RIGHT_MIN "/right/v_wall/point_min"
    #define PARAM_WALL_POINT_RIGHT_MAX "/right/v_wall/point_max"
    #define PARAM_P_WALL_RIGHT "/right/v_wall/p_pos"
    #define PARAM_I_WALL_RIGHT "/right/v_wall/i_pos"
    #define PARAM_D_WALL_RIGHT "/right/v_wall/d_pos"
    #define PARAM_P_SPEED_RIGHT "/right/p_vel"
    #define PARAM_I_SPEED_RIGHT "/right/i_vel"
    #define PARAM_D_SPEED_RIGHT "/right/d_vel"
    #define PARAM_P_FS_RIGHT "/right/p_fs"
    #define PARAM_I_FS_RIGHT "/right/i_fs"
    #define PARAM_D_FS_RIGHT "/right/d_fs"
    #define PARAM_P_RCM_RIGHT "/right/p_rcm"
    #define PARAM_I_RCM_RIGHT "/right/i_rcm"
    #define PARAM_D_RCM_RIGHT "/right/d_rcm"
    #define PARAM_POS_RCM_RIGHT "/right/pos_rcm"
    #define PARAM_COMPENSATION_RIGHT "/right/comp_"


    #if (PLATFORM_ID == LEFT_PLATFORM_ID) 
        #define PLATFORM_SUBSCRIBER_NAME PLATFORM_SUBSCRIBER_NAME_LEFT
        #define FORCE_SUBSCRIBER_NAME FORCE_SUBSCRIBER_NAME_LEFT
        #define PLATFORM_PUBLISHER_NAME PLATFORM_PUBLISHER_NAME_LEFT
        #define SERVICE_CHANGE_STATE_NAME SERVICE_CHANGE_STATE_NAME_LEFT
        #define SERVICE_CHANGE_CTRL_NAME SERVICE_CHANGE_CTRL_NAME_LEFT
        #define PARAM_P_POS_NAME PARAM_P_POS_LEFT
        #define PARAM_I_POS_NAME PARAM_I_POS_LEFT
        #define PARAM_D_POS_NAME PARAM_D_POS_LEFT
        #define PARAM_WALL_POINT_MIN PARAM_WALL_POINT_LEFT_MIN
        #define PARAM_WALL_POINT_MAX PARAM_WALL_POINT_LEFT_MAX
        #define PARAM_P_WALL_NAME PARAM_P_WALL_LEFT
        #define PARAM_I_WALL_NAME PARAM_I_WALL_LEFT
        #define PARAM_D_WALL_NAME PARAM_D_WALL_LEFT
        #define PARAM_P_SPEED_NAME PARAM_P_SPEED_LEFT
        #define PARAM_I_SPEED_NAME PARAM_I_SPEED_LEFT
        #define PARAM_D_SPEED_NAME PARAM_D_SPEED_LEFT
        #define PARAM_P_FS_NAME PARAM_P_FS_LEFT
        #define PARAM_I_FS_NAME PARAM_I_FS_LEFT
        #define PARAM_D_FS_NAME PARAM_D_FS_LEFT
        #define PARAM_P_RCM_NAME PARAM_P_RCM_LEFT
        #define PARAM_I_RCM_NAME PARAM_I_RCM_LEFT
        #define PARAM_D_RCM_NAME PARAM_D_RCM_LEFT
        #define PARAM_POS_RCM_NAME PARAM_POS_RCM_LEFT
        #define PARAM_COMPENSATION_NAME  PARAM_COMPENSATION_LEFT
        
        
    #else
        #define PLATFORM_SUBSCRIBER_NAME PLATFORM_SUBSCRIBER_NAME_RIGHT
        #define FORCE_SUBSCRIBER_NAME FORCE_SUBSCRIBER_NAME_RIGHT
        #define PLATFORM_PUBLISHER_NAME PLATFORM_PUBLISHER_NAME_RIGHT
        #define SERVICE_CHANGE_STATE_NAME SERVICE_CHANGE_STATE_NAME_RIGHT
        #define SERVICE_CHANGE_CTRL_NAME SERVICE_CHANGE_CTRL_NAME_RIGHT   
        #define PARAM_P_POS_NAME PARAM_P_POS_RIGHT
        #define PARAM_I_POS_NAME PARAM_I_POS_RIGHT
        #define PARAM_D_POS_NAME PARAM_D_POS_RIGHT
        #define PARAM_WALL_POINT_MIN PARAM_WALL_POINT_RIGHT_MIN
        #define PARAM_WALL_POINT_MAX PARAM_WALL_POINT_RIGHT_MAX
        #define PARAM_P_WALL_NAME PARAM_P_WALL_RIGHT
        #define PARAM_I_WALL_NAME PARAM_I_WALL_RIGHT
        #define PARAM_D_WALL_NAME PARAM_D_WALL_RIGHT
        #define PARAM_P_SPEED_NAME PARAM_P_SPEED_RIGHT
        #define PARAM_I_SPEED_NAME PARAM_I_SPEED_RIGHT
        #define PARAM_D_SPEED_NAME PARAM_D_SPEED_RIGHT
        #define PARAM_P_FS_NAME PARAM_P_FS_RIGHT
        #define PARAM_I_FS_NAME PARAM_I_FS_RIGHT
        #define PARAM_D_FS_NAME PARAM_D_FS_RIGHT
        #define PARAM_P_RCM_NAME PARAM_P_RCM_RIGHT
        #define PARAM_I_RCM_NAME PARAM_I_RCM_RIGHT
        #define PARAM_D_RCM_NAME PARAM_D_RCM_RIGHT
        #define PARAM_POS_RCM_NAME PARAM_POS_RCM_RIGHT
        #define PARAM_COMPENSATION_NAME PARAM_COMPENSATION_RIGHT

#endif


#endif // DEFINITIONS_ROS_H