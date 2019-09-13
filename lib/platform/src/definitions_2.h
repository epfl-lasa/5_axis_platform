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

        #define SPEED_D_HOMING_X -2.7                               //[m/s]                         
        #define SPEED_D_HOMING_Y 2.5                                //[deg/s]
        #define SPEED_D_HOMING_PITCH 300.0                          //[deg/s]                             
        
        #define KP_HOMING_SPEED_X 2500.0f * 1e-2                    //[N.s/m]     
        #define KP_HOMING_SPEED_Y 2000.0f * 1e-2                    //[N.s/m]     
        #define KP_HOMING_SPEED_PITCH 1700.0f *1e-5                 //[Nm.s/deg]     
        
        #define KI_HOMING_SPEED_X 2500.0f*1e-2                      //[N.s/m.s]     
        #define KI_HOMING_SPEED_Y 2000.0f*1e-2                      //[N.s/m.s]     
        #define KI_HOMING_SPEED_PITCH 850.0f * 1e-5                 //[N.s/deg.s]     

    #else 

        #define SPEED_D_HOMING_X 2.5                                //[m/s]                    
        #define SPEED_D_HOMING_Y 2.5                                //[deg/s]
        #define SPEED_D_HOMING_PITCH -300                           //[deg/s]

        #define KP_HOMING_SPEED_X 2500.0f * 1e-2                    //[N.s/m]
        #define KP_HOMING_SPEED_Y 1500.0f * 1e-2                    //[N.s/m]   
        #define KP_HOMING_SPEED_PITCH 1700.0f * 1e-5                //[Nm.s/deg]

        #define KI_HOMING_SPEED_X 2500.0f * 1e-2                    //[N.s/m.s]
        #define KI_HOMING_SPEED_Y 1000.0f * 1e-2                    //[N.s/m.s]     
        #define KI_HOMING_SPEED_PITCH 850.0f * 1e-5                 //[N.s/deg.s]    
    #endif


//*************************GOTO******************************************************


    #if (PLATFORM_ID==LEFT_PLATFORM)
    
        #define GT_KP_POSITION_X  2000.0f                           //[N/m]
        #define GT_KP_POSITION_Y  2500.0f                           //[N/m]
        #define GT_KP_POSITION_PITCH  4250.0f * 1e-4                //[Nm/deg]
        #define GT_KP_POSITION_ROLL  4250.0f * 1e-4                 //[Nm/deg]
        #define GT_KP_POSITION_YAW 4250.0f * 1e-4                   //[Nm/deg]
        
        #define GT_KI_POSITION_X  2000.0f                           //[N/m.s]
        #define GT_KI_POSITION_Y  2500.0f                           //[N/m.s]
        #define GT_KI_POSITION_PITCH  8500.0f * 1e-4                //[N/deg.s]   
        #define GT_KI_POSITION_ROLL  8500.0f * 1e-4                 //[N/deg.s]   
        #define GT_KI_POSITION_YAW 8500.0f * 1e-4                   //[N/deg.s]

        
        #define GT_KD_POSITION_X  1.0f                              //[N.s/m]                
        #define GT_KD_POSITION_Y  0.5f                              //[N.s/m]
        #define GT_KD_POSITION_PITCH  8.5f * 1e-4                   //[Nm.s/deg]
        #define GT_KD_POSITION_ROLL  8.5f * 1e-4                    //[Nm.s/deg]
        #define GT_KD_POSITION_YAW  8.5f * 1e-4                     //[Nm.s/deg]
        
        
    #else  //! TODO TUNE FOR RIGHT_PLATFORM
    
        #define GT_KP_POSITION_X  2000.0f                           //[N/m]
        #define GT_KP_POSITION_Y  2500.0f                           //[N/m]
        #define GT_KP_POSITION_PITCH  4250.0f * 1e-4                //[Nm/deg]
        #define GT_KP_POSITION_ROLL  4250.0f * 1e-4                 //[Nm/deg]
        #define GT_KP_POSITION_YAW 4250.0f * 1e-4                   //[Nm/deg]
               
        #define GT_KI_POSITION_X  2000.0f                           //[N/m.s]
        #define GT_KI_POSITION_Y  2500.0f                           //[N/m.s]
        #define GT_KI_POSITION_PITCH  8500.0f * 1e-4                //[N/deg.s]   
        #define GT_KI_POSITION_ROLL  8500.0f * 1e-4                 //[N/deg.s]   
        #define GT_KI_POSITION_YAW 8500.0f * 1e-4                   //[N/deg.s]

        #define GT_KD_POSITION_X  1.0f                              //[N.s/m]                
        #define GT_KD_POSITION_Y  0.5f                              //[N.s/m]
        #define GT_KD_POSITION_PITCH  8.5f * 1e-4                   //[Nm.s/deg]
        #define GT_KD_POSITION_ROLL  8.5f * 1e-4                    //[Nm.s/deg]
        #define GT_KD_POSITION_YAW 8.5f * 1e-4                      //[Nm.s/deg]

    #endif

//*************************W_CONSTRAINS****AKA.VIRTUAL_WALLS****************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        #define C_WS_KP_POSITION_X  2000.0f                          //[N/m]                              
        #define C_WS_KP_POSITION_Y  2000.0f                          //[N/m]
        #define C_WS_KP_POSITION_PITCH  3400.0f * 1e-4               //[Nm/deg]
        #define C_WS_KP_POSITION_ROLL  3400.0f * 1e-4                //[Nm/deg]
        #define C_WS_KP_POSITION_YAW 3400.0f * 1e-4                  //[Nm/deg]

        #define C_WS_KD_POSITION_X  1.0f                             //[N.s/m]                
        #define C_WS_KD_POSITION_Y  0.5f                             //[N.s/m]
        #define C_WS_KD_POSITION_PITCH  3.4f * 1e-4                  //[Nm.s/deg]
        #define C_WS_KD_POSITION_ROLL  3.4f * 1e-4                   //[Nm.s/deg]
        #define C_WS_KD_POSITION_YAW 3.4f * 1e-4                     //[Nm.s/deg]

           //***************************SHOULD-BE-ZERO****************/ 
                #define C_WS_KI_POSITION_X  0.0f                     //[N/m.s]                
                #define C_WS_KI_POSITION_Y  0.0f                     //[N/m.s]
                #define C_WS_KI_POSITION_PITCH  0.0f                 //[Nm/deg.s]
                #define C_WS_KI_POSITION_ROLL  0.0f                  //[Nm/deg.s]
                #define C_WS_KI_POSITION_YAW 0.0f                    //[Nm/deg.s]
    
    
    #else
    
        #define C_WS_KP_POSITION_X  1000.0f                         //[N/m]                              
        #define C_WS_KP_POSITION_Y  2000.0f                         //[N/m]
        #define C_WS_KP_POSITION_PITCH  3400.0f * 1e-4              //[Nm/deg]
        #define C_WS_KP_POSITION_ROLL  3400.0f * 1e-4               //[Nm/deg]
        #define C_WS_KP_POSITION_YAW 3400.0f * 1e-4                 //[Nm/deg]

        #define C_WS_KD_POSITION_X  1.0f                            //[N.s/m]                
        #define C_WS_KD_POSITION_Y  0.5f                            //[N.s/m]
        #define C_WS_KD_POSITION_PITCH  3.4f * 1e-4                 //[Nm.s/deg]
        #define C_WS_KD_POSITION_ROLL  3.4f * 1e-4                  //[Nm.s/deg]
        #define C_WS_KD_POSITION_YAW 3.4f * 1e-4                    //[Nm.s/deg]

        //***************************SHOULD-BE-ZERO****************/ 

            #define C_WS_KI_POSITION_X  0.0f                        //[N/m.s]                
            #define C_WS_KI_POSITION_Y  0.0f                        //[N/m.s]
            #define C_WS_KI_POSITION_PITCH  0.0f                    //[Nm/deg.s]
            #define C_WS_KI_POSITION_ROLL  0.0f                     //[Nm/deg.s]
            #define C_WS_KI_POSITION_YAW 0.0f                       //[Nm/deg.s]

    
    #endif

//*************************MOTION_DAMPING****AKA.SUPRESS_TREMOR****************************


    #if (PLATFORM_ID==LEFT_PLATFORM)

        #define MOTION_DAMPING_KP_SPEED_X  800.0f*1e-2              //[N/m.s]                
        #define MOTION_DAMPING_KP_SPEED_Y  800.0f*1e-2              //[N/m.s]
        #define MOTION_DAMPING_KP_SPEED_PITCH  170.0f * 1e-5        //[Nm/deg.s]
        #define MOTION_DAMPING_KP_SPEED_ROLL  170.0f * 1e-5         //[Nm/deg.s]
        #define MOTION_DAMPING_KP_SPEED_YAW 170.0f * 1e-5           //[Nm/deg.s]            

           //***************************SHOULD-BE-ZERO****************/             

            #define MOTION_DAMPING_KI_SPEED_YAW 0.0f                
            #define MOTION_DAMPING_KI_SPEED_X  0.0f
            #define MOTION_DAMPING_KI_SPEED_Y  0.0f 
            #define MOTION_DAMPING_KI_SPEED_PITCH  0.0f
            #define MOTION_DAMPING_KI_SPEED_ROLL  0.0f

            #define MOTION_DAMPING_KD_SPEED_YAW 0.0f
            #define MOTION_DAMPING_KD_SPEED_X  0.0f
            #define MOTION_DAMPING_KD_SPEED_Y  0.0f
            #define MOTION_DAMPING_KD_SPEED_PITCH  0.0f
            #define MOTION_DAMPING_KD_SPEED_ROLL  0.0f

    #else 
    
        #define MOTION_DAMPING_KP_SPEED_X  800.0f*1e-2
        #define MOTION_DAMPING_KP_SPEED_Y  800.0f*1e-2
        #define MOTION_DAMPING_KP_SPEED_PITCH  170.0f * 1e-5
        #define MOTION_DAMPING_KP_SPEED_ROLL  170.0f * 1e-5
        #define MOTION_DAMPING_KP_SPEED_YAW 170.0f * 1e-5

        //***************************SHOULD-BE-ZERO****************/ 

            #define MOTION_DAMPING_KI_SPEED_X  0.0f
            #define MOTION_DAMPING_KI_SPEED_Y  0.0f 
            #define MOTION_DAMPING_KI_SPEED_PITCH  0.0f
            #define MOTION_DAMPING_KI_SPEED_YAW 0.0f 
            #define MOTION_DAMPING_KI_SPEED_ROLL  0.0f 
            
            #define MOTION_DAMPING_KD_SPEED_X  0.0f
            #define MOTION_DAMPING_KD_SPEED_YAW 0.0f
            #define MOTION_DAMPING_KD_SPEED_Y  0.0f
            #define MOTION_DAMPING_KD_SPEED_PITCH  0.0f
            #define MOTION_DAMPING_KD_SPEED_ROLL  0.0f

    #endif



#endif //DEFINITIONS_2_HH
