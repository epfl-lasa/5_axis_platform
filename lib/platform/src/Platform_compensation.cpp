#include <Platform.h>
#include <LP_Filter.h>
#include <definitions.h>
#include <definitions_2.h>
#include "/home/lsrob107772/.platformio/lib/Eigen_ID3522/Dense.h"
//#include "Dense.h"

float const SPEED_THRESHOLD[NB_AXIS] = {0.001f, 0.001f, 0.05f, 0.09f, 0.09f};
float const MIN_SPEED[NB_AXIS] = {0.00002f, 0.00002f, 0.00002f, 0.02f, 0.02f};

LP_Filter comp_filter(0.7f);

using namespace std;

#define WITH_FORCE_SENSOR 1
#define WITHOUT_FORCE_SENSOR 0

#define PRESENCE_FORCE_SENSOR WITHOUT_FORCE_SENSOR

#if (PRESENCE_FORCE_SENSOR)
  const float CoMDistanceSensorFromAxis = 0.100f; //[m]
  const float CoMMassSensor = 1.2f;               //[kg]
#else
  const float CoMDistanceSensorFromAxis = 0.100f - 0.056f -0.020f; //[m]
  const float CoMMassSensor = 1.2f-0.295f;
#endif

//! 1
void Platform::gravityCompensation()
{

  Eigen::Vector3f Torque, Weight, CoMVector;
  Eigen::Matrix3f R_pitch, R_roll, R_tot;
  CoMVector << 0.0, 0.0, CoMDistanceSensorFromAxis;
  Weight << 0.0, 0.0, (CoMMassSensor * GRAVITY);

  float rad_pitch = _position[PITCH] / 180.0 * PI;
  float rad_roll = _position[ROLL] / 180.0 * PI;

  R_pitch<<1.0f , 0.0f, 0.0f, 
            0.0f, cos(rad_pitch), -sin(rad_pitch),
            0.0f, sin(rad_pitch), cos(rad_pitch);

  R_roll << cos(rad_roll)  , 0.0f, sin(rad_roll),
              0.0f          , 1.0f, 0.0f,
            -sin(rad_roll), 0.0f ,cos(rad_roll);
  R_tot=R_pitch*R_roll;

  Torque = - R_tot * (CoMVector.cross(R_tot.transpose()*Weight));

 _effortD_ADD[COMPENSATION][PITCH] = Torque(0);
 _effortD_ADD[COMPENSATION][ROLL] = Torque(1);
    
  }

#define LINEAR_MODEL 1
#define NON_LINEAR_MODEL_JACOBIAN 2
#define MODEL_TYPE LINEAR_MODEL

void Platform::stictionCompensation(int axis_)
{
  if (axis_==-1)
  {
    for (int k=0; k<NB_AXIS; k++)
    {
      stictionCompensation(k);
    }
  }
  else
  {
    switch(axis_)
    {
      case X:
        {
        float comp_effort = 0.0f;
        float stiction =0.0f;
        float max_torque = 6.6067f;
        float min_torque = -8.5588f;
        #if (MODEL_TYPE==LINEAR_MODEL)

          if (abs(_speed[X])>=MIN_SPEED[X])
          
          { comp_effort =
              -0.29777f +
              24.189f * _position[X] +
              4.2495f * _position[Y] +
              -1.0458f * _position[PITCH] +
              // -0.32975f * _position[ROLL] +
               0.49452f * _position[YAW] + 
              3464.8f * _speed[X] +
              -34328.0f * _speed[Y] + 
              // 1327.0f * _speed[PITCH] +
              // -1361.8f * _speed[ROLL] +
              // 848.42f * _speed[YAW] +
              0.0f;
            stiction=comp_filter.update(comp_effort);
          }

        
        #elif (MODEL_TYPE == NON_LINEAR_MODEL_JACOBIAN)

        if (abs(_speed[X]) >= MIN_SPEED[X])

        {
          comp_effort = -0.29777f + 24.189f * _position[X] +
                        4.2495f * _position[Y] + -1.0458f * _position[PITCH] +
                        // -0.32975f * _position[ROLL] +
                        0.49452f * _position[YAW] + 3464.8f * _speed[X] +
                        // -34328.0f * _speed[Y] +
                        // 1327.0f * _speed[PITCH] +
                        // -1361.8f * _speed[ROLL] +
                        // 848.42f * _speed[YAW] +
                        0.0f;
        }

        #else 
        #endif
          stiction = clamp(stiction, min_torque, max_torque);
          _effortD_ADD[COMPENSATION][X] = stiction;
        }
    }
  }
}
