#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>
#include "/home/lsrob107772/.platformio/lib/Eigen_ID3522/Dense.h"
//#include "Dense.h"

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