#include <Platform.h>
#include <LP_Filter.h>
#include <definitions.h>
#include <definitions_2.h>

float const SPEED_THRESHOLD[NB_AXIS] = {0.0035f, 0.004f, 0.05f, 0.09f, 0.09f};
float const MIN_SPEED[NB_AXIS] = {0.00002f, 0.00002f, 0.00002f, 0.02f, 0.02f};

LP_Filter comp_filter[NB_STICTION_COMP]{0.5f, 0.7f};

using namespace std;
using namespace Eigen;

#define WITH_FORCE_SENSOR 1
#define WITHOUT_FORCE_SENSOR 0

#define PRESENCE_FORCE_SENSOR WITHOUT_FORCE_SENSOR

#if (PRESENCE_FORCE_SENSOR)
  const float CoMDistanceSensorFromAxis = 0.100f; //[m]
  const float CoMMassSensor = 1.2f;               //[kg]
#else
  const float CoMDistanceSensorFromAxis = 0.100f - 0.056f -0.010f; //[m]
  const float CoMMassSensor = 1.2f-0.295f;
#endif

//! 1
void Platform::gravityCompensation()
{

  Eigen::Vector3f Torque, Weight, CoMVector;
  Eigen::Matrix3f R_pitch, R_roll, R_tot;
  CoMVector << 0.0, 0.0, CoMDistanceSensorFromAxis;
  Weight << 0.0, 0.0, (CoMMassSensor * GRAVITY);

  float rad_pitch = _position[PITCH] * DEG_TO_RAD;
  float rad_roll = _position[ROLL] * DEG_TO_RAD;

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
#define GAUSSIAN_PROCESS 2

#define MODEL_TYPE LINEAR_MODEL

float const viscous_k[NB_VISCOUS_COMP] = {0.0f, 0.0f, 0.0862f / SPEED_THRESHOLD[PITCH],
                                            0.1859f / SPEED_THRESHOLD[ROLL], 0.0862f / SPEED_THRESHOLD[YAW]};

  void Platform::frictionCompensation()
  {
#if (MODEL_TYPE == LINEAR_MODEL)
  quadraticRegression();

  float dry_friction[NB_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float viscous_friction[NB_VISCOUS_COMP] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float stiction[NB_SIGN_COMP][NB_STICTION_COMP] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

  for (int axis_=0; axis_<NB_AXIS; axis_++)
  {
    if (axis_<NB_STICTION_COMP)
    {
      float dry_scale = 1.0f;
      uint w_sign = MID;
      
      if (_speed[axis_] >= SPEED_THRESHOLD[axis_])
      {
        w_sign = POS;   
      }
      else if (_speed[axis_] <= -SPEED_THRESHOLD[axis_])
      {
        w_sign = NEG;
      }
      else
      {
        w_sign = MID;
      }
      
      dry_scale = clamp(abs(SPEED_THRESHOLD[axis_] / (_speed[axis_] + 1e-10f)),0.95f,1.0f);
      stiction[NEG][axis_] = clamp( _compEffort[NEG](axis_), _effortCompLim[NEG](axis_, L_MIN), _effortCompLim[NEG](axis_, L_MAX));
      stiction[POS][axis_] = clamp( _compEffort[POS](axis_), _effortCompLim[POS](axis_, L_MIN), _effortCompLim[POS](axis_, L_MAX));

      if (w_sign == POS || w_sign == NEG)
      {
        dry_friction[axis_] = dry_scale * stiction[w_sign][axis_];
      }
      else
      {
        //! linear deazone
        float deadzone;
        //deadzone = map(smoothRise(_speed[axis_], -SPEED_THRESHOLD[axis_], SPEED_THRESHOLD[axis_]),0.0f,1.0f, dry_scale * dry_scale * stiction[NEG][axis_], dry_scale * dry_scale * stiction[POS][axis_]);
        // probabilistic random deadzone
        deadzone = map(smoothRise(((float)(rand() % 10) - 5.0f), -5.0f, 5.0f), 0.0f, 1.0f, 0.7 * stiction[NEG][axis_], 0.7 * stiction[POS][axis_]);
        dry_friction[axis_] = comp_filter[axis_].update(deadzone);
      }
    }
    // Viscous friction compensation

    viscous_friction[PITCH] = clamp(_speed[PITCH] * viscous_k[PITCH], -0.6f, 0.3f);
    viscous_friction[ROLL] = clamp(_speed[ROLL] * viscous_k[ROLL], -0.1f, 0.1f);
    viscous_friction[YAW] = clamp(_speed[YAW] * viscous_k[YAW], -0.1f, 0.08f);

    _effortD_ADD[COMPENSATION][axis_] = dry_friction[axis_] + viscous_friction[axis_];
  }

  


#endif           
}

void Platform::quadraticRegression()
{
  for (int sign_=0; sign_<NB_STICTION_COMP; sign_++)
  {
    for (int i=0; i<NB_STICTION_COMP; i++)
    {
      _predictors[sign_].col(i) << _position[X], _position[Y], DEG_TO_RAD * _position[PITCH], DEG_TO_RAD * _position[ROLL], DEG_TO_RAD * _position[YAW], _position[X] * _position[X], _position[Y] * _position[Y], DEG_TO_RAD * _position[PITCH] * DEG_TO_RAD * _position[PITCH], DEG_TO_RAD * _position[ROLL] * DEG_TO_RAD * _position[ROLL], DEG_TO_RAD * _position[YAW] * DEG_TO_RAD * _position[YAW];
    }
    _predictors[sign_] = (_predictors[sign_] - _mean[sign_]).cwiseProduct(_stdInv[sign_]);
    //! main formula!
    _compEffort[sign_] << (_betas[sign_].transpose() * _predictors[sign_]).diagonal() + _bias[sign_].transpose();
    }
}
