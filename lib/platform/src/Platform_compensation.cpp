#include <Platform.h>
#include <LP_Filter.h>
#include <definitions.h>
#include <definitions_2.h>

float const SPEED_THRESHOLD[NB_AXIS] = {0.0035f, 0.004f, 0.05f, 0.09f, 0.09f};
float const MIN_SPEED[NB_AXIS] = {0.00002f, 0.00002f, 0.00002f, 0.02f, 0.02f};
float const VIS_EFFORT_LIMS[NB_AXIS][NB_LIMS] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {-0.6f, 0.3f}, {-0.5f, 0.5f}, {-0.5f, 0.5f}};
float const GRAVITY_EFFORT_LIMS[NB_AXIS][NB_LIMS] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {-2.0f, 2.0f}, {-2.0f, 2.0f}, {-2.0f, 2.0f}};
float const INERTIA_EFFORT_LIMS[NB_AXIS][NB_LIMS] = {{-4.0f, 4.0f}, {-3.5f, 3.5f}, {-0.2f, 0.2f}, {-0.2f, 0.2f}, {-0.2f, 0.2f}};
float const DRY_EFFORT_LIMS[NB_SIGN_COMP][NB_AXIS][NB_LIMS] = {{{2.5*-8.55883f, -1.47001f}, {-16.0498f, -3.10896f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}},
                                                               {{0.875992f,1.5*6.60670f}, {1.90903f, 15.5236f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}}};

float const VISCOUS_K[NB_AXIS] = {0.0f, 0.0f, 46.0734f * DEG_TO_RAD,
                                  62.3174f * DEG_TO_RAD, 0.0f * DEG_TO_RAD};

float const INERTIA_K[NB_AXIS] = {13.7704f, 13.6178f, 0.2521f * DEG_TO_RAD,
                                  0.1831f* DEG_TO_RAD, 0.1867f * DEG_TO_RAD};

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
  // const float CoMDistanceSensorFromAxis[3] = {0.0f,0.01f,0.034f}; //[m]
  const float CoMDistanceSensorFromAxis[3] = {0.0f, 0.001f, 0.110f}; //[m]
  const float CoMMassSensor = 1.2f-0.293f;
#endif

//! 1

void Platform::dynamicCompensation()
{

  gravityCompensation();
  dryFrictionCompensation();
  viscFrictionCompensation();
  inertiaCompensation();
  
  _effortD_ADD.col(COMPENSATION) = _compensationEffort.rowwise().sum();
}

void Platform::gravityCompensation()
{
  
  Eigen::Vector3f Torque, Weight, CoMVector;
  Eigen::Matrix3f R_pitch, R_roll, R_yaw, R_tot;
  //CoMVector << 0.0, 0.0, CoMDistanceSensorFromAxis;
  CoMVector << CoMDistanceSensorFromAxis[0], CoMDistanceSensorFromAxis[1], CoMDistanceSensorFromAxis[2];
  Weight << 0.0, 0.0, (CoMMassSensor * GRAVITY);

  float rad_pitch = _position[PITCH] * DEG_TO_RAD;
  float rad_roll = _position[ROLL] * DEG_TO_RAD;
  float rad_yaw = _position[YAW] * DEG_TO_RAD;

  R_pitch<<1.0f , 0.0f, 0.0f, 
            0.0f, cos(rad_pitch), -sin(rad_pitch),
            0.0f, sin(rad_pitch), cos(rad_pitch);

  R_roll << cos(rad_roll)  , 0.0f, sin(rad_roll),
              0.0f          , 1.0f, 0.0f,
            -sin(rad_roll), 0.0f ,cos(rad_roll);

  R_yaw << cos(rad_yaw),-sin(rad_yaw), 0.0f,
           sin(rad_yaw), cos(rad_yaw), 0.0f,
          0.0f, 0.0f, 1.0f;

  R_tot = R_pitch * R_roll * R_yaw;
  //R_tot = R_pitch * R_roll;

  Torque = - R_tot * (CoMVector.cross(R_tot.transpose()*Weight));

    for (int axis_=PITCH; axis_<NB_AXIS; axis_++)
    {
      _compensationEffort.col(COMP_GRAVITY)(axis_) = clip(Torque(axis_ - PITCH), GRAVITY_EFFORT_LIMS[axis_][L_MIN], GRAVITY_EFFORT_LIMS[axis_][L_MAX]);
    }
  }

#define LINEAR_MODEL 1
#define GAUSSIAN_PROCESS 2

#define MODEL_TYPE LINEAR_MODEL

// float const VISCOUS_K[NB_VISCOUS_COMP] = {0.0f, 0.0f, 0.0862f / SPEED_THRESHOLD[PITCH],
//                                             0.1859f / SPEED_THRESHOLD[ROLL], 0.0862f / SPEED_THRESHOLD[YAW]};

  void Platform::dryFrictionCompensation()
  {
    #if (MODEL_TYPE == LINEAR_MODEL)

    quadraticRegression();

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
          
          dry_scale = clip(abs(SPEED_THRESHOLD[axis_] / (_speed[axis_] + 1e-10f)),0.95f,1.0f);

          _dryFrictionEffortSign[NEG](axis_) = clip(_dryFrictionEffortSign[NEG](axis_), DRY_EFFORT_LIMS[NEG][axis_][L_MIN], DRY_EFFORT_LIMS[NEG][axis_][L_MAX]);
          _dryFrictionEffortSign[POS](axis_) = clip(_dryFrictionEffortSign[POS](axis_), DRY_EFFORT_LIMS[POS][axis_][L_MIN], DRY_EFFORT_LIMS[POS][axis_][L_MAX]);

          if (w_sign == POS || w_sign == NEG)
          {
            _compensationEffort(axis_, COMP_DRY_FRICTION) = dry_scale * _dryFrictionEffortSign[w_sign][axis_];
          }
          else
          {
            //! linear deazone
            float deadzone;
            //deadzone = map(smoothRise(_speed[axis_], -SPEED_THRESHOLD[axis_], SPEED_THRESHOLD[axis_]),0.0f,1.0f, dry_scale * dry_scale * stiction[NEG][axis_], dry_scale * dry_scale * stiction[POS][axis_]);
            // probabilistic random deadzone
            deadzone = map(smoothRise(((float)(rand() % 10) - 5.0f), -5.0f, 5.0f), 0.0f, 1.0f, 0.7 * _dryFrictionEffortSign[NEG][axis_], 0.7 * _dryFrictionEffortSign[POS][axis_]);
            _compensationEffort(axis_,COMP_DRY_FRICTION) = comp_filter[axis_].update(deadzone);
          }
        }
      }
    #endif
  }

  void Platform::viscFrictionCompensation()
  {
      // Viscous friction compensation
    for (int axis_=0; axis_<NB_AXIS; axis_++)
    {
      if (abs(_position[axis_]) <= COMP_WS_LIMITS[axis_]) // only if inside the desired workspace
      {
        _compensationEffort(axis_,COMP_VISC_FRICTION) = clip(_speed[axis_] * VISCOUS_K[axis_], VIS_EFFORT_LIMS[axis_][L_MIN], VIS_EFFORT_LIMS[axis_][L_MAX]);
      }
    }

  }

  void Platform::inertiaCompensation()
  {
    // Inertia compensation
    for (int axis_ = 0; axis_ < NB_AXIS; axis_++)
    {
      if (abs(_position[axis_]) <= COMP_WS_LIMITS[axis_]) // only if inside the desired workspace
      {
        _compensationEffort(axis_, COMP_INERTIA) = clip(_acceleration[axis_] * INERTIA_K[axis_], INERTIA_EFFORT_LIMS[axis_][L_MIN], INERTIA_EFFORT_LIMS[axis_][L_MAX]);
      }
    }
  }

void Platform::quadraticRegression()
{
  for (int sign_ = 0; sign_ < NB_STICTION_COMP; sign_++)
  {
    for (int i = 0; i < NB_STICTION_COMP; i++)
    {
      _predictors[sign_].col(i) << _position[X], _position[Y], DEG_TO_RAD * _position[PITCH], DEG_TO_RAD * _position[ROLL], DEG_TO_RAD * _position[YAW], _position[X] * _position[X], _position[Y] * _position[Y], DEG_TO_RAD * _position[PITCH] * DEG_TO_RAD * _position[PITCH], DEG_TO_RAD * _position[ROLL] * DEG_TO_RAD * _position[ROLL], DEG_TO_RAD * _position[YAW] * DEG_TO_RAD * _position[YAW];
    }
    _predictors[sign_] = (_predictors[sign_] - _mean[sign_]).cwiseProduct(_stdInv[sign_]);
    //! main formula!
    _dryFrictionEffortSign[sign_] << (_betas[sign_].transpose() * _predictors[sign_]).diagonal() + _bias[sign_].transpose();
  }
}
