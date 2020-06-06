#include "Platform.h"
#include "LP_Filter.h"
#include "definitions.h"
#include "definitions_2.h"

//float const SPEED_THRESHOLD[NB_AXIS] = {0.004f, 0.0035f, 0.05f, 0.09f, 0.09f};
//float const SPEED_THRESHOLD[NB_AXIS] = {0.006f, 0.005f, 0.07f, 0.12f, 0.12f};
float const SPEED_THRESHOLD[NB_AXIS] = {0.010f, 0.010f, 0.09f, 0.12f, 0.12f};
float const VIS_EFFORT_LIMS[NB_AXIS][NB_LIMS] = {{-2.0f, 2.0f}, {-2.0f, 2.0f}, {-0.6f, 0.3f}, {-0.5f, 0.5f}, {-0.5f, 0.5f}};
float const GRAVITY_EFFORT_LIMS[NB_AXIS][NB_LIMS] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {-2.0f, 2.0f}, {-2.0f, 2.0f}, {-2.0f, 2.0f}};
float const INERTIA_EFFORT_LIMS[NB_AXIS][NB_LIMS] = {{-3.0f, 3.0f}, {-3.0f, 3.0f}, {-0.5f, 0.5f}, {-0.5f, 0.5f}, {-0.5f, 0.5f}};
float const DRY_EFFORT_LIMS[NB_SIGN_COMP][NB_AXIS][NB_LIMS] = {{{-8.55883f, -1.47001f}, {-16.0498f, -3.10896f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}},
                                                               {{0.875992f,6.60670f}, {1.90903f, 15.5236f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}}};

float const VISCOUS_K[NB_AXIS] = {3.0f, 3.0f, 46.0734f * DEG_TO_RAD,
                                  62.3174f * DEG_TO_RAD, 0.0f * DEG_TO_RAD};

float const INERTIA_K[NB_AXIS] = {13.7704f, 13.6178f, 0.2521f * DEG_TO_RAD,
                                   0.1831f* DEG_TO_RAD, 0.1867f * DEG_TO_RAD};

//! variables from Platform_model.cpp

extern float LINKS_COM[NB_LINKS][2];
extern float LINKS_MASS[NB_LINKS];
extern const float r3;
extern const float d6; 
extern const float d7; 
extern const float r8;


LP_Filter comp_filter[NB_STICTION_COMP]{0.9f, 0.9f};


using namespace std;
using namespace Eigen;


//! 1

void Platform::dynamicCompensation()
{
  
  _compensationEffort.setConstant(0.0f);
  gravityCompensation();
  dryFrictionCompensation();
  viscFrictionCompensation();
  inertiaCompensation();
  
  _effortD_ADD.col(COMPENSATION) = _compensationEffort.rowwise().sum();
}

void Platform::gravityCompensation()
{
  
  float c_theta = cos(_position[PITCH] * DEG_TO_RAD);
  float c_phi = cos(_position[ROLL] * DEG_TO_RAD);
  float c_psi = cos(_position[YAW] * DEG_TO_RAD);
  float s_theta = sin(_position[PITCH] * DEG_TO_RAD);
  float s_phi = sin(_position[ROLL] * DEG_TO_RAD);
  float s_psi = sin(_position[YAW] * DEG_TO_RAD);

  Eigen::Matrix<float, 6,1> gravityTauFSWRTBase; //! force torque of the weight of the center of mass in the frame of the forcesensor w.r.t base
  Eigen::Vector3f coMPitchPedalWRTBase, coMtoFSVector;
  Eigen::Vector3f weightForce, weightTorque;
  Eigen::Vector3f gravityCompTorque;

  gravityTauFSWRTBase.setConstant(0.0f);
  weightForce.setConstant(0.0f);
  weightTorque.setConstant(0.0f);
  gravityCompTorque.setConstant(0.0f); //! Required torque to compensate
  coMtoFSVector.setConstant(0.0f);
  coMPitchPedalWRTBase.setConstant(0.0f); //! net CoM of the pedal, roll, yaw and pitch links
  
  coMPitchPedalWRTBase = comLinkWRTBase(LINK_PEDAL)*_massLinks(LINK_PEDAL)+
                         comLinkWRTBase(LINK_YAW)*_massLinks(LINK_YAW)+
                         comLinkWRTBase(LINK_ROLL)*_massLinks(LINK_ROLL)+ 
                         comLinkWRTBase(LINK_PITCH)*_massLinks(LINK_PITCH);

  coMPitchPedalWRTBase/=_massLinks.tail(NB_LINKS - LINK_PITCH).sum();

  coMtoFSVector = coMPitchPedalWRTBase - positionFrame(FRAME_FS);

  weightForce(2)=_massLinks.tail(NB_LINKS - LINK_PITCH).sum() * GRAVITY;
  weightTorque = coMtoFSVector.cross(weightForce);

  gravityTauFSWRTBase.block(0,0,3,1) = weightForce;
  gravityTauFSWRTBase.block(3,0,3,1) = weightTorque;

  _gravityCompJointsTorque = geometricJacobian(FRAME_FS).transpose() * -gravityTauFSWRTBase;

  // _effortM[1] = _gravityCompJointsTorque(PITCH);
  // _effortM[2] = _gravityCompJointsTorque(ROLL);
  // _effortM[3] = _gravityCompJointsTorque(YAW);

  for (int axis_ = PITCH; axis_ < NB_AXIS; axis_++)
  {
    _compensationEffort.col(COMP_GRAVITY)(axis_) = clip(_gravityCompJointsTorque(axis_), GRAVITY_EFFORT_LIMS[axis_][L_MIN], GRAVITY_EFFORT_LIMS[axis_][L_MAX]);
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
          
          if (_speed(axis_) >= SPEED_THRESHOLD[axis_])
          {
            w_sign = POS;   
          }
          else if (_speed(axis_) <= -SPEED_THRESHOLD[axis_])
          {
            w_sign = NEG;
          }
          else
          {
            w_sign = MID;
          }
          
          dry_scale = clip(abs(SPEED_THRESHOLD[axis_] / (_speed(axis_) + 1e-10f)),0.95f,1.0f);

          _dryFrictionEffortSign[NEG](axis_) = clip(_dryFrictionEffortSign[NEG](axis_), DRY_EFFORT_LIMS[NEG][axis_][L_MIN], DRY_EFFORT_LIMS[NEG][axis_][L_MAX]);
          _dryFrictionEffortSign[POS](axis_) = clip(_dryFrictionEffortSign[POS](axis_), DRY_EFFORT_LIMS[POS][axis_][L_MIN], DRY_EFFORT_LIMS[POS][axis_][L_MAX]);

          if (w_sign == POS || w_sign == NEG)
          {
            _compensationEffort(axis_, COMP_DRY_FRICTION) = dry_scale * _dryFrictionEffortSign[w_sign][axis_];
          }
          else
          {
            //! probabilistic deadzone
            float deadzone;
            deadzone = map(smoothRise(((float)(rand() % 10) - 5.0f), -5.0f, 5.0f), 0.0f, 1.0f, 0.7 * _dryFrictionEffortSign[NEG][axis_], 0.7 * _dryFrictionEffortSign[POS][axis_]);
            _compensationEffort(axis_,COMP_DRY_FRICTION) = comp_filter[axis_].update(deadzone);
            //! linear deazone
            //!_compensationEffort(axis_,COMP_DRY_FRICTION) = map(_speed(axis_), -SPEED_THRESHOLD[axis_],SPEED_THRESHOLD[axis_], 0.7f*_dryFrictionEffortSign[NEG][axis_], 0.7f*_dryFrictionEffortSign[POS][axis_]);
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
        _compensationEffort(axis_,COMP_VISC_FRICTION) = clip(_speed(axis_) * VISCOUS_K[axis_], VIS_EFFORT_LIMS[axis_][L_MIN], VIS_EFFORT_LIMS[axis_][L_MAX]);
      }
    }
  }

  void Platform::inertiaCompensation()
  {
    // Inertia compensation

    // D(q) = sum (mi*Jvi*Jvi' + Jwi'*Ri*Ii*Ri'*Jwi)



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
