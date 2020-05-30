#include <Platform.h>
#include <LP_Filter.h>
#include <definitions.h>
#include <definitions_2.h>

float const SPEED_THRESHOLD[NB_AXIS] = {0.004f, 0.0035f, 0.05f, 0.09f, 0.09f};
//float const SPEED_THRESHOLD[NB_AXIS] = {0.006f, 0.005f, 0.07f, 0.12f, 0.12f};
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


LP_Filter comp_filter[NB_STICTION_COMP]{0.7f, 0.7f};


using namespace std;
using namespace Eigen;


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
  
  float c_theta = cos(_position[PITCH] * DEG_TO_RAD);
  float c_phi = cos(_position[ROLL] * DEG_TO_RAD);
  float c_psi = cos(_position[YAW] * DEG_TO_RAD);
  float s_theta = sin(_position[PITCH] * DEG_TO_RAD);
  float s_phi = sin(_position[ROLL] * DEG_TO_RAD);
  float s_psi = sin(_position[YAW] * DEG_TO_RAD);

  Eigen::Matrix<float, 6,1> gravityTauFSWRTBase; //! force torque of the weight of the center of mass in the frame of the forcesensor w.r.t base
  Eigen::Vector3f coMPitchPedalWRTBase, coMtoFSVector;
  Eigen::Vector3f weightForce, weightTorque;
  Eigen::Vector4f massPitchPedal;
  Eigen::Vector3f gravityCompTorque;

  gravityTauFSWRTBase.setConstant(0.0f);
  weightForce.setConstant(0.0f);
  weightTorque.setConstant(0.0f);
  massPitchPedal.setConstant(0.0f);
  gravityCompTorque.setConstant(0.0f); //! Required torque to compensate
  coMtoFSVector.setConstant(0.0f);
  coMPitchPedalWRTBase.setConstant(0.0f); //! net CoM of the pedal, roll, yaw and pitch links

  massPitchPedal<< LINKS_MASS[LINK_PITCH], 
                   LINKS_MASS[LINK_ROLL],
                   LINKS_MASS[LINK_YAW],
                   LINKS_MASS[LINK_PEDAL];
  
  coMPitchPedalWRTBase = comLinkWRTBase(LINK_PEDAL)*massPitchPedal(LINK_PEDAL-LINK_PITCH)+
                         comLinkWRTBase(LINK_YAW)*massPitchPedal(LINK_YAW-LINK_PITCH)+
                         comLinkWRTBase(LINK_ROLL)*massPitchPedal(LINK_ROLL-LINK_PITCH)+ 
                         comLinkWRTBase(LINK_PITCH)*massPitchPedal(LINK_PITCH-LINK_PITCH);

  coMPitchPedalWRTBase/=massPitchPedal.sum();

  coMtoFSVector = coMPitchPedalWRTBase - positionFrame(FRAME_FS);

  weightForce(2)=massPitchPedal.sum() * GRAVITY;
  weightTorque = coMtoFSVector.cross(weightForce);

  gravityTauFSWRTBase.block(0,0,3,1) = weightForce;
  gravityTauFSWRTBase.block(3,0,3,1) = weightTorque;

  _gravityCompJointsTorque = geometricJacobian(FRAME_FS).transpose() * -gravityTauFSWRTBase;

  // 

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
            //! probabilistic deazone
            // float deadzone;
            //! deadzone = map(smoothRise(((float)(rand() % 10) - 5.0f), -5.0f, 5.0f), 0.0f, 1.0f, 0.7 * _dryFrictionEffortSign[NEG][axis_], 0.7 * _dryFrictionEffortSign[POS][axis_]);
            //! _compensationEffort(axis_,COMP_DRY_FRICTION) = comp_filter[axis_].update(deadzone);
            
            //! linear deazone
            _compensationEffort(axis_,COMP_DRY_FRICTION) = map(_speed[axis_], -SPEED_THRESHOLD[axis_],SPEED_THRESHOLD[axis_], 0.7f*_dryFrictionEffortSign[NEG][axis_], 0.7f*_dryFrictionEffortSign[POS][axis_]);
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








// #define s1 0.37289094873267458751797676086426f
// // #define s2 0.00017838561681537967729127558413893f
// // #define s3 0.1202649827939694660017266869545f
// // #define s4 5.3278916881681652739644050598145f
// // #define s5 0.060133779052037886837501552679998f
// // #define s6 1.5642521308821102975981439484377f
// // #define s7 0.00078704884439950717478495789691806f
// // #define s8 0.00029624205462752684070437680929899f
// // #define s9 6.2129561752277892082929611206055f
// // #define s10 0.098287905464530922472476959228516f
// // #define s11 5.3278916964040078922555905394144f

// // gravityCompTorque(0) = s1*s_theta - s2*c_theta  + s3*c_psi*c_theta + s4*c_phi*s_theta + s7*c_theta*s_psi - s8*s_phi*s_theta - s9*c_phi*s_theta - s3*s_phi*s_psi*s_theta + s7*c_psi*s_phi*s_theta;
// // gravityCompTorque(1) = c_theta*( - s5*cos(_position[ROLL]*DEG_TO_RAD + _position[YAW]*DEG_TO_RAD + s6) - s9*s_phi + s11*cos(_position[ROLL]*DEG_TO_RAD - M_PI_2) - s5*cos(_position[YAW]*DEG_TO_RAD - 1.0f*_position[ROLL]*DEG_TO_RAD + s6) + s10);
// // gravityCompTorque(2) = s_phi*(s1*s_theta - s2*c_theta  + s3*c_psi*c_theta + s4*c_phi*s_theta + s7*c_theta*s_psi - s8*s_phi*s_theta - s3*s_phi*s_psi*s_theta + s7*c_psi*s_phi*s_theta) - 1.0f*c_phi*s_theta*( + s8*c_phi + s4*s_phi - s7*c_phi*c_psi + s3*c_phi*s_psi + s10);

// _gravityCompJointsTorque(PITCH) =  GRAVITY * (LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PITCH] * c_theta -
//                                   LINKS_COM[LINK_ROLL][2] * LINKS_MASS[LINK_ROLL]* c_theta +
//                                   LINKS_COM[LINK_PITCH][0] * LINKS_MASS[LINK_PITCH] * s_theta +
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][2] * c_phi * s_theta +
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][0] * c_theta * s_psi -
//                                   LINKS_COM[LINK_ROLL][1] * LINKS_MASS[LINK_ROLL] * s_phi * s_theta +
//                                   LINKS_COM[LINK_PEDAL][0] * LINKS_MASS[LINK_PEDAL] * c_psi * c_theta -
//                                   d6 * LINKS_MASS[LINK_PITCH] * c_phi * s_theta -
//                                   d6 * LINKS_MASS[LINK_ROLL] * c_phi * s_theta -
//                                   d6 * LINKS_MASS[LINK_YAW] * c_phi * s_theta +
//                                   LINKS_COM[LINK_PEDAL][2] * LINKS_MASS[LINK_PEDAL] * c_phi * s_theta -
//                                   LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PEDAL] * c_theta * s_psi +
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][1] * c_psi * c_theta +
//                                   LINKS_COM[LINK_ROLL][0] * LINKS_MASS[LINK_ROLL] * c_phi * s_theta -
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][1] * s_phi * s_psi * s_theta -
//                                   LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PEDAL] * c_psi * s_phi * s_theta -
//                                   LINKS_COM[LINK_PEDAL][0] * LINKS_MASS[LINK_PEDAL] * s_phi * s_psi * s_theta +
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][0] * c_psi * s_phi * s_theta)   +
//                                   GRAVITY * c_phi * s_theta * d6 * (LINKS_MASS[LINK_PEDAL] + LINKS_MASS[LINK_PITCH] +
//                                   LINKS_MASS[LINK_ROLL] + LINKS_MASS[LINK_YAW]);

// _gravityCompJointsTorque(ROLL) =  GRAVITY * c_theta * (LINKS_COM[LINK_PITCH][2] * LINKS_MASS[LINK_PITCH] -
//                                   d6 * LINKS_MASS[LINK_PITCH] * s_phi +
//                                   LINKS_COM[LINK_ROLL][1] * LINKS_MASS[LINK_ROLL] * c_phi -
//                                   d6 * LINKS_MASS[LINK_ROLL] * s_phi -
//                                   d6 * LINKS_MASS[LINK_YAW] * s_phi +
//                                   LINKS_COM[LINK_PEDAL][2] * LINKS_MASS[LINK_PEDAL] * s_phi +
//                                   LINKS_COM[LINK_ROLL][0] * LINKS_MASS[LINK_ROLL] * s_phi +
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][2] * s_phi +
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][1] * c_phi * s_psi +
//                                   LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PEDAL] * c_phi * c_psi +
//                                   LINKS_COM[LINK_PEDAL][0] * LINKS_MASS[LINK_PEDAL] * c_phi * s_psi -
//                                   LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][0] * c_phi * c_psi) +
//                                   GRAVITY * c_theta * s_phi * d6 * (LINKS_MASS[LINK_PEDAL] + LINKS_MASS[LINK_PITCH] +
//                                   LINKS_MASS[LINK_ROLL] + LINKS_MASS[LINK_YAW]);

// _gravityCompJointsTorque(YAW) =  GRAVITY * s_phi * (LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PITCH] * c_theta -
//                                  LINKS_COM[LINK_ROLL][2] * LINKS_MASS[LINK_ROLL] * c_theta +
//                                  LINKS_COM[LINK_PITCH][0] * LINKS_MASS[LINK_PITCH] * s_theta +
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][2] * c_phi * s_theta +
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][0] * c_theta * s_psi -
//                                  LINKS_COM[LINK_ROLL][1] * LINKS_MASS[LINK_ROLL] * s_phi * s_theta +
//                                  LINKS_COM[LINK_PEDAL][0] * LINKS_MASS[LINK_PEDAL] * c_psi * c_theta -
//                                  d6 * LINKS_MASS[LINK_PITCH] * c_phi * s_theta -
//                                  d6 * LINKS_MASS[LINK_ROLL] * c_phi * s_theta -
//                                  d6 * LINKS_MASS[LINK_YAW] * c_phi * s_theta +
//                                  LINKS_COM[LINK_PEDAL][2] * LINKS_MASS[LINK_PEDAL] * c_phi * s_theta -
//                                  LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PEDAL] * c_theta * s_psi +
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][1] * c_psi * c_theta +
//                                  LINKS_COM[LINK_ROLL][0] * LINKS_MASS[LINK_ROLL] * c_phi * s_theta -
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][1] * s_phi * s_psi * s_theta -
//                                  LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PEDAL] * c_psi * s_phi * s_theta -
//                                  LINKS_COM[LINK_PEDAL][0] * LINKS_MASS[LINK_PEDAL] * s_phi * s_psi * s_theta +
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][0] * c_psi * s_phi * s_theta) -
//                                  GRAVITY * c_phi * s_theta * (LINKS_COM[LINK_PITCH][2] * LINKS_MASS[LINK_PITCH] -
//                                  d6 * LINKS_MASS[LINK_PITCH] * s_phi + LINKS_COM[LINK_ROLL][1] * LINKS_MASS[LINK_ROLL] * c_phi -
//                                  d6 * LINKS_MASS[LINK_ROLL] * s_phi -  d6 * LINKS_MASS[LINK_YAW] * s_phi +
//                                  LINKS_COM[LINK_PEDAL][2] * LINKS_MASS[LINK_PEDAL] * s_phi +
//                                  LINKS_COM[LINK_ROLL][0] * LINKS_MASS[LINK_ROLL] * s_phi +
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][2] * s_phi +
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][1] * c_phi * s_psi +
//                                  LINKS_COM[LINK_PEDAL][1] * LINKS_MASS[LINK_PEDAL] * c_phi * c_psi +
//                                  LINKS_COM[LINK_PEDAL][0] * LINKS_MASS[LINK_PEDAL] * c_phi * s_psi -
//                                  LINKS_MASS[LINK_YAW] * LINKS_COM[LINK_YAW][0] * c_phi * c_psi);
