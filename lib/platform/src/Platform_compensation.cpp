#include "Platform.h"
#include "LP_Filter.h"
#include "MatLP_Filter.h"
#include "definitions.h"
#include "definitions_2.h"

float const SPEED_THRESHOLD[NB_AXIS] = {0.010f, 0.010f, 0.09f,
                                        0.12f * DEG_TO_RAD, 0.12f * DEG_TO_RAD};
extern float const VISC_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-2.0f, -2.0f , -0.6f, -0.5f, -0.5f}, {2.0f, 0.3f, 0.5f,0.5f}};
extern float const GRAVITY_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{0.0f, 0.0f, -2.0f, -2.0f, -2.0f}, {0.0f, 2.0f, 2.0f, 2.0f}};
extern float const INERTIA_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-3.0f,-3.0f,-0.5f,-0.5f,-0.5f}, {3.0f, 3.0f, 0.5f, 0.5f, 0.5f}};
extern float const CORIOLIS_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-3.0f,-3.0f,-0.5f,-0.5f,-0.5f}, {3.0f, 3.0f, 0.5f, 0.5f, 0.5f}};
extern float const DRY_EFFORT_LIMS[NB_SIGN_COMP][NB_LIMS][NB_AXIS] = {{{-16.0498f,-8.55883f,0.0f,0.0f,0.0f}, { -3.10896f, -1.47001f, 0.0f, 0.0f, 0.0f}},
                                                               {{1.90903f,0.875992f,0.0f,0.0f,0.0f},{15.5236f, 6.60670f, 0.0f, 0.0f, 0.0f}}};

extern const float invSpeedSampT;
//! variables from Platform_model.cpp

extern float LINKS_COM[NB_LINKS][NB_CART_AXIS];
extern float LINKS_MASS[NB_LINKS];
extern const float r3;
extern const float d6; 
extern const float d7; 
extern const float r8;

//! Filter of the probabilistic deadzone
LP_Filter comp_filter[NB_STICTION_COMP]{0.9f, 0.9f};

#define alphaMatrices 0.5f

#if (CORIOLIS_DEV_STRATEGY==CORIOLIS_TEMPORAL)
MatLP_Filter filter_devLinkCOMGeometricJ[NB_LINKS]{alphaMatrices, alphaMatrices, alphaMatrices, alphaMatrices, alphaMatrices,alphaMatrices,alphaMatrices}; //! one alpha per link
MatLP_Filter filter_devRotationMatrixCOM[NB_LINKS]{alphaMatrices, alphaMatrices, alphaMatrices, alphaMatrices, alphaMatrices,alphaMatrices,alphaMatrices}; //! one alpha per link
#endif
//! Filters for the derivatives of the rotation and jacobian matrices;
extern const float filterdevMatrices =
        0.3; // for jacobian and rotation matrices

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
  coriolisCompensation(); //! coriolis has to be compensated after the inertia;

  //! Saturation of the compensation effort except the dry friction
  _compensationEffort.block(0,0,NB_AXIS,NB_COMPENSATION_COMP-1) =
      boundMat(_compensationEffort.block(0,0,NB_AXIS,NB_COMPENSATION_COMP-1), _compTorqueLims[L_MIN], _compTorqueLims[L_MAX]);

  _effortD_ADD.col(COMPENSATION) = _compensationEffort.rowwise().sum();
}

void Platform::gravityCompensation()
{
  
  Eigen::Matrix<float,NB_AXIS,1> gravityCompTorque;
  Eigen::Matrix<float,6,1> gravityTau;

  gravityTau<< 0.0f, 0.0f, GRAVITY, 0.0f, 0.0f, 0.0f; 

  gravityCompTorque.setConstant(0.0f); //! Required torque to compensate
  
  for (int link_= LINK_BASE; link_<NB_LINKS; link_++)
  {
    gravityCompTorque+= - comGeometricJacobian( (link_chain) link_).transpose()*(_massLinks(link_)*gravityTau);
  }

  _compensationEffort.col(COMP_GRAVITY) = gravityCompTorque;

}
  void Platform::viscFrictionCompensation()
  {

    _compensationEffort.col(COMP_VISC_FRICTION) = _jointsViscosityMat * _speed;
  }

  void Platform::inertiaCompensation()
  {

    Eigen::Matrix<float, NB_AXIS, NB_AXIS> inertiaJointGainMatrix; //! M(q)
    
    for (int link_ = LINK_BASE; link_ < NB_LINKS; link_++) {
      _linkCOMGeomJacobian[link_] = comGeometricJacobian( (link_chain) link_);
      _rotationMatrixCOM[link_] = comRotationMatrix((link_chain) link_);

      inertiaJointGainMatrix+= 
      - (
        _massLinks(link_)*
        _linkCOMGeomJacobian[link_].block(0,0,NB_CART_AXIS,NB_AXIS).transpose()*
        _linkCOMGeomJacobian[link_].block(0,0,NB_CART_AXIS,NB_AXIS)
        +
        _linkCOMGeomJacobian[link_].block(NB_CART_AXIS,0,NB_CART_AXIS,NB_AXIS).transpose()*
                    (_rotationMatrixCOM[link_]*
                    _momentInertiaLinks[link_]*
                    _rotationMatrixCOM[link_].transpose())*
        _linkCOMGeomJacobian[link_].block(NB_CART_AXIS,0,NB_CART_AXIS,NB_AXIS)
        );
    }

    _compensationEffort.col(COMP_INERTIA) = inertiaJointGainMatrix*_acceleration;

   }

   void Platform::coriolisCompensation() {
     // From bjerkeng2012
     Eigen::Matrix<float, NB_AXIS, NB_AXIS> coriolisJointGainMatrix; //! V(q,q)
          
     coriolisJointGainMatrix.setConstant(0.0f);
     
    #if (CORIOLIS_DEV_STRATEGY==CORIOLIS_KRONECKER)
        Eigen::Matrix<float,NB_AXIS*NB_AXIS,NB_AXIS> kronProductSpeed;
        kronProductSpeed.setZero();
        kronProductSpeed = kroneckerProductEye(_speed);
    #endif

     for (int link_ = LINK_BASE; link_ < NB_LINKS; link_++) {

    #if (CORIOLIS_DEV_STRATEGY == CORIOLIS_TEMPORAL)
       if (_flagSpeedSampledForCoriolis){ // Calculate the derivative of the Jacobian and Rotation Matrices
         _devLinkCOMGeomJacobian[link_] = filter_devLinkCOMGeometricJ[link_].update((_linkCOMGeomJacobian[link_] - _linkCOMGeometricJ_prev[link_])*invSpeedSampT) ;
         _devRotationMatrixCOM[link_] = filter_devRotationMatrixCOM[link_].update((_rotationMatrixCOM[link_] - _rotationMatrixCOM_prev[link_])*invSpeedSampT) ;
         _rotationMatrixCOM_prev[link_] = _rotationMatrixCOM[link_];
         _linkCOMGeometricJ_prev[link_] = _linkCOMGeomJacobian[link_];
         _flagSpeedSampledForCoriolis = false;
       }

    #else
       
       _devLinkCOMGeomJacobian[link_] = devQComGeomJacobian((link_chain) link_) * kronProductSpeed;  
       _devRotationMatrixCOM[link_] =   devQComRotationMatrix((link_chain) link_) * kronProductSpeed.block(0,0,NB_AXIS*3,3);  

    #endif


       coriolisJointGainMatrix +=
           -(_massLinks(link_)*
             _linkCOMGeomJacobian[link_].block(0,0,NB_CART_AXIS,NB_AXIS).transpose()*
             _devLinkCOMGeomJacobian[link_].block(0,0,NB_CART_AXIS,NB_AXIS)
            +
            _linkCOMGeomJacobian[link_].block(NB_CART_AXIS,0,NB_CART_AXIS,NB_AXIS).transpose()*
                        (_rotationMatrixCOM[link_]*
                        _momentInertiaLinks[link_]*
                        _rotationMatrixCOM[link_].transpose())*
            _devLinkCOMGeomJacobian[link_].block(NB_CART_AXIS,0,NB_CART_AXIS,NB_AXIS)
            +
             _linkCOMGeomJacobian[link_].block(NB_CART_AXIS,0,NB_CART_AXIS,NB_AXIS).transpose()*
                 (_devRotationMatrixCOM[link_]*
                  _momentInertiaLinks[link_]*
                  _rotationMatrixCOM[link_].transpose()) *
             _linkCOMGeomJacobian[link_].block(NB_CART_AXIS,0,NB_CART_AXIS,NB_AXIS)
           );
     }

     _compensationEffort.col(COMP_CORIOLIS) =
         coriolisJointGainMatrix * _speed;
   }

   /**************************************FRICTION COMPENSATION*************************/
#define LINEAR_MODEL 1
#define GAUSSIAN_PROCESS 2

#define MODEL_TYPE LINEAR_MODEL

   void Platform::dryFrictionCompensation() {
#if (MODEL_TYPE == LINEAR_MODEL)

     quadraticRegression();

     for (int axis_ = 0; axis_ < NB_AXIS; axis_++) {
       if (axis_ < NB_STICTION_COMP) {
         float dry_scale = 1.0f;
         uint w_sign = MID;

         if (_speed(axis_) >= SPEED_THRESHOLD[axis_]) {
           w_sign = POS;
         } else if (_speed(axis_) <= -SPEED_THRESHOLD[axis_]) {
           w_sign = NEG;
         } else {
           w_sign = MID;
         }

         dry_scale =
             clip(abs(SPEED_THRESHOLD[axis_] / (_speed(axis_) + 1e-10f)), 0.95f,
                  1.0f);

         _dryFrictionEffortSign[NEG](axis_) =
             clip(_dryFrictionEffortSign[NEG](axis_),
                  DRY_EFFORT_LIMS[NEG][L_MIN][axis_],
                  DRY_EFFORT_LIMS[NEG][L_MAX][axis_]);
         _dryFrictionEffortSign[POS](axis_) =
             clip(_dryFrictionEffortSign[POS](axis_),
                  DRY_EFFORT_LIMS[POS][L_MIN][axis_],
                  DRY_EFFORT_LIMS[POS][L_MAX][axis_]);

         if (w_sign == POS || w_sign == NEG) {
           _compensationEffort(axis_, COMP_DRY_FRICTION) =
               dry_scale * _dryFrictionEffortSign[w_sign][axis_];
         } else {
           //! probabilistic deadzone
           float deadzone;
           deadzone =
               map(smoothRise(((float)(rand() % 10) - 5.0f), -5.0f, 5.0f), 0.0f,
                   1.0f, 0.7 * _dryFrictionEffortSign[NEG][axis_],
                   0.7 * _dryFrictionEffortSign[POS][axis_]);
           _compensationEffort(axis_, COMP_DRY_FRICTION) =
               comp_filter[axis_].update(deadzone);
           //! linear deazone
           //!_compensationEffort(axis_,COMP_DRY_FRICTION) = map(_speed(axis_),
           //!-SPEED_THRESHOLD[axis_],SPEED_THRESHOLD[axis_],
           //!0.7f*_dryFrictionEffortSign[NEG][axis_],
           //!0.7f*_dryFrictionEffortSign[POS][axis_]);
         }
       }
     }
#endif
   }

void Platform::quadraticRegression() {
     for (int sign_ = 0; sign_ < NB_STICTION_COMP; sign_++) {

       _predictors[sign_].block(0, 0, NB_AXIS, 1) = _position;
       _predictors[sign_].block(NB_AXIS, 0, 2 * NB_AXIS, 1) =
           _position.cwiseProduct(_position);
       _predictors[sign_] =
           (_predictors[sign_] - _mean[sign_]).cwiseProduct(_stdInv[sign_]);
       //! main formula!
       _dryFrictionEffortSign[sign_] =
           (_betas[sign_].transpose() * _predictors[sign_]).diagonal() +
           _bias[sign_].transpose();
     }
}
