#include "Platform.h"
#include "LP_Filter.h"
#include "MatLP_Filter.h"
#include "definitions.h"
#include "definitions_2.h"

float const SPEED_THRESHOLD[NB_AXIS] = {0.010f, 0.010f, 0.09f * DEG_TO_RAD,
                                        0.12f * DEG_TO_RAD, 0.12f * DEG_TO_RAD};
extern float const VISC_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-2.0f, -2.0f , -0.6f, -0.5f, -0.5f}, {2.0f, 2.0f, 0.5f,0.5f}};
extern float const GRAVITY_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{0.0f, 0.0f, -2.0f, -2.0f, -2.0f}, {0.0f, 2.0f, 2.0f, 2.0f}};
extern float const INERTIA_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-3.0f,-3.0f,-0.5f,-0.5f,-0.5f}, {3.0f, 3.0f, 0.5f, 0.5f, 0.5f}};
extern float const CORIOLIS_EFFORT_LIMS[NB_LIMS][NB_AXIS] = {{-3.0f,-3.0f,-0.5f,-0.5f,-0.5f}, {3.0f, 3.0f, 0.5f, 0.5f, 0.5f}};
extern float const DRY_EFFORT_LIMS[NB_SIGN_COMP][NB_LIMS][NB_AXIS] = {{{-16.0498f,-8.55883f,0.0f,0.0f,0.0f}, { -3.10896f, -1.47001f, 0.0f, 0.0f, 0.0f}},
                                                               {{1.90903f,0.875992f,0.0f,0.0f,0.0f},{15.5236f, 6.60670f, 0.0f, 0.0f, 0.0f}}};

extern const float invSpeedSampT;
//! variables from Platform_model.cpp

extern float const LINKS_COM[NB_LINKS][NB_CART_AXIS];
extern float const LINKS_MASS[NB_LINKS];
extern const float r3;
extern const float d6; 
extern const float d7; 
extern const float r8;

//! Filter of the probabilistic deadzone
LP_Filter comp_filter[NB_STICTION_COMP]{0.9f, 0.9f};

#if(false)
#if (CORIOLIS_DEV_STRATEGY==CORIOLIS_TEMPORAL)
#define aMat 0.5f
#define lJMat 6
#define wJMat 5
#define lRMat 3
#define wRMat 3

MatLP_Filter filter_devLinkCOMGeometricJ[NB_LINKS]{{aMat,lJMat,wJMat}, {aMat,lJMat,wJMat}, {aMat,lJMat,wJMat}, {aMat,lJMat,wJMat}, {aMat,lJMat,wJMat},{aMat,lJMat,wJMat},{aMat,lJMat,wJMat}}; //! one alpha per link
MatLP_Filter filter_devRotationMatrixCOM[NB_LINKS]{{aMat,lRMat,wRMat}, {aMat,lRMat,wRMat}, {aMat,lRMat,wRMat}, {aMat,lRMat,wRMat}, {aMat,lRMat,wRMat},{aMat,lRMat,wRMat},{aMat,lRMat,wRMat}}; //! one alpha per link
#endif
//! Filters for the derivatives of the rotation and jacobian matrices;
#endif
using namespace std;
using namespace Eigen;


//! 1

void Platform::dynamicCompensation(const int* components_)
{
  if (*(components_+COMP_GRAVITY)==1)
    {gravityCompensation();}
  if (*(components_+COMP_VISC_FRICTION)==1)
    {viscFrictionCompensation();}
  if (*(components_+COMP_INERTIA)==1)
   { inertiaCompensation();}
  if (*(components_+COMP_CORIOLIS)==1){
    coriolisCompensation(); //! coriolis has to be compensated after the inertia;
                             //! coriolis is too computationally expensive       
  }
  //! Saturation of the compensation effort except the dry friction
  if (*(components_+COMP_DRY_FRICTION)==1)
    {dryFrictionCompensation();}
   
  _compensationEffort.block(0,0,NB_AXIS,NB_COMPENSATION_COMP-1) =
        boundMat(_compensationEffort.block(0,0,NB_AXIS,NB_COMPENSATION_COMP-1), _compTorqueLims[L_MIN], _compTorqueLims[L_MAX]);
  _effortD_ADD.col(COMPENSATION) = _compensationEffort.rowwise().sum();
}


#define MATRICIAL 0
#define EQUATIONS 1
#define COMPENSATION_TYPE MATRICIAL
#if (COMPENSATION_TYPE == MATRICIAL)
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

#else

void Platform::gravityCompensation() {
  _compensationEffort.col(COMP_GRAVITY).block(0,0,1,1) =  GRAVITY*_massLinks(LINK_PEDAL)*(LINKS_COM[LINK_PEDAL][0]*_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi)*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta + _c_phi*_s_theta*d6) + GRAVITY*_massLinks(LINK_YAW)*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi) + yw_x*_s_psi + yw_z*_c_phi*_s_theta + _s_phi*_s_theta*(yw_x*_c_psi) - LINKS_COM[LINK_YAW][1]*_s_psi) + GRAVITY*pitch_mass*(p_y*_c_theta + p_x*_s_theta) - GRAVITY*roll_mass*(r_z*_c_theta - _s_theta*(r_x*_c_phi - r_y*_s_phi));
  _compensationEffort.col(COMP_GRAVITY).block(0,0,2,1) =  GRAVITY*_massLinks(LINK_YAW)*_c_theta*(yw_z*_s_phi - _c_phi*(yw_x*_c_psi) - LINKS_COM[LINK_YAW][1]*_s_psi) + GRAVITY*_massLinks(LINK_PEDAL)*_c_theta*(LINKS_COM[LINK_PEDAL][2]*_s_phi + _s_phi*d6 + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi) + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi + GRAVITY*roll_mass*_c_theta*(r_y*_c_phi + r_x*_s_phi);
  _compensationEffort.col(COMP_GRAVITY).block(0,0,3,1) =  GRAVITY*_massLinks(LINK_YAW)*(_s_phi*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi) + yw_x*_s_psi + yw_z*_c_phi*_s_theta + _s_phi*_s_theta*(yw_x*_c_psi) - LINKS_COM[LINK_YAW][1]*_s_psi) - _c_phi*_s_theta*(yw_z*_s_phi - _c_phi*(yw_x*_c_psi) - LINKS_COM[LINK_YAW][1]*_s_psi)) + GRAVITY*_massLinks(LINK_PEDAL)*(_s_phi*(LINKS_COM[LINK_PEDAL][0]*_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi)*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta + _c_phi*_s_theta*d6) - _c_phi*_s_theta*(LINKS_COM[LINK_PEDAL][2]*_s_phi + _s_phi*d6 + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi) + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi);
}
#endif


void Platform::viscFrictionCompensation()
  {

    _compensationEffort.col(COMP_VISC_FRICTION) = _jointsViscosityGains.cwiseProduct(_speed);
  }

  void Platform::inertiaCompensation()
  {

    Eigen::Matrix<float, NB_AXIS, NB_AXIS> inertiaJointGainMatrix; //! M(q)
    inertiaJointGainMatrix.setZero();
    for (int link_ = LINK_BASE; link_ < NB_LINKS; link_++) {
      _linkCOMGeomJacobian[link_] = comGeometricJacobian( (link_chain) link_);
      _rotationMatrixCOM[link_] = comRotationMatrix((link_chain) link_);

      inertiaJointGainMatrix+= 
       (
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
   
   if (_flagSpeedSampledForCoriolis)
    { // Calculate the derivative of the Jacobian and Rotation Matrices
    
     // From bjerkeng2012
      Eigen::Matrix<float, NB_AXIS, NB_AXIS> coriolisJointGainMatrix; //! V(q,q)     
      coriolisJointGainMatrix.setConstant(0.0f);

    #if (CORIOLIS_DEV_STRATEGY==CORIOLIS_KRONECKER)
        Eigen::Matrix<float,NB_AXIS*NB_AXIS,NB_AXIS> kronProductSpeed;
        kronProductSpeed = kroneckerProductEye(_speed);
    #endif

     for (int link_ = LINK_BASE; link_ < NB_LINKS; link_++) 
     {

        #if (CORIOLIS_DEV_STRATEGY == CORIOLIS_TEMPORAL)
        //  _devLinkCOMGeomJacobian[link_] = filter_devLinkCOMGeometricJ[link_].update((_linkCOMGeomJacobian[link_] - _linkCOMGeometricJ_prev[link_])*invSpeedSampT ) ;
        //  _devRotationMatrixCOM[link_] = filter_devRotationMatrixCOM[link_].update((_rotationMatrixCOM[link_] - _rotationMatrixCOM_prev[link_])* invSpeedSampT)  ;
         _devLinkCOMGeomJacobian[link_] = (_linkCOMGeomJacobian[link_] - _linkCOMGeometricJ_prev[link_]) * invSpeedSampT  ;
         _devRotationMatrixCOM[link_] = (_rotationMatrixCOM[link_] - _rotationMatrixCOM_prev[link_])* invSpeedSampT  ;
         _rotationMatrixCOM_prev[link_] = _rotationMatrixCOM[link_];
         _linkCOMGeometricJ_prev[link_] = _linkCOMGeomJacobian[link_];

         #else
          Eigen::Matrix<float,6,NB_AXIS*NB_AXIS> temp;
          temp = devQComGeomJacobian((link_chain)link_);
          _devLinkCOMGeomJacobian[link_] =  temp* kronProductSpeed.block(0,0,NB_AXIS*NB_AXIS,NB_AXIS);
          Eigen::Matrix<float,NB_CART_AXIS,NB_CART_AXIS*NB_AXIS> temp2;
          temp2 = devQComRotationMatrix((link_chain) link_);
          _devRotationMatrixCOM[link_] =    temp2 * kronProductSpeed.block(0,0,NB_AXIS*3,3);
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
       _flagSpeedSampledForCoriolis = false;
    }
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
         } 
         else if (_speed(axis_) <= -SPEED_THRESHOLD[axis_]) {
           w_sign = NEG;
         } 
         else {
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
               dry_scale * _dryFrictionEffortSign[w_sign](axis_);
         } else {
           //! probabilistic deadzone
           float deadzone;
           //deadzone = map(smoothRise(((float)(rand() % 10) - 5.0f), -5.0f, 5.0f), 0.0f,1.0f, 0.7 * _dryFrictionEffortSign[NEG](axis_),0.7 * _dryFrictionEffortSign[POS](axis_));
           //_compensationEffort(axis_, COMP_DRY_FRICTION) =comp_filter[axis_].update(deadzone);
           _compensationEffort(axis_, COMP_DRY_FRICTION) = 0.0f;
           //! linear deazone
           //!_compensationEffort(axis_,COMP_DRY_FRICTION) = map(_speed(axis_),-SPEED_THRESHOLD[axis_],SPEED_THRESHOLD[axis_], 0.7f*_dryFrictionEffortSign[NEG][axis_], 0.7f*_dryFrictionEffortSign[POS][axis_]);
         }
       }
     }
    #endif
   }

void Platform::quadraticRegression() {
     for (int sign_ = 0; sign_ < NB_SIGN_COMP; sign_++) {
    
     for (int axis_ = 0; axis_ < NB_STICTION_COMP; axis_++) {
         _predictors[sign_].block(0, axis_, NB_AXIS, 1) = _position;
         _predictors[sign_].block(NB_AXIS, axis_, NB_AXIS, 1) = _position.cwiseProduct(_position);
       }
         _predictors[sign_] = (_predictors[sign_] - _mean[sign_]).cwiseProduct(_stdInv[sign_]);
         //! main formula!
         _dryFrictionEffortSign[sign_] = ( (_betas[sign_].cwiseProduct(_predictors[sign_])).colwise().sum() +
        _bias[sign_] ).transpose();
     }
}
