#include "Platform.h"

using namespace std;
using namespace Eigen;

//! Variables of the Quadratic Regression for Dry Friction

Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_AXIS> betas[NB_SIGN_COMP];
Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_AXIS> mean[NB_SIGN_COMP];
Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_AXIS> stdInv[NB_SIGN_COMP];
Eigen::Matrix<float, 1 , NB_STICTION_AXIS> bias[NB_SIGN_COMP];
bool flagQuadraticRegressionInit=false;


//! 1
void Platform::dynamicCompensation()
{
  _compensationEffort.setZero();
  if (_platform_compensation[COMP_GRAVITY])
    {gravityCompensation();}
  if (_platform_compensation[COMP_VISC_FRICTION])
    {viscFrictionCompensation();}
  if (_platform_compensation[COMP_INERTIA])
    {inertiaCompensation();}
  if (_platform_compensation[COMP_CORIOLIS])
    {coriolisCompensation(); //! coriolis has to be compensated after the inertia;
  }
  if (_platform_compensation[COMP_FORCE_SENSOR])
    {forceSensorCompensation(); }
  if (_platform_compensation[COMP_DRY_FRICTION])
    {dryFrictionCompensation();}

  _compensationEffort.block(0,0,NB_AXIS,NB_COMPENSATION_COMP-1) =
        boundMat(_compensationEffort.block(0,0,NB_AXIS,NB_COMPENSATION_COMP-1), _compTorqueLims[L_MIN], _compTorqueLims[L_MAX]);
  if (_platform_controlledAxis==-1)
  {
    _effortD_ADD.col(COMPENSATION) = _compensationEffort.rowwise().sum();
  }
  else
  {
    _effortD_ADD(_platform_controlledAxis, COMPENSATION) =
        _compensationEffort.row(_platform_controlledAxis).sum();
  }
  
}



#if (COMPENSATION_TYPE == COMP_MATRICIAL)
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

void Platform::inertiaCompensation() {

  Eigen::Matrix<float, NB_AXIS, NB_AXIS> inertiaJointGainMatrix; //! M(q)
  inertiaJointGainMatrix.setZero();
  for (int link_ = LINK_BASE; link_ < NB_LINKS; link_++) {
    _linkCOMGeomJacobian[link_] = comGeometricJacobian((link_chain)link_);
    _rotationMatrixCOM[link_] = comRotationMatrix((link_chain)link_);

    inertiaJointGainMatrix +=
        (_massLinks(link_) *
             _linkCOMGeomJacobian[link_]
                 .block(0, 0, NB_CART_AXIS, NB_AXIS)
                 .transpose() *
             _linkCOMGeomJacobian[link_].block(0, 0, NB_CART_AXIS, NB_AXIS) +
         _linkCOMGeomJacobian[link_]
                 .block(NB_CART_AXIS, 0, NB_CART_AXIS, NB_AXIS)
                 .transpose() *
             (_rotationMatrixCOM[link_] * _momentInertiaLinks[link_] *
              _rotationMatrixCOM[link_].transpose()) *
             _linkCOMGeomJacobian[link_].block(NB_CART_AXIS, 0, NB_CART_AXIS,
                                               NB_AXIS));
  }

  _compensationEffort.col(COMP_INERTIA) =
      inertiaJointGainMatrix * _acceleration;
}

void Platform::coriolisCompensation() {

  if (_flagSpeedSampledForCoriolis) { // Calculate the derivative of the
                                      // Jacobian and Rotation Matrices

    // From bjerkeng2012
    Eigen::Matrix<float, NB_AXIS, NB_AXIS> coriolisJointGainMatrix; //! V(q,q)
    coriolisJointGainMatrix.setConstant(0.0f);

#if (CORIOLIS_DEV_STRATEGY == CORIOLIS_KRONECKER)
    Eigen::Matrix<float, NB_AXIS * NB_AXIS, NB_AXIS> kronProductSpeed;
    kronProductSpeed = kroneckerProductEye(_speed);
#endif

    for (int link_ = LINK_BASE; link_ < NB_LINKS; link_++) {

#if (CORIOLIS_DEV_STRATEGY == CORIOLIS_TEMPORAL)
      //  _devLinkCOMGeomJacobian[link_] =
      //  filter_devLinkCOMGeometricJ[link_].update((_linkCOMGeomJacobian[link_]
      //  - _linkCOMGeometricJ_prev[link_])*invSpeedSampT ) ;
      //  _devRotationMatrixCOM[link_] =
      //  filter_devRotationMatrixCOM[link_].update((_rotationMatrixCOM[link_] -
      //  _rotationMatrixCOM_prev[link_])* invSpeedSampT)  ;
      _devLinkCOMGeomJacobian[link_] =
          (_linkCOMGeomJacobian[link_] - _linkCOMGeometricJ_prev[link_]) *
          invSpeedSampT;
      _devRotationMatrixCOM[link_] =
          (_rotationMatrixCOM[link_] - _rotationMatrixCOM_prev[link_]) *
          invSpeedSampT;
      _rotationMatrixCOM_prev[link_] = _rotationMatrixCOM[link_];
      _linkCOMGeometricJ_prev[link_] = _linkCOMGeomJacobian[link_];

#else
      Eigen::Matrix<float, 6, NB_AXIS * NB_AXIS> temp;
      temp = devQComGeomJacobian((link_chain)link_);
      _devLinkCOMGeomJacobian[link_] =
          temp * kronProductSpeed.block(0, 0, NB_AXIS * NB_AXIS, NB_AXIS);
      Eigen::Matrix<float, NB_CART_AXIS, NB_CART_AXIS * NB_AXIS> temp2;
      temp2 = devQComRotationMatrix((link_chain)link_);
      _devRotationMatrixCOM[link_] =
          temp2 * kronProductSpeed.block(0, 0, NB_AXIS * 3, 3);
#endif

      coriolisJointGainMatrix +=
          -(_massLinks(link_) *
                _linkCOMGeomJacobian[link_]
                    .block(0, 0, NB_CART_AXIS, NB_AXIS)
                    .transpose() *
                _devLinkCOMGeomJacobian[link_].block(0, 0, NB_CART_AXIS,
                                                     NB_AXIS) +
            _linkCOMGeomJacobian[link_]
                    .block(NB_CART_AXIS, 0, NB_CART_AXIS, NB_AXIS)
                    .transpose() *
                (_rotationMatrixCOM[link_] * _momentInertiaLinks[link_] *
                 _rotationMatrixCOM[link_].transpose()) *
                _devLinkCOMGeomJacobian[link_].block(NB_CART_AXIS, 0,
                                                     NB_CART_AXIS, NB_AXIS) +
            _linkCOMGeomJacobian[link_]
                    .block(NB_CART_AXIS, 0, NB_CART_AXIS, NB_AXIS)
                    .transpose() *
                (_devRotationMatrixCOM[link_] * _momentInertiaLinks[link_] *
                 _rotationMatrixCOM[link_].transpose()) *
                _linkCOMGeomJacobian[link_].block(NB_CART_AXIS, 0, NB_CART_AXIS,
                                                  NB_AXIS));
    }

    _compensationEffort.col(COMP_CORIOLIS) = coriolisJointGainMatrix * _speed;
    _flagSpeedSampledForCoriolis = false;
  }
}

#else


void Platform::inertiaCompensation() {

  _compensationEffort(Y, COMP_INERTIA) = COMP_INERTIA_EQ_Y;
  _compensationEffort(X, COMP_INERTIA) = COMP_INERTIA_EQ_X;
  _compensationEffort(PITCH, COMP_INERTIA) = COMP_INERTIA_EQ_PITCH;
  _compensationEffort(ROLL, COMP_INERTIA) = COMP_INERTIA_EQ_ROLL;
  _compensationEffort(YAW, COMP_INERTIA) = COMP_INERTIA_EQ_YAW;
  _compensationEffort.col(COMP_INERTIA) = _compensationEffort.col(COMP_INERTIA).cwiseProduct(_platform_filterAxisFS);
}

void Platform::forceSensorCompensation()
{
  _forceSensorD = (_effortD_ADD.col(NORMAL) + _effortD_ADD.col(CONSTRAINS) + _effortD_ADD.col(RCM_MOTION));
  for (int k=0; k<NB_AXIS; k++)
  {
    if (abs(_platform_filterAxisFS(k)) != 0.0f)
    {
      _pidForceSensor[k]->compute();
    }
    else
    {
     _forceSensorCtrlOut(k) = 0.0f ;
    }
  }
  _compensationEffort.col(COMP_FORCE_SENSOR)= _forceSensorCtrlOut;
}

void Platform::coriolisCompensation() {
  _compensationEffort(Y, COMP_CORIOLIS) = COMP_CORIOLIS_EQ_Y;
  _compensationEffort(X, COMP_CORIOLIS) = COMP_CORIOLIS_EQ_X;
  _compensationEffort(PITCH, COMP_CORIOLIS) = COMP_CORIOLIS_EQ_PITCH;
  _compensationEffort(ROLL, COMP_CORIOLIS) = COMP_CORIOLIS_EQ_ROLL;
  _compensationEffort(YAW, COMP_CORIOLIS) = COMP_CORIOLIS_EQ_YAW;
  _compensationEffort.col(COMP_CORIOLIS) = _compensationEffort.col(COMP_CORIOLIS).cwiseProduct(_platform_filterAxisFS);
}

void Platform::gravityCompensation() {
   _compensationEffort(Y,COMP_GRAVITY) = 0.0f;
   _compensationEffort(X, COMP_GRAVITY) = 0.0f;
   _compensationEffort(PITCH,COMP_GRAVITY) = GRAVITY*_massLinks(LINK_PEDAL)*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta + _c_phi*_s_theta*d6) + GRAVITY*_massLinks(LINK_YAW)*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) + GRAVITY*_massLinks(LINK_PITCH)*(LINKS_COM[LINK_PITCH][1]*_c_theta + LINKS_COM[LINK_PITCH][0]*_s_theta) - GRAVITY*_massLinks(LINK_ROLL)*(LINKS_COM[LINK_ROLL][2]*_c_theta - _s_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi));
   _compensationEffort(ROLL,COMP_GRAVITY)  = GRAVITY*_massLinks(LINK_YAW)*_c_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) + GRAVITY*_massLinks(LINK_PEDAL)*_c_theta*(LINKS_COM[LINK_PEDAL][2]*_s_phi + _s_phi*d6 + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi) + GRAVITY*_massLinks(LINK_ROLL)*_c_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi);
   _compensationEffort(YAW,COMP_GRAVITY) = GRAVITY*_massLinks(LINK_YAW)*(_s_phi*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi))) + GRAVITY*_massLinks(LINK_PEDAL)*(_s_phi*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta + _c_phi*_s_theta*d6) - _c_phi*_s_theta*(LINKS_COM[LINK_PEDAL][2]*_s_phi + _s_phi*d6 + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi));
}
#endif


void Platform::viscFrictionCompensation()
  {

    _compensationEffort.col(COMP_VISC_FRICTION) = _jointsViscosityGains.cwiseProduct(_speed).cwiseProduct(_platform_filterAxisFS);
  }


   /**************************************FRICTION COMPENSATION*************************/

   void Platform::dryFrictionCompensation() {



    #if (MODEL_TYPE == LINEAR_MODEL)

      quadraticRegression();

     for (int axis_ = 0; axis_ < NB_AXIS; axis_++) {
       if (axis_ < NB_STICTION_AXIS) {
         float dry_scale = 1.0f;
         uint w_sign = MID_FRIC;

         if (_speed(axis_) > SPEED_THRESHOLD[axis_]) {
           w_sign = POS_FRIC;
         } 
         else if (_speed(axis_) < -SPEED_THRESHOLD[axis_]) {
           w_sign = NEG_FRIC;
         } 
         else {
           w_sign = MID_FRIC;
         }

         dry_scale =
             clip(abs(SPEED_THRESHOLD[axis_] / (_speed(axis_) + 1e-10f)), 0.95f,
                  1.0f);

         _dryFrictionEffortSign[NEG_FRIC](axis_) =
             clip(_dryFrictionEffortSign[NEG_FRIC](axis_),
                  DRY_EFFORT_LIMS[NEG_FRIC][L_MIN][axis_],
                  DRY_EFFORT_LIMS[NEG_FRIC][L_MAX][axis_]);
         _dryFrictionEffortSign[POS_FRIC](axis_) =
             clip(_dryFrictionEffortSign[POS_FRIC](axis_),
                  DRY_EFFORT_LIMS[POS_FRIC][L_MIN][axis_],
                  DRY_EFFORT_LIMS[POS_FRIC][L_MAX][axis_]);

         if (w_sign == POS_FRIC || w_sign == NEG_FRIC) {
           _compensationEffort(axis_, COMP_DRY_FRICTION) =
               dry_scale * _dryFrictionEffortSign[w_sign](axis_);
         } else {
           //! probabilistic deadzone
               _compensationEffort(axis_, COMP_DRY_FRICTION) = 0.0f;
         }
       }
     }
    #endif
   }

void Platform::quadraticRegression() {
     
    if (!flagQuadraticRegressionInit)
     {
       for (int sign_ = 0; sign_ < NB_SIGN_COMP; sign_++) {
        betas[sign_].col(Y) = Eigen::Map<const Eigen::MatrixXf>(BETAS_QR_Y[sign_], 2 * NB_AXIS, 1);
        mean[sign_].col(Y) = Eigen::Map<const Eigen::MatrixXf>(MEAN_QR_Y[sign_], 2 * NB_AXIS, 1);
        stdInv[sign_].col(Y) = Eigen::Map<const Eigen::MatrixXf>(STDINV_QR_Y[sign_], 2 * NB_AXIS, 1);
        betas[sign_].col(X) = Eigen::Map<const Eigen::MatrixXf>(BETAS_QR_X[sign_], 2 * NB_AXIS, 1);
        mean[sign_].col(X) = Eigen::Map<const Eigen::MatrixXf>(MEAN_QR_X[sign_], 2 * NB_AXIS, 1);
        stdInv[sign_].col(X) = Eigen::Map<const Eigen::MatrixXf>(STDINV_QR_X[sign_], 2 * NB_AXIS, 1);
        bias[sign_] << BIAS_QR_Y[sign_] , BIAS_QR_X[sign_];
       }

       flagQuadraticRegressionInit=true;
     }

     for (int sign_ = 0; sign_ < NB_SIGN_COMP; sign_++) {
        
        for (int axis_ = 0; axis_ < NB_STICTION_AXIS; axis_++) {
            _predictors[sign_].block(0, axis_, NB_AXIS, 1) = _position;
            _predictors[sign_].block(NB_AXIS, axis_, NB_AXIS, 1) = _position.cwiseProduct(_position);
          }
         _predictors[sign_] = (_predictors[sign_] - mean[sign_]).cwiseProduct(stdInv[sign_]);
         //! main formula!
         _dryFrictionEffortSign[sign_] = ( (betas[sign_].cwiseProduct(_predictors[sign_])).colwise().sum() +
        bias[sign_] ).transpose();
     }
}
