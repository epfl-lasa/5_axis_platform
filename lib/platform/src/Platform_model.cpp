#include "Platform.h"
#include "LP_Filter.h"
#include "definitions.h"
#include "definitions_2.h"

using namespace std;
using namespace Eigen;


//! Geometric Parameters
extern const float r3 = 0.305f;
#if (PRESENCE_FORCE_SENSOR)
extern const float d6 = 0.1326f;
//! Links Inertial Parameters
extern const float LINKS_COM[NB_LINKS][3] = {{0.06506077f, 0.23761375f, 0.06685000f},
                                            {0.02817123f, -0.07585496f, -0.00969366f},
                                            {-0.15754146f, 0.02306281f, 0.06946538f},
                                            {-0.00593072f, -0.00000124f, -0.00156324f},
                                            {0.03719409f, -0.00002562f, -0.00002217f},
                                            {-0.00151943f, -0.00177412f, 0.10396420f,},
                                            {-0.03259637f, 0.00014150f, 0.02254560f}};

extern const float LINKS_MASS[NB_LINKS] = {6.20696344f, 1.86377694f, 3.45202011f, 6.40922346f, 1.17868723f, 0.42803941f, 0.37200000f};

extern const float LINKS_MOMENT_OF_INERTIAS[NB_LINKS][9] = {
    {0.16517950f , 0.01377293f , -0.00179536f, 
     0.01377293f , 0.15434303f ,  0.01782405f, 
    -0.00179536f , 0.01782405f ,  0.29648455f}  // LINK X  W.R.T COM_BASE
    ,
    {0.00343968f, 0.00007319f,0.00001718f,
	 0.00007319f, 0.06820557f,0.00085107f,
	 0.00001718f, 0.00085107f,0.06938224f} //LINK Y W.R.T COM_Y
    ,
    { 0.02316770f, -0.00205135f, -0.00581720f,
	 -0.00205135f,  0.02061438f,  0.00627358f,
	 -0.00581720f,  0.00627358f,  0.02484097f} // LINK X W.R.T COM_X 
    ,
    {0.10886020f, 0.00001273f, 0.00088705f,
     0.00001273f, 0.02644452f, 0.00000809f,
	 0.00088705f, 0.00000809f, 0.09345014f} // LINK_PITCH W.R.T COM_PITCH 
    ,
    { 0.00153790, -0.00000106,	0.00003310,
	 -0.00000106,  0.00213092,	0.00000026,
	  0.00003310,  0.00000026,	0.00126543} // LINK_ROLL W.R.T COM_ROLL 
    ,
    { 0.00031947f, -0.00002243f, -0.00000878f,
	 -0.00002243f,  0.00032862f,  0.00000745f,
	 -0.00000878f,  0.00000745f,  0.00027962f} // LINK_YAW W.R.T COM_YAW
    ,
    { 0.00045218f, -0.00001660f,  0.00011638f,
     -0.00001660f,  0.00284161f, -0.00000098f,
      0.00011638f, -0.00000098f,  0.00296033f} // LINK_PEDAL W.R.T IN COM_FS
};

#else
extern const float d6 = 0.0786f; 
//! Links Inertial Parameters

extern const float LINKS_COM[NB_LINKS][NB_CART_AXIS] = {{0.06506077f, 0.23761375f, 0.06685000f},
                                          {0.02817123f, -0.07585496f, -0.00969366f},
                                          {-0.15754146f, 0.02306281f, 0.06946538f},
                                          {-0.00593072f, -0.00000124f, -0.00156324f}, 
                                          { 0.03719409f, -0.00002562f, -0.00002217f},
                                          {0.00000000f, 0.00000000f, 0.07495233f},
                                          {-0.03295545f, 0.00021567f, 0.02639184f}};

extern const float LINKS_MASS[NB_LINKS] = {6.20696344f, 1.86377694f, 3.45202011f, 6.40922346f, 1.17868723f, 0.09770847f,0.37200000f};

extern const float mod_inertia_pitch = 0.09345014f - 0.09345014f  - 0.00213092f - 0.00004184f - (0.00335913f + LINKS_MASS[6]*d6*d6) + 0.2521f;

extern const float mod_inertia_roll= 0.00126543f - 0.00126543f - 0.00003916f -(0.00049690f + LINKS_MASS[6]*d6*d6) + 0.1831f; 

extern const float mod_inertia_yaw = 0.00000699f - 0.00000699f -  0.00351089f + 0.1867f;

extern const float LINKS_MOMENT_OF_INERTIAS[NB_LINKS][NB_CART_AXIS*NB_CART_AXIS] = {
    {0.16517950f , 0.01377293f , -0.00179536f, 
     0.01377293f , 0.15434303f ,  0.01782405f, 
    -0.00179536f , 0.01782405f ,  0.29648455f}  // LINK BASE  W.R.T COM_BASE
    ,
    {0.00343968f, 0.00007319f,0.00001718f,
	 0.00007319f, 0.06820557f,0.00085107f,
	 0.00001718f, 0.00085107f,0.06938224f} //LINK Y W.R.T COM_Y
    ,
    { 0.02316770f, -0.00205135f, -0.00581720f,
	 -0.00205135f,  0.02061438f,  0.00627358f,
	 -0.00581720f,  0.00627358f,  0.02484097f} // LINK X W.R.T COM_X 
    ,
    {0.10886020f, 0.00001273f, 0.00088705f,
     0.00001273f, 0.02644452f, 0.00000809f,
	 0.00088705f, 0.00000809f, mod_inertia_pitch} // LINK_PITCH W.R.T COM_PITCH 
    ,
    { 0.00153790f, -0.00000106f,	0.00003310f,
	 -0.00000106f,  0.00213092f,	0.00000026f,
	  0.00003310f,  0.00000026f,	mod_inertia_roll} // LINK_ROLL W.R.T COM_ROLL 
    ,
    {0.00004184f, -0.00000023f, 0.00000000f,
	-0.00000023f,  0.00003916f, 0.00000000f,
	 0.00000000f,  0.00000000f, mod_inertia_yaw} // LINK_YAW W.R.T COM_YAW
    ,
    { 0.00049690f, -0.00001989f,  0.00016983f,
     -0.00001989f,  0.00335913f, -0.00000136f,
      0.00016983f, -0.00000136f,  0.00351089f} // LINK_PEDAL W.R.T IN COM_FS
};

#endif
extern const float d7 = 0.023f; 
extern const float r8 = 0.209f;


//extern float const VISCOUS_K[NB_AXIS] = {2.0f, 2.0f, 46.0734f,62.3174f, 0.0f};
extern float const VISCOUS_K[NB_AXIS] = {2.0f, 2.0f, 46.0734f*DEG_TO_RAD, 62.3174f*DEG_TO_RAD,10.0f*DEG_TO_RAD};

// float const EMPIRICAL_INERTIA_K[NB_AXIS] = {13.6178f,13.7704f, 0.2521f, 
//                                   0.1831f, 0.1867f};


Eigen::Vector3f Platform::positionFrame(frame_chain frame)
{
    
  Eigen::Vector3f positionFrame_;
  positionFrame_.setConstant(0.0f);



    switch (frame)
    {
        case FRAME_BASE:
        {
            positionFrame_.setConstant(0.0f);
            break;   
        }

        case FRAME_Y:
        {
            positionFrame_.setConstant(0.0f);
            break;   
        }

        case FRAME_X:
        {
            positionFrame_<<0.0f,_position(Y),0.0f; 
            break;  
        }

        case FRAME_Z:
        {
            positionFrame_<<_position(X),_position(Y),0.0f; 
            break;  
        }

        case FRAME_PITCH ... FRAME_YAW:
        {
            positionFrame_<<_position(X) , _position(Y) , r3;
            break;   
        }

        case FRAME_FS:
        {
            positionFrame_(CART_X) = _position(X) + d6 * _s_phi;
            positionFrame_(CART_Y) = _position(Y) - d6 * _c_phi * _s_theta;
            positionFrame_(CART_Z) =  r3 + d6 * _c_phi * _c_theta;
            break;   
        }
        
        case FRAME_PEDAL:
        {
            positionFrame_(CART_X) = _position(X) + (d6 + d7) * _s_phi;
            positionFrame_(CART_Y) = _position(Y) - (d6 + d7) * _c_phi * _s_theta;
            positionFrame_(CART_Z) = r3 + (d6+d7) * _c_phi * _c_theta; 
            break;  
        }

        case FRAME_EPOINT:
        {
            positionFrame_(CART_X) = _position(X) + (d6 + d7) * _s_phi - r8 * _c_phi * _s_psi;
            positionFrame_(CART_Y) = _position(Y) - (d6 + d7) * _c_phi * _s_theta + 
                              r8 * ( _c_psi*_c_theta - _s_phi * _s_psi * _s_theta) ;
            positionFrame_(CART_Z) = r3 + (d6+d7) * _c_phi * _c_theta + 
                              r8 * ( _c_psi*_s_theta + _c_theta*_s_phi*_s_psi ) ; 
            break;   
        }
    }
    return positionFrame_;
}

Eigen::Matrix3f Platform::rotationMatrix(frame_chain frame) //! orientation of the joint frame wrt to the world frame
{
    
  Eigen::Matrix3f rotationMatrix_;
  rotationMatrix_.setIdentity();



    switch (frame)
    {
        case FRAME_BASE:
        {
            rotationMatrix_.setIdentity();
            break;   
        }

        case FRAME_Y:
        {
            rotationMatrix_ << 1.0f , 0.0f , 0.0f,
                             0.0f , 0.0f , 1.0f,
                             0.0f ,-1.0f , 0.0f; 
            break;   
        }

        case FRAME_X:   
        {
            rotationMatrix_ << 0.0f , 0.0f , 1.0f,
                             0.0f , 1.0f , 0.0f,
                            -1.0f , 0.0f , 0.0f;  
            break;  
        }

        case FRAME_Z ... FRAME_PITCH:
        {
            rotationMatrix_ << 0.0f , 0.0f , 1.0f,
                             0.0f ,-1.0f , 0.0f,
                             1.0f , 0.0f , 0.0f;  
            break;  
        }

        case FRAME_ROLL:
        {
            rotationMatrix_ << 0.0f    , 1.0f ,    0.0f,
                            -_s_theta , 0.0f , _c_theta,
                             _c_theta , 0.0f , _s_theta; 
            break;   
        }


        case FRAME_YAW:
        {

            
            rotationMatrix_ << -_c_phi         ,  0.0f    ,          _s_phi,
                             -_s_phi*_s_theta , -_c_theta , -_c_phi*_s_theta,
                              _c_theta*_s_phi , -_s_theta ,  _c_phi*_c_theta; 
            break;   
        }

        case FRAME_FS:
        {
            rotationMatrix_(0,0) = _c_phi*_s_psi;
            rotationMatrix_(1,0) = _s_phi*_s_psi*_s_theta -  _c_psi*_c_theta;
            rotationMatrix_(2, 0) = - (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi);
            rotationMatrix_(0, 1) = _c_phi*_c_psi;
            rotationMatrix_(1, 1) = _c_theta*_s_psi + _c_psi*_s_phi*_s_theta;
            rotationMatrix_(2, 1) = _s_psi*_s_theta - _c_psi*_c_theta*_s_phi;
            rotationMatrix_(0, 2) = _s_phi;
            rotationMatrix_(1, 2) = -_c_phi*_s_theta;
            rotationMatrix_(2, 2) = _c_phi*_c_theta;

        //    rotationMatrix_ <<  _c_phi*_s_psi                         ,  _c_phi*_c_psi                         ,  _s_phi,
        //                     _s_phi*_s_psi*_s_theta -  _c_psi*_c_theta , _c_theta*_s_psi + _c_psi*_s_phi*_s_theta , -_c_phi*_s_theta,
        //                     -(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) , _s_psi*_s_theta - _c_psi*_c_theta*_s_phi  ,  _c_phi*_c_theta; 
            break;   
        }
        
        case FRAME_PEDAL ... FRAME_EPOINT:
        {
            rotationMatrix_(0,0) = -_c_phi*_s_psi;
            rotationMatrix_(1,0) = -(_s_phi*_s_psi*_s_theta -  _c_psi*_c_theta);
            rotationMatrix_(2,0) = _c_psi * _s_theta + _c_theta * _s_phi * _s_psi;
            rotationMatrix_(0,1) = -_c_phi*_c_psi;
            rotationMatrix_(1,1) = -( _c_theta*_s_psi + _c_psi*_s_phi*_s_theta);
            rotationMatrix_(2,1) = -( _s_psi*_s_theta - _c_psi*_c_theta*_s_phi);
            rotationMatrix_(0,2) = _s_phi;
            rotationMatrix_(1,2) = -_c_phi*_s_theta;
            rotationMatrix_(2,2) = _c_phi*_c_theta;
            //    rotationMatrix_ <<  -_c_phi*_s_psi                            , -_c_phi*_c_psi                            ,  _s_phi,
            //  -(_s_phi*_s_psi*_s_theta -  _c_psi*_c_theta) , -( _c_theta*_s_psi + _c_psi*_s_phi*_s_theta) , -_c_phi*_s_theta,
            //    _c_psi*_s_theta + _c_theta*_s_phi*_s_psi    , -( _s_psi*_s_theta - _c_psi*_c_theta*_s_phi) ,  _c_phi*_c_theta;
                break;
        }

    }

    return rotationMatrix_;
}


Eigen::Matrix<float,6,NB_AXIS> Platform::geometricJacobian(frame_chain frame)
{

    Eigen::Matrix<float, 6, NB_AXIS> J;
    J.setConstant(0.0f);

    switch (frame)
    {
        case FRAME_BASE ... FRAME_Y:
        {
            J.setConstant(0.0f);
            break;   
        }

        case FRAME_X:
        {
            J(1,0) = 1.0f;
            break;   
        }

        case FRAME_Z ... FRAME_PITCH:
        {
            J(1,0) = 1.0f;
            J(0,1) = 1.0f;
            break;  
        }

        case FRAME_ROLL:
        {
            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(3, 2) = 1.0f;
            // J<< 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            //     1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            //     0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
            break;   
        }


        case FRAME_YAW:
        {
            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(3, 2) = 1.0f;
            J(4, 3) = _c_theta;
            J(5, 3) = _s_theta;
            // J<< 0.0f, 1.0f, 0.0f,    0.0f, 0.0f,
            //     1.0f, 0.0f, 0.0f,    0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f,    0.0f, 0.0f,
            //     0.0f, 0.0f, 1.0f,    0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f, _c_theta, 0.0f,
            //     0.0f, 0.0f, 0.0f, _s_theta, 0.0f;


            
             break;   
        }

        case FRAME_FS:
        {
            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(1, 2) = -d6 * _c_phi * _c_theta;
            J(2, 2) = -d6 * _c_phi * _s_theta;
            J(3, 2) = 1.0f;
            J(0, 3) = d6 * _c_phi;
            J(1, 3) = d6 * _s_phi * _s_theta;
            J(2, 3) = -d6 * _c_theta * _s_phi;
            J(4, 3) = _c_theta;
            J(5, 3) = _s_theta;
            J(3, 4) = _s_phi;
            J(4, 4) = -_c_phi * _s_theta;
            J(5, 4) = _c_phi * _c_theta;

            // J<< 0.0f, 1.0f,              0.0f, d6*_c_phi*_c_theta*_c_theta + d6*_c_phi*_s_theta*_s_theta,           0.0f,
            //     1.0f, 0.0f, -d6*_c_phi*_c_theta,                                    d6*_s_phi*_s_theta,           0.0f,
            //     0.0f, 0.0f, -d6*_c_phi*_s_theta,                                   -d6*_c_theta*_s_phi,           0.0f,
            //     0.0f, 0.0f,              1.0f,                                                0.0f,          _s_phi,
            //     0.0f, 0.0f,              0.0f,                                             _c_theta, -_c_phi*_s_theta,
            //     0.0f, 0.0f,              0.0f,                                             _s_theta,  _c_phi*_c_theta;
            break;   
        }
        
        case FRAME_PEDAL:
        {

            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(1, 2) = -(d6+d7) * _c_phi * _c_theta;
            J(2, 2) = -(d6+d7) * _c_phi * _s_theta;
            J(3, 2) = 1.0f;
            J(0, 3) = (d6+d7) * _c_phi ;
            J(1, 3) = (d6+d7) * _s_phi * _s_theta;
            J(2, 3) = -(d6+d7) * _c_theta * _s_phi;
            J(4, 3) = _c_theta;
            J(5, 3) = _s_theta;
            J(0, 4) = 0.0f;
            J(1, 4) = 0.0f;
            J(2, 4) = 0.0f;
            J(3, 4) = _s_phi;
            J(4, 4) = -_c_phi * _s_theta;
            J(5, 4) = _c_phi * _c_theta;
            // J<< 0.0f, 1.0f,                                  0.0f, _c_theta*(d6*_c_phi*_c_theta + d7*_c_phi*_c_theta) + _s_theta*(d6*_c_phi*_s_theta + d7*_c_phi*_s_theta), _c_phi*_c_theta*(d6*_c_phi*_s_theta + d7*_c_phi*_s_theta) - _c_phi*_s_theta*(d6*_c_phi*_c_theta + d7*_c_phi*_c_theta),
            //     1.0f, 0.0f, - d6*_c_phi*_c_theta - d7*_c_phi*_c_theta,                                                                 _s_theta*(d6*_s_phi + d7*_s_phi),                         _c_phi*_c_theta*(d6*_s_phi + d7*_s_phi) - _s_phi*(d6*_c_phi*_c_theta + d7*_c_phi*_c_theta),
            //     0.0f, 0.0f, - d6*_c_phi*_s_theta - d7*_c_phi*_s_theta,                                                                -_c_theta*(d6*_s_phi + d7*_s_phi),                         _c_phi*_s_theta*(d6*_s_phi + d7*_s_phi) - _s_phi*(d6*_c_phi*_s_theta + d7*_c_phi*_s_theta),
            //     0.0f, 0.0f,                                  1.0f,                                                                                          0.0f,                                                                                                     _s_phi,
            //     0.0f, 0.0f,                                  0.0f,                                                                                       _c_theta,                                                                                            -_c_phi*_s_theta,
            //     0.0f, 0.0f,                                  0.0f,                                                                                       _s_theta,                                                                                             _c_phi*_c_theta;
            break; 
        }
         case FRAME_EPOINT:
        {

            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(1, 2) = -(d6 + d7) * _c_phi * _c_theta - r8 * (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi);
            J(2, 2) = -(d6 + d7) * _c_phi * _s_theta - r8 * (_c_psi * _c_theta + _s_theta * _s_phi * _s_psi);
            J(3, 2) = 1.0f;
            J(0, 3) = _c_theta * (d6 * _c_phi * _c_theta + d7 * _c_phi * _c_theta + r8 * _c_psi * _s_theta +
                      r8 * _c_theta * _s_phi * _s_psi) + _s_theta * (d6 * _c_phi * _s_theta - 
                      r8 * _c_psi * _c_theta + d7 * _c_phi * _s_theta + r8 * _s_phi * _s_psi * _s_theta);
            J(1, 3) = _s_theta * (d6 * _s_phi + d7 * _s_phi - r8 * _c_phi * _s_psi);
            J(2, 3) = -_c_theta * (d6 * _s_phi + d7 * _s_phi - r8 * _c_phi * _s_psi);
            J(4, 3) = _c_theta;
            J(5, 3) = _s_theta;
            J(0, 4) = _c_phi * _c_theta * (d6 * _c_phi * _s_theta - 
                      r8 * _c_psi * _c_theta + d7 * _c_phi * _s_theta + 
                      r8 * _s_phi * _s_psi * _s_theta) - _c_phi * _s_theta * (d6 * _c_phi * _c_theta + 
                      d7 * _c_phi * _c_theta + r8 * _c_psi * _s_theta + 
                      r8 * _c_theta * _s_phi * _s_psi);
            J(1, 4) = _c_phi*_c_theta*(d6*_s_phi + d7*_s_phi - 
                      r8*_c_phi*_s_psi) - _s_phi*(d6*_c_phi*_c_theta + 
                      d7*_c_phi*_c_theta + r8*_c_psi*_s_theta + 
                      r8*_c_theta*_s_phi*_s_psi);
            J(2, 4) = _c_phi * _s_theta * (d6 * _s_phi + d7 * _s_phi - r8 * _c_phi * _s_psi) - 
                    _s_phi * (d6 * _c_phi * _s_theta - 
                    r8 * _c_psi * _c_theta + d7 * _c_phi * _s_theta + 
                    r8 * _s_phi * _s_psi * _s_theta);
            J(3, 4) = _s_phi;
            J(4, 4) = -_c_phi * _s_theta;
            J(5, 4) = _c_phi * _c_theta;

            // J<< 0.0f, 1.0f,                                                                              0.0f, _c_theta*(d6*_c_phi*_c_theta + d7*_c_phi*_c_theta + r8*_c_psi*_s_theta + r8*_c_theta*_s_phi*_s_psi) + _s_theta*(d6*_c_phi*_s_theta - r8*_c_psi*_c_theta + d7*_c_phi*_s_theta + r8*_s_phi*_s_psi*_s_theta) , _c_phi*_c_theta*(d6*_c_phi*_s_theta - r8*_c_psi*_c_theta + d7*_c_phi*_s_theta + r8*_s_phi*_s_psi*_s_theta) - _c_phi*_s_theta*(d6*_c_phi*_c_theta + d7*_c_phi*_c_theta + r8*_c_psi*_s_theta + r8*_c_theta*_s_phi*_s_psi),
            //     1.0f, 0.0f, - d6*_c_phi*_c_theta - d7*_c_phi*_c_theta - r8*_c_psi*_s_theta - r8*_c_theta*_s_phi*_s_psi,                                                                                                                                        _s_theta*(d6*_s_phi + d7*_s_phi - r8*_c_phi*_s_psi) ,                                                    _c_phi*_c_theta*(d6*_s_phi + d7*_s_phi - r8*_c_phi*_s_psi) - _s_phi*(d6*_c_phi*_c_theta + d7*_c_phi*_c_theta + r8*_c_psi*_s_theta + r8*_c_theta*_s_phi*_s_psi),
            //     0.0f, 0.0f,   r8*_c_psi*_c_theta - d6*_c_phi*_s_theta - d7*_c_phi*_s_theta - r8*_s_phi*_s_psi*_s_theta,                                                                                                                                       -_c_theta*(d6*_s_phi + d7*_s_phi - r8*_c_phi*_s_psi) ,                                                    _c_phi*_s_theta*(d6*_s_phi + d7*_s_phi - r8*_c_phi*_s_psi) - _s_phi*(d6*_c_phi*_s_theta - r8*_c_psi*_c_theta + d7*_c_phi*_s_theta + r8*_s_phi*_s_psi*_s_theta),
            //     0.0f, 0.0f,                                                                              1.0f,                                                                                                                                                                                  0.0f ,                                                                                                                                                                                             _s_phi,
            //     0.0f, 0.0f,                                                                              0.0f,                                                                                                                                                                               _c_theta ,                                                                                                                                                                                    -_c_phi*_s_theta,
            //     0.0f, 0.0f,                                                                              0.0f,                                                                                                                                                                               _s_theta ,                                                                                                                                                                                     _c_phi*_c_theta;
 

                    break; 
        }   
    }
  return J;
}

Eigen::Vector3f Platform::comLinkWRTBase(link_chain link) {

  Eigen::Vector3f positionCOMLink_;
  positionCOMLink_.setConstant(0.0f);

  float _c_theta = cos(_position(PITCH));
  float _c_phi = cos(_position(ROLL));
  float _c_psi = cos(_position(YAW));
  float _s_theta = sin(_position(PITCH));
  float _s_phi = sin(_position(ROLL));
  float _s_psi = sin(_position(YAW));

  switch (link) {
    case LINK_BASE: {

        positionCOMLink_(CART_X) = LINKS_COM[LINK_BASE][0];
        positionCOMLink_(CART_Y) = LINKS_COM[LINK_BASE][1];
        positionCOMLink_(CART_Z) = LINKS_COM[LINK_BASE][2];
        break;
    }

    case LINK_Y:

    {
        positionCOMLink_(CART_X) = LINKS_COM[LINK_Y][0];
        positionCOMLink_(CART_Y) = _position(Y) + LINKS_COM[LINK_Y][2];
        positionCOMLink_(CART_Z) = -LINKS_COM[LINK_Y][0];
        break;
    }

    case LINK_X:

    {
        positionCOMLink_(CART_X) = _position(X) + LINKS_COM[LINK_X][2];
        positionCOMLink_(CART_Y) = _position(Y) + LINKS_COM[LINK_X][1];
        positionCOMLink_(CART_Z) = -LINKS_COM[LINK_X][0];
        break;
    }

    case LINK_PITCH:

    {
        positionCOMLink_(CART_X) = LINKS_COM[LINK_PITCH][2] + _position(X);
        positionCOMLink_(CART_Y) = _position(Y) - LINKS_COM[LINK_PITCH][1] * _c_theta -
                            LINKS_COM[LINK_PITCH][0] * _s_theta;
        positionCOMLink_(CART_Z) = LINKS_COM[LINK_PITCH][0] * _c_theta -
                            LINKS_COM[LINK_PITCH][1] * _s_theta + r3;
        break;
    }

    case LINK_ROLL:

    {
        positionCOMLink_(CART_X) = _position(X) + LINKS_COM[LINK_ROLL][1] * _c_phi +
                            LINKS_COM[LINK_ROLL][0] * _s_phi;
        positionCOMLink_(CART_Y) = _position(Y) -
                            _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -
                                        LINKS_COM[LINK_ROLL][1] * _s_phi) +
                            LINKS_COM[LINK_ROLL][2] * _c_theta;
        positionCOMLink_(CART_Z) = _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -
                                        LINKS_COM[LINK_ROLL][1] * _s_phi) +
                            LINKS_COM[LINK_ROLL][2] * _s_theta + r3;
        break;
    }

    case LINK_YAW:

    {
        positionCOMLink_(CART_X) = _position(X) -
                            _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                    LINKS_COM[LINK_YAW][1] * _s_psi) +
                            LINKS_COM[LINK_YAW][2] * _s_phi;
        positionCOMLink_(CART_Y) =_position(Y) -
                            _c_theta *
                                (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) -
                            _s_phi * _s_theta *
                                (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -
                            LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta;
        positionCOMLink_(CART_Z) =
                            _c_theta * _s_phi *
                                (LINKS_COM[LINK_YAW][0] * _c_psi - LINKS_COM[LINK_YAW][1] * _s_psi) -
                            _s_theta *
                                (LINKS_COM[LINK_YAW][1] * _c_psi + LINKS_COM[LINK_YAW][0] * _s_psi) +
                            LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta + r3;
        break;
    }

    case LINK_PEDAL:

    {
        positionCOMLink_(CART_X) = _position(X) + d6 * _s_phi +
                            LINKS_COM[LINK_PEDAL][2] * _s_phi +
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi;
        positionCOMLink_(CART_Y) =
            _position(Y) - d6 * _c_phi * _s_theta +
            LINKS_COM[LINK_PEDAL][1] * (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) -
            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -
            LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta;
        positionCOMLink_(CART_Z) =
            d6 * _c_phi * _c_theta -
            LINKS_COM[LINK_PEDAL][0] * (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +
            LINKS_COM[LINK_PEDAL][1] * (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) +
            LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta + r3;

        break;
    }
  }
  return positionCOMLink_;
}

Eigen::Matrix3f Platform::comRotationMatrix(link_chain link){

  Eigen::Matrix3f comRotationMatrix_;
  comRotationMatrix_.setIdentity();



    switch (link)
    {
        case LINK_BASE:
        {
            comRotationMatrix_.setIdentity();
            break;   
        }

        case LINK_Y:
        {
            comRotationMatrix_ << 1.0f , 0.0f , 0.0f,
                                  0.0f , 0.0f , 1.0f,
                                  0.0f ,-1.0f , 0.0f; 
            break;   
        }

        case LINK_X:   
        {
            comRotationMatrix_ << 0.0f , 0.0f , 1.0f,
                                  0.0f , 1.0f , 0.0f,
                                 -1.0f , 0.0f , 0.0f;  
            break;  
        }

        case LINK_PITCH:
        {
            comRotationMatrix_ <<      0.0f,        0.0f, 1.0f,
                                   -_s_theta,    -_c_theta, 0.0f,
                                    _c_theta,    -_s_theta, 0.0f;  
            break;  
        }

        case LINK_ROLL:
        {
            comRotationMatrix_ <<             _s_phi,          _c_phi,    0.0f,
                                     -_c_phi*_s_theta,  _s_phi*_s_theta, _c_theta,
                                      _c_phi*_c_theta, -_c_theta*_s_phi, _s_theta;
            break;
        }


        case LINK_YAW:
        {

            
            comRotationMatrix_ <<                         -_c_phi*_c_psi,                           _c_phi*_s_psi,          _s_phi,
                                 - _c_theta*_s_psi - _c_psi*_s_phi*_s_theta,   _s_phi*_s_psi*_s_theta - _c_psi*_c_theta, -_c_phi*_s_theta,
                                   _c_psi*_c_theta*_s_phi - _s_psi*_s_theta, - _c_psi*_s_theta - _c_theta*_s_phi*_s_psi,  _c_phi*_c_theta;
 
            break;   
        }

        case LINK_PEDAL: // THis link is measured w.r.t to FRAME_FS
        {

        comRotationMatrix_ <<                          _c_phi*_s_psi,                         _c_phi*_c_psi,          _s_phi,
                               _s_phi*_s_psi*_s_theta - _c_psi*_c_theta, _c_theta*_s_psi + _c_psi*_s_phi*_s_theta, -_c_phi*_s_theta,
                             - _c_psi*_s_theta - _c_theta*_s_phi*_s_psi, _s_psi*_s_theta - _c_psi*_c_theta*_s_phi,  _c_phi*_c_theta;
            break;   
        }

    }

    return comRotationMatrix_;
}

Eigen::Matrix<float, 6, NB_AXIS> Platform::comGeometricJacobian(link_chain link){

    Eigen::Matrix<float, 6, NB_AXIS> comJ;
    comJ.setConstant(0.0f);

    switch (link)
    {
        case LINK_BASE:
        {
            comJ.setConstant(0.0f);
            break;   
        }

        case LINK_Y: {
            comJ(1, 0) = 1.0f;
            break;
        }

        case LINK_X:
        {
            comJ(1,0) = 1.0f;
            comJ(0,1) = 1.0f;
            break;   
        }

        case LINK_PITCH:
        {
            
          comJ<< 0.0f, 1.0f,                                                                  0.0f, 0.0f, 0.0f,
                 1.0f, 0.0f,   LINKS_COM[LINK_PITCH][1]*_s_theta - LINKS_COM[LINK_PITCH][0]*_c_theta, 0.0f, 0.0f,
                 0.0f, 0.0f, - LINKS_COM[LINK_PITCH][1]*_c_theta - LINKS_COM[LINK_PITCH][0]*_s_theta, 0.0f, 0.0f,
                 0.0f, 0.0f,                                                                  1.0f, 0.0f, 0.0f,
                 0.0f, 0.0f,                                                                  0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f,                                                                  0.0f, 0.0f, 0.0f;
            break;  
        }

        case LINK_ROLL:
        {
            comJ(1, 0) = 1.0f;
            comJ(0, 1) = 1.0f;
            comJ(1, 2) = -LINKS_COM[LINK_ROLL][2] * _s_theta -
                         _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -
                         LINKS_COM[LINK_ROLL][1] * _s_phi);
            
            comJ(2, 2) = LINKS_COM[LINK_ROLL][2] * _c_theta -
                         _s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -
                                    LINKS_COM[LINK_ROLL][1] * _s_phi);
            comJ(3, 2) = 1.0f;

            comJ(0, 3) =
                        _c_theta * (LINKS_COM[LINK_ROLL][2] * _s_theta +
                        _c_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -
                        LINKS_COM[LINK_ROLL][1] * _s_phi)) +
                        _s_theta * (_s_theta * (LINKS_COM[LINK_ROLL][0] * _c_phi -
                        LINKS_COM[LINK_ROLL][1] * _s_phi) -
                        LINKS_COM[LINK_ROLL][2] * _c_theta);
            comJ(1, 3) = _s_theta * (LINKS_COM[LINK_ROLL][1] * _c_phi +
                                    LINKS_COM[LINK_ROLL][0] * _s_phi);
            comJ(2, 3) = -_c_theta * (LINKS_COM[LINK_ROLL][1] * _c_phi +
                                     LINKS_COM[LINK_ROLL][0] * _s_phi);
            comJ(4, 3) = _c_theta;
            comJ(5, 3) = _s_theta;
            // comJ<<  0.0f, 1.0f,                                                                                                        0.0f, _c_theta*(LINKS_COM[LINK_ROLL][2]*_s_theta + _c_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi)) + _s_theta*(_s_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi) - LINKS_COM[LINK_ROLL][2]*_c_theta), 0.0f,
            //         1.0f, 0.0f, - LINKS_COM[LINK_ROLL][2]*_s_theta - _c_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi),                                                                                                                                                                   _s_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi), 0.0f,
            //         0.0f, 0.0f,   LINKS_COM[LINK_ROLL][2]*_c_theta - _s_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi),                                                                                                                                                                  -_c_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi), 0.0f,
            //         0.0f, 0.0f,                                                                                                        1.0f,                                                                                                                                                                                                                                      0.0f, 0.0f,
            //         0.0f, 0.0f,                                                                                                        0.0f,                                                                                                                                                                                                                                   _c_theta, 0.0f,
            //         0.0f, 0.0f,                                                                                                        0.0f,                                                                                                                                                                                                                                   _s_theta, 0.0f;
            break;   
        }


        case LINK_YAW:
        {
            comJ(1, 0) = 1.0f;
            comJ(0, 1) = 1.0f;
            comJ(1, 2) = _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                    LINKS_COM[LINK_YAW][0] * _s_psi) -
                         LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -
                         _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                            LINKS_COM[LINK_YAW][1] * _s_psi);
            comJ(2, 2) = -_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                     LINKS_COM[LINK_YAW][0] * _s_psi) -
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta -
                         _s_phi * _s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                            LINKS_COM[LINK_YAW][1] * _s_psi);
            comJ(3,2) = 1.0f;
            comJ(0,3) =
                _c_theta * (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -
                           _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                      LINKS_COM[LINK_YAW][0] * _s_psi) +
                           _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                              LINKS_COM[LINK_YAW][1] * _s_psi)) +
                _s_theta * (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                      LINKS_COM[LINK_YAW][0] * _s_psi) +
                           LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +
                           _s_phi * _s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                              LINKS_COM[LINK_YAW][1] * _s_psi));

            comJ(1,3) = _s_theta * (LINKS_COM[LINK_YAW][2] * _s_phi -
                                    _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                             LINKS_COM[LINK_YAW][1] * _s_psi));
            comJ(2, 3) = -_c_theta * (LINKS_COM[LINK_YAW][2] * _s_phi -
                                     _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                              LINKS_COM[LINK_YAW][1] * _s_psi));
            comJ(4, 3) = _c_theta;
            comJ(5, 3) = _s_theta;
            comJ(0, 4) =
                _c_phi * _c_theta *
                    (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                LINKS_COM[LINK_YAW][0] * _s_psi) +
                     LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +
                     _s_phi * _s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                        LINKS_COM[LINK_YAW][1] * _s_psi)) -
                _c_phi * _s_theta *
                    (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -
                     _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                LINKS_COM[LINK_YAW][0] * _s_psi) +
                     _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                        LINKS_COM[LINK_YAW][1] * _s_psi));
            comJ(1, 4) =
                _c_phi * _c_theta * (LINKS_COM[LINK_YAW][2] * _s_phi -
                                   _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                            LINKS_COM[LINK_YAW][1] * _s_psi)) -
                _s_phi * (LINKS_COM[LINK_YAW][2] * _c_phi * _c_theta -
                         _s_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                    LINKS_COM[LINK_YAW][0] * _s_psi) +
                         _c_theta * _s_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                            LINKS_COM[LINK_YAW][1] * _s_psi));
            comJ(2, 4) =
                _c_phi * _s_theta * (LINKS_COM[LINK_YAW][2] * _s_phi -
                                   _c_phi * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                            LINKS_COM[LINK_YAW][1] * _s_psi)) -
                _s_phi * (_c_theta * (LINKS_COM[LINK_YAW][1] * _c_psi +
                                    LINKS_COM[LINK_YAW][0] * _s_psi) +
                         LINKS_COM[LINK_YAW][2] * _c_phi * _s_theta +
                         _s_phi * _s_theta * (LINKS_COM[LINK_YAW][0] * _c_psi -
                                            LINKS_COM[LINK_YAW][1] * _s_psi));

            comJ(3,4) = _s_phi;
            comJ(4,4) = -_c_phi*_s_theta;
            comJ(5,4) = _c_phi*_c_theta;;                                            
            // comJ<< 0.0f, 1.0f,                                                                                                                                                                                         0.0f, _c_theta*(LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) + _s_theta*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)), _c_phi*_c_theta*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)),
            //        1.0f, 0.0f,   _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) - LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi),                                                                                                                                                                                                                                                                                                _s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)),                                                                                                 _c_phi*_c_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _s_phi*(LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)),
            //        0.0f, 0.0f, - _c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) - LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta - _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi),                                                                                                                                                                                                                                                                                               -_c_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)),                                                                                                 _c_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _s_phi*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)),
            //        0.0f, 0.0f,                                                                                                                                                                                         1.0f,                                                                                                                                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                   _s_phi,
            //        0.0f, 0.0f,                                                                                                                                                                                         0.0f,                                                                                                                                                                                                                                                                                                                                                                                                     _c_theta,                                                                                                                                                                                                                                                                                                                                                                                                          -_c_phi*_s_theta,
            //        0.0f, 0.0f,                                                                                                                                                                                         0.0f,                                                                                                                                                                                                                                                                                                                                                                                                     _s_theta,                                                                                                                                                                                                                                                                                                                                                                                                           _c_phi*_c_theta;
;

            
             break;   
        }

        case LINK_PEDAL:
        {
            comJ(1, 0) = 1.0f;
            comJ(0, 1) = 1.0f;
            comJ(1, 2) = LINKS_COM[LINK_PEDAL][0] *
                             (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) -
                         LINKS_COM[LINK_PEDAL][1] *
                             (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -
                         d6 * _c_phi * _c_theta -
                         LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta;
            comJ(2, 2) = LINKS_COM[LINK_PEDAL][1] *
                             (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) -
                         LINKS_COM[LINK_PEDAL][0] *
                             (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -
                         d6 * _c_phi * _s_theta -
                         LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta;
            comJ(3,2) = 1.0f;
            comJ(1, 3) =
                _c_theta * (LINKS_COM[LINK_PEDAL][1] *
                               (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -
                           LINKS_COM[LINK_PEDAL][0] *
                               (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +
                           d6 * _c_phi * _c_theta +
                           LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta) +
                _s_theta * (LINKS_COM[LINK_PEDAL][0] *
                               (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -
                           LINKS_COM[LINK_PEDAL][1] *
                               (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) +
                           d6 * _c_phi * _s_theta +
                           LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta);
            comJ(2, 3) =
                -_c_theta * (d6 * _s_phi + LINKS_COM[LINK_PEDAL][2] * _s_phi +
                            LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +
                            LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi);
            comJ(4, 3) = _c_theta;
            comJ(5, 3) = _s_theta;
            comJ(0, 4) = _c_phi * _c_theta *
                             (LINKS_COM[LINK_PEDAL][0] *
                                  (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -
                              LINKS_COM[LINK_PEDAL][1] *
                                  (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) +
                              d6 * _c_phi * _s_theta +
                              LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta) -
                         _c_phi * _s_theta *
                             (LINKS_COM[LINK_PEDAL][1] *
                                  (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -
                              LINKS_COM[LINK_PEDAL][0] *
                                  (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +
                              d6 * _c_phi * _c_theta +
                              LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta);
            comJ(1,4) =
                      _c_phi * _c_theta *
                          (d6 * _s_phi + LINKS_COM[LINK_PEDAL][2] * _s_phi +
                           LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +
                           LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -
                      _s_phi * (LINKS_COM[LINK_PEDAL][1] *
                                   (_s_psi * _s_theta - _c_psi * _c_theta * _s_phi) -
                               LINKS_COM[LINK_PEDAL][0] *
                                   (_c_psi * _s_theta + _c_theta * _s_phi * _s_psi) +
                               d6 * _c_phi * _c_theta +
                               LINKS_COM[LINK_PEDAL][2] * _c_phi * _c_theta);
            comJ(2,4) = 
                     _c_phi * _s_theta *
                         (d6 * _s_phi + LINKS_COM[LINK_PEDAL][2] * _s_phi +
                          LINKS_COM[LINK_PEDAL][1] * _c_phi * _c_psi +
                          LINKS_COM[LINK_PEDAL][0] * _c_phi * _s_psi) -
                     _s_phi * (LINKS_COM[LINK_PEDAL][0] *
                                  (_c_psi * _c_theta - _s_phi * _s_psi * _s_theta) -
                              LINKS_COM[LINK_PEDAL][1] *
                                  (_c_theta * _s_psi + _c_psi * _s_phi * _s_theta) +
                              d6 * _c_phi * _s_theta +
                              LINKS_COM[LINK_PEDAL][2] * _c_phi * _s_theta);
            comJ(3,4) =          _s_phi;
            comJ(4,4) = -_c_phi*_s_theta;
            comJ(5,4) =  _c_phi*_c_theta;                  
            //comJ<< 0.0f, 1.0f,                                                                                                                                                                                        0.0f, _c_theta*(LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) + d6*_c_phi*_c_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta) + _s_theta*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta), _c_phi*_c_theta*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta) - _c_phi*_s_theta*(LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) + d6*_c_phi*_c_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta),
            //       1.0f, 0.0f, LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) - LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - d6*_c_phi*_c_theta - LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta,                                                                                                                                                                                                                                                                             _s_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi),                                                                             _c_phi*_c_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi) - _s_phi*(LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) + d6*_c_phi*_c_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta),
            //       0.0f, 0.0f, LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - d6*_c_phi*_s_theta - LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta,                                                                                                                                                                                                                                                                            -_c_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi),                                                                             _c_phi*_s_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi) - _s_phi*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta),
            //       0.0f, 0.0f,                                                                                                                                                                                        1.0f,                                                                                                                                                                                                                                                                                                                                                                                                          0.0f,                                                                                                                                                                                                                                                                                                                                                                                                                     _s_phi,
            //       0.0f, 0.0f,                                                                                                                                                                                        0.0f,                                                                                                                                                                                                                                                                                                                                                                                                       _c_theta,                                                                                                                                                                                                                                                                                                                                                                                                            -_c_phi*_s_theta,
            //       0.0f, 0.0f,                                                                                                                                                                                        0.0f,                                                                                                                                                                                                                                                                                                                                                                                                       _s_theta,                                                                                                                                                                                                                                                                                                                                                                                                             _c_phi*_c_theta;
 
            break;   
        }  
    }
  return comJ;
}

Eigen::Matrix4f Platform::dhTransform(float r,float d,float alpha,float beta)
{
    Eigen::Matrix4f T_x, T_z;
    Eigen::Matrix3f R_x, R_z;
    Eigen::Vector4f p_x, p_z;

    T_z << cos(beta), -sin(beta), 0.0f, 0.0f,
           sin(beta),  cos(beta), 0.0f, 0.0f,
                0.0f,       0.0f, 1.0f,    d,
                0.0f,       0.0f, 0.0f, 1.0f;  

    T_x<<1.0f,        0.0f,        0.0f,    r, 
         0.0f,  cos(alpha), -sin(alpha), 0.0f,
         0.0f,  sin(alpha),  cos(alpha), 0.0f,
         0.0f,        0.0f,        0.0f, 1.0f;

    return T_z*T_x;

}

#if (CORIOLIS_DEV_STRATEGY == CORIOLIS_KRONECKER)

Eigen::Matrix<float, 6, NB_AXIS * NB_AXIS> Platform::devQComGeomJacobian(link_chain link)
{
    Eigen::Matrix<float, 6, NB_AXIS * NB_AXIS> devComJ;
    devComJ.setConstant(0.0f);

    switch (link)
    {
        case LINK_BASE ... LINK_X:
        {
            devComJ.setConstant(0.0f); //-> already zero before the switchCase
            break;   
        }

        case LINK_PITCH:
        {
            devComJ(12,1)=LINKS_COM[LINK_PITCH][1]* _c_theta + LINKS_COM[LINK_PITCH][0]*_s_theta;
            devComJ(12, 2) = LINKS_COM[LINK_PITCH][1] * _s_theta - LINKS_COM[LINK_PITCH][0] * _c_theta;
            break;  

        //    devComJ<<[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        //             [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, p_y*cos(theta) + p_x*sin(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        //             [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, p_y*sin(theta) - p_x*cos(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        //             [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        //             [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        //             [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        }

        case LINK_ROLL:
        {
            devComJ(12, 1) =   _s_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi) - LINKS_COM[LINK_ROLL][2]*_c_theta;
            devComJ(12, 2) = - LINKS_COM[LINK_ROLL][2]*_s_theta - _c_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi);               
            devComJ(13, 1) =  _c_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi) ;
            devComJ(13, 2) =  _s_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi) ;
            devComJ(17, 1) =  _c_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi);
            devComJ(17, 2) =  _s_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi);
            devComJ(17, 4) = -_s_theta;
            devComJ(17, 5) =  _c_theta;
            devComJ(18, 0) = - _c_theta*_c_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi) - _s_theta*_s_theta*(LINKS_COM[LINK_ROLL][1]*_c_phi + LINKS_COM[LINK_ROLL][0]*_s_phi);
            devComJ(18, 1) =                                                  _s_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi);
            devComJ(18, 2) =                                                 -_c_theta*(LINKS_COM[LINK_ROLL][0]*_c_phi - LINKS_COM[LINK_ROLL][1]*_s_phi);
        // devComJ<<  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                           0,                                        0, 0, 0, 0,                                        0, - cos(theta)^2*(r_y*cos(phi) + r_x*sin(phi)) - sin(theta)^2*(r_y*cos(phi) + r_x*sin(phi)), 0, 0, 0, 0, 0, 0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   sin(theta)*(r_x*cos(phi) - r_y*sin(phi)) - r_z*cos(theta), cos(theta)*(r_y*cos(phi) + r_x*sin(phi)), 0, 0, 0, cos(theta)*(r_y*cos(phi) + r_x*sin(phi)),                                                  sin(theta)*(r_x*cos(phi) - r_y*sin(phi)), 0, 0, 0, 0, 0, 0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, - r_z*sin(theta) - cos(theta)*(r_x*cos(phi) - r_y*sin(phi)), sin(theta)*(r_y*cos(phi) + r_x*sin(phi)), 0, 0, 0, sin(theta)*(r_y*cos(phi) + r_x*sin(phi)),                                                 -cos(theta)*(r_x*cos(phi) - r_y*sin(phi)), 0, 0, 0, 0, 0, 0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                           0,                                        0, 0, 0, 0,                                        0,                                                                                         0, 0, 0, 0, 0, 0, 0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                           0,                                        0, 0, 0, 0,                              -sin(theta),                                                                                         0, 0, 0, 0, 0, 0, 0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                           0,                                        0, 0, 0, 0,                               cos(theta),                                                                                         0, 0, 0, 0, 0, 0, 0]

            break;   
        }


        case LINK_YAW:
        {
            devComJ(12, 1) = _c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi);
            devComJ(12, 2) = _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) - LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi);
            devComJ(13, 1) = LINKS_COM[LINK_YAW][2]*_c_theta*_s_phi - _c_phi*_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)  ;
            devComJ(13, 2) = LINKS_COM[LINK_YAW][2]*_s_phi*_s_theta - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)  ;
            devComJ(14, 1) = _s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi);
            devComJ(14, 2) = _s_phi*_s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) - _c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi);
            devComJ(17, 1) = _c_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi))  ;
            devComJ(17, 2) = _s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi))  ;
            devComJ(17, 4) = -_s_theta  ;
            devComJ(17, 5) =  _c_theta  ;
            devComJ(18, 0) =  - _s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi*_s_theta - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_theta*(LINKS_COM[LINK_YAW][2]*_c_theta*_s_phi - _c_phi*_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) ;
            devComJ(18, 1) =   _s_theta*(_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi) ;
            devComJ(18, 2) =  -_c_theta*(_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi) ;
            devComJ(19, 0) = _s_theta*(_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) - _s_phi*_s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi)) - _c_theta*(_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi));
            devComJ(19, 1) =   _c_phi*_s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi);
            devComJ(19, 2) =  -_c_phi*_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi);
            devComJ(22, 1) =  _s_phi*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)); 
            devComJ(22, 2) =  _c_phi*_c_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _s_phi*(LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)); 
            devComJ(22, 4) =  -_c_phi*_c_theta;
            devComJ(22, 5) =  -_c_phi*_s_theta;
            devComJ(23, 0) = _s_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_theta*_s_phi*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) + _c_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_c_theta*_s_phi - _c_phi*_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_phi*_c_theta*(LINKS_COM[LINK_YAW][2]*_s_phi*_s_theta - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi));
            devComJ(23, 1) = _s_phi*(LINKS_COM[LINK_YAW][2]*_c_theta*_s_phi - _c_phi*_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_phi*(LINKS_COM[LINK_YAW][2]*_c_phi*_c_theta - _s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) + _c_phi*_c_theta*(_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi) - _c_theta*_s_phi*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi));
            devComJ(23, 2) = _s_phi*(LINKS_COM[LINK_YAW][2]*_s_phi*_s_theta - _c_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) - _c_phi*(_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi*_s_theta + _s_phi*_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi)) + _c_phi*_s_theta*(_s_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + LINKS_COM[LINK_YAW][2]*_c_phi) - _s_phi*_s_theta*(LINKS_COM[LINK_YAW][2]*_s_phi - _c_phi*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi));
            devComJ(23, 3) =             _c_phi;
            devComJ(23, 4) =  _s_phi*_s_theta;
            devComJ(23, 5) = -_c_theta*_s_phi;
            devComJ(24, 0) = _c_phi*_c_theta*(_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) - _s_phi*_s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi)) + _c_phi*_s_theta*(_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi)) ;
            devComJ(24, 1) = _s_phi*(_s_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) + _c_theta*_s_phi*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi)) + _c_phi*_c_phi*_c_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) ;
            devComJ(24, 2) = _c_phi*_c_phi*_s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi) - _s_phi*(_c_theta*(LINKS_COM[LINK_YAW][0]*_c_psi - LINKS_COM[LINK_YAW][1]*_s_psi) - _s_phi*_s_theta*(LINKS_COM[LINK_YAW][1]*_c_psi + LINKS_COM[LINK_YAW][0]*_s_psi)) ;
        // devComJ<<  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                           0,                                                                              0,                                                                                                0, 0, 0,                                                                     0, - sin(theta)*(yw_z*sin(phi)*sin(theta) - cos(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) - cos(theta)*(yw_z*cos(theta)*sin(phi) - cos(phi)*cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi))), sin(theta)*(cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) - sin(phi)*sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi))) - cos(theta)*(sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) + cos(theta)*sin(phi)*(yw_y*cos(psi) + yw_x*sin(psi))), 0, 0,                                                                                                                                                                                                                       0, sin(phi)*sin(theta)*(yw_z*cos(phi)*cos(theta) - sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + cos(theta)*sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi))) - cos(theta)*sin(phi)*(cos(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + yw_z*cos(phi)*sin(theta) + sin(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) + cos(phi)*sin(theta)*(yw_z*cos(theta)*sin(phi) - cos(phi)*cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) - cos(phi)*cos(theta)*(yw_z*sin(phi)*sin(theta) - cos(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi))), cos(phi)*cos(theta)*(cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) - sin(phi)*sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi))) + cos(phi)*sin(theta)*(sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) + cos(theta)*sin(phi)*(yw_y*cos(psi) + yw_x*sin(psi)))]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cos(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + yw_z*cos(phi)*sin(theta) + sin(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi)), yw_z*cos(theta)*sin(phi) - cos(phi)*cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi)), sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) + cos(theta)*sin(phi)*(yw_y*cos(psi) + yw_x*sin(psi)), 0, 0, cos(theta)*(yw_z*sin(phi) - cos(phi)*(yw_x*cos(psi) - yw_y*sin(psi))),                                                                                                                       sin(theta)*(sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi)) + yw_z*cos(phi)),                                                                                                                                                                           cos(phi)*sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)), 0, 0, sin(phi)*(cos(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + yw_z*cos(phi)*sin(theta) + sin(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) - cos(phi)*sin(theta)*(yw_z*sin(phi) - cos(phi)*(yw_x*cos(psi) - yw_y*sin(psi))),                                                                                                                sin(phi)*(yw_z*cos(theta)*sin(phi) - cos(phi)*cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) - cos(phi)*(yw_z*cos(phi)*cos(theta) - sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + cos(theta)*sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi))) + cos(phi)*cos(theta)*(sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi)) + yw_z*cos(phi)) - cos(theta)*sin(phi)*(yw_z*sin(phi) - cos(phi)*(yw_x*cos(psi) - yw_y*sin(psi))),                                                                             sin(phi)*(sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) + cos(theta)*sin(phi)*(yw_y*cos(psi) + yw_x*sin(psi))) + cos(phi)^2*cos(theta)*(yw_y*cos(psi) + yw_x*sin(psi))]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) - yw_z*cos(phi)*cos(theta) - cos(theta)*sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi)), yw_z*sin(phi)*sin(theta) - cos(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi)), sin(phi)*sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) - cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi)), 0, 0, sin(theta)*(yw_z*sin(phi) - cos(phi)*(yw_x*cos(psi) - yw_y*sin(psi))),                                                                                                                      -cos(theta)*(sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi)) + yw_z*cos(phi)),                                                                                                                                                                          -cos(phi)*cos(theta)*(yw_y*cos(psi) + yw_x*sin(psi)), 0, 0, cos(phi)*cos(theta)*(yw_z*sin(phi) - cos(phi)*(yw_x*cos(psi) - yw_y*sin(psi))) - sin(phi)*(yw_z*cos(phi)*cos(theta) - sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + cos(theta)*sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi))),                                                                                                                sin(phi)*(yw_z*sin(phi)*sin(theta) - cos(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) - cos(phi)*(cos(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) + yw_z*cos(phi)*sin(theta) + sin(phi)*sin(theta)*(yw_x*cos(psi) - yw_y*sin(psi))) + cos(phi)*sin(theta)*(sin(phi)*(yw_x*cos(psi) - yw_y*sin(psi)) + yw_z*cos(phi)) - sin(phi)*sin(theta)*(yw_z*sin(phi) - cos(phi)*(yw_x*cos(psi) - yw_y*sin(psi))),                                                                             cos(phi)^2*sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)) - sin(phi)*(cos(theta)*(yw_x*cos(psi) - yw_y*sin(psi)) - sin(phi)*sin(theta)*(yw_y*cos(psi) + yw_x*sin(psi)))]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                           0,                                                                              0,                                                                                                0, 0, 0,                                                                     0,                                                                                                                                                                                           0,                                                                                                                                                                                                                             0, 0, 0,                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            cos(phi),                                                                                                                                                                                                                                               0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                           0,                                                                              0,                                                                                                0, 0, 0,                                                           -sin(theta),                                                                                                                                                                                           0,                                                                                                                                                                                                                             0, 0, 0,                                                                                                                                                                                                    -cos(phi)*cos(theta),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 sin(phi)*sin(theta),                                                                                                                                                                                                                                               0]
        //            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                           0,                                                                              0,                                                                                                0, 0, 0,                                                            cos(theta),                                                                                                                                                                                           0,                                                                                                                                                                                                                             0, 0, 0,                                                                                                                                                                                                    -cos(phi)*sin(theta),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                -cos(theta)*sin(phi),                                                                                                                                                                                                                                               0]           
             break;   
        }

        case LINK_PEDAL:
        {
            devComJ(12, 1) = LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta;
            devComJ(12, 2) =  LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) - LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - d6*_c_phi*_c_theta - LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta;
            devComJ(13, 1) = d6*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][2]*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_c_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_theta*_s_psi;
            devComJ(13, 2) =  d6*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_s_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi*_s_theta;
            devComJ(14, 1) = - LINKS_COM[LINK_PEDAL][0]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][1]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi);
            devComJ(14, 2) =    LINKS_COM[LINK_PEDAL][0]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta);
            devComJ(17, 1) = _c_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi);
            devComJ(17, 2) =  _s_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi);
            devComJ(17, 4) = -_s_theta;
            devComJ(17, 5) =  _c_theta;
            devComJ(18, 0) = - _c_theta*(d6*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][2]*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_c_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_theta*_s_psi) - _s_theta*(d6*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_s_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi*_s_theta);
            devComJ(18, 1) =  _s_theta*(d6*_c_phi + LINKS_COM[LINK_PEDAL][2]*_c_phi - LINKS_COM[LINK_PEDAL][1]*_c_psi*_s_phi - LINKS_COM[LINK_PEDAL][0]*_s_phi*_s_psi);
            devComJ(18, 2) =  -_c_theta*(d6*_c_phi + LINKS_COM[LINK_PEDAL][2]*_c_phi - LINKS_COM[LINK_PEDAL][1]*_c_psi*_s_phi - LINKS_COM[LINK_PEDAL][0]*_s_phi*_s_psi);
            devComJ(19, 0) =  _c_theta*(LINKS_COM[LINK_PEDAL][0]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi)) - _s_theta*(LINKS_COM[LINK_PEDAL][0]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta));
            devComJ(19, 1) =   _s_theta*(LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_psi - LINKS_COM[LINK_PEDAL][1]*_c_phi*_s_psi);
            devComJ(19, 2) =  -_c_theta*(LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_psi - LINKS_COM[LINK_PEDAL][1]*_c_phi*_s_psi);
            devComJ(22, 1) =  _s_phi*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta) - _c_phi*_s_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi);
            devComJ(22, 2) =  _c_phi*_c_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi) - _s_phi*(LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) + d6*_c_phi*_c_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta);
            devComJ(22, 4) =  -_c_phi*_c_theta;
            devComJ(22, 5) =  -_c_phi*_s_theta;
            devComJ(23, 0) =  _s_phi*_s_theta*(LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) + d6*_c_phi*_c_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta) - _c_theta*_s_phi*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta) + _c_phi*_s_theta*(d6*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][2]*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_c_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_theta*_s_psi) - _c_phi*_c_theta*(d6*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_s_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi*_s_theta);
            devComJ(23, 1) =  _s_phi*(d6*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][2]*_c_theta*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_c_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_theta*_s_psi) - _c_phi*(LINKS_COM[LINK_PEDAL][1]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) - LINKS_COM[LINK_PEDAL][0]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi) + d6*_c_phi*_c_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_c_theta) + _c_phi*_c_theta*(d6*_c_phi + LINKS_COM[LINK_PEDAL][2]*_c_phi - LINKS_COM[LINK_PEDAL][1]*_c_psi*_s_phi - LINKS_COM[LINK_PEDAL][0]*_s_phi*_s_psi) - _c_theta*_s_phi*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi);
            devComJ(23, 2) =  _s_phi*(d6*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_s_phi*_s_theta + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi*_s_theta + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi*_s_theta) - _c_phi*(LINKS_COM[LINK_PEDAL][0]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta) - LINKS_COM[LINK_PEDAL][1]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + d6*_c_phi*_s_theta + LINKS_COM[LINK_PEDAL][2]*_c_phi*_s_theta) + _c_phi*_s_theta*(d6*_c_phi + LINKS_COM[LINK_PEDAL][2]*_c_phi - LINKS_COM[LINK_PEDAL][1]*_c_psi*_s_phi - LINKS_COM[LINK_PEDAL][0]*_s_phi*_s_psi) - _s_phi*_s_theta*(d6*_s_phi + LINKS_COM[LINK_PEDAL][2]*_s_phi + LINKS_COM[LINK_PEDAL][1]*_c_phi*_c_psi + LINKS_COM[LINK_PEDAL][0]*_c_phi*_s_psi);
            devComJ(23, 3) =   _c_phi;
            devComJ(23, 4) =   _s_phi*_s_theta;
            devComJ(23, 5) =  -_c_theta*_s_phi;
            devComJ(24, 0) =  - _c_phi*_s_theta*(LINKS_COM[LINK_PEDAL][0]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi)) - _c_phi*_c_theta*(LINKS_COM[LINK_PEDAL][0]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta));
            devComJ(24, 1) =  _c_phi*_c_theta*(LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_psi - LINKS_COM[LINK_PEDAL][1]*_c_phi*_s_psi) - _s_phi*(LINKS_COM[LINK_PEDAL][0]*(_s_psi*_s_theta - _c_psi*_c_theta*_s_phi) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_s_theta + _c_theta*_s_phi*_s_psi));
            devComJ(24, 2) =  _s_phi*(LINKS_COM[LINK_PEDAL][0]*(_c_theta*_s_psi + _c_psi*_s_phi*_s_theta) + LINKS_COM[LINK_PEDAL][1]*(_c_psi*_c_theta - _s_phi*_s_psi*_s_theta)) + _c_phi*_s_theta*(LINKS_COM[LINK_PEDAL][0]*_c_phi*_c_psi - LINKS_COM[LINK_PEDAL][1]*_c_phi*_s_psi);

            //devComJ<< [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                  0,                                                                                                                                  0,                                                                                                                             0, 0, 0,                                                                                                   0, - cos(theta)*(d6*cos(theta)*sin(phi) + pedal_z*cos(theta)*sin(phi) + pedal_y*cos(phi)*cos(psi)*cos(theta) + pedal_x*cos(phi)*cos(theta)*sin(psi)) - sin(theta)*(d6*sin(phi)*sin(theta) + pedal_z*sin(phi)*sin(theta) + pedal_y*cos(phi)*cos(psi)*sin(theta) + pedal_x*cos(phi)*sin(psi)*sin(theta)), cos(theta)*(pedal_x*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + pedal_y*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))) - sin(theta)*(pedal_x*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + pedal_y*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))), 0, 0,                                                                                                                                                                                                                                                                                                            0, sin(phi)*sin(theta)*(pedal_y*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - pedal_x*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + d6*cos(phi)*cos(theta) + pedal_z*cos(phi)*cos(theta)) - cos(theta)*sin(phi)*(pedal_x*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - pedal_y*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + d6*cos(phi)*sin(theta) + pedal_z*cos(phi)*sin(theta)) + cos(phi)*sin(theta)*(d6*cos(theta)*sin(phi) + pedal_z*cos(theta)*sin(phi) + pedal_y*cos(phi)*cos(psi)*cos(theta) + pedal_x*cos(phi)*cos(theta)*sin(psi)) - cos(phi)*cos(theta)*(d6*sin(phi)*sin(theta) + pedal_z*sin(phi)*sin(theta) + pedal_y*cos(phi)*cos(psi)*sin(theta) + pedal_x*cos(phi)*sin(psi)*sin(theta)), - cos(phi)*sin(theta)*(pedal_x*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + pedal_y*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))) - cos(phi)*cos(theta)*(pedal_x*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + pedal_y*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)))]
            //          [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pedal_x*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - pedal_y*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + d6*cos(phi)*sin(theta) + pedal_z*cos(phi)*sin(theta), d6*cos(theta)*sin(phi) + pedal_z*cos(theta)*sin(phi) + pedal_y*cos(phi)*cos(psi)*cos(theta) + pedal_x*cos(phi)*cos(theta)*sin(psi), - pedal_x*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - pedal_y*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), 0, 0, cos(theta)*(d6*sin(phi) + pedal_z*sin(phi) + pedal_y*cos(phi)*cos(psi) + pedal_x*cos(phi)*sin(psi)),                                                                                                                                                                                                 sin(theta)*(d6*cos(phi) + pedal_z*cos(phi) - pedal_y*cos(psi)*sin(phi) - pedal_x*sin(phi)*sin(psi)),                                                                                                                                                                                                                  sin(theta)*(pedal_x*cos(phi)*cos(psi) - pedal_y*cos(phi)*sin(psi)), 0, 0, sin(phi)*(pedal_x*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - pedal_y*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + d6*cos(phi)*sin(theta) + pedal_z*cos(phi)*sin(theta)) - cos(phi)*sin(theta)*(d6*sin(phi) + pedal_z*sin(phi) + pedal_y*cos(phi)*cos(psi) + pedal_x*cos(phi)*sin(psi)),                                                                                                                                                               sin(phi)*(d6*cos(theta)*sin(phi) + pedal_z*cos(theta)*sin(phi) + pedal_y*cos(phi)*cos(psi)*cos(theta) + pedal_x*cos(phi)*cos(theta)*sin(psi)) - cos(phi)*(pedal_y*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - pedal_x*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + d6*cos(phi)*cos(theta) + pedal_z*cos(phi)*cos(theta)) + cos(phi)*cos(theta)*(d6*cos(phi) + pedal_z*cos(phi) - pedal_y*cos(psi)*sin(phi) - pedal_x*sin(phi)*sin(psi)) - cos(theta)*sin(phi)*(d6*sin(phi) + pedal_z*sin(phi) + pedal_y*cos(phi)*cos(psi) + pedal_x*cos(phi)*sin(psi)),                                                                                    cos(phi)*cos(theta)*(pedal_x*cos(phi)*cos(psi) - pedal_y*cos(phi)*sin(psi)) - sin(phi)*(pedal_x*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + pedal_y*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)))]
            //          [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pedal_x*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - pedal_y*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - d6*cos(phi)*cos(theta) - pedal_z*cos(phi)*cos(theta), d6*sin(phi)*sin(theta) + pedal_z*sin(phi)*sin(theta) + pedal_y*cos(phi)*cos(psi)*sin(theta) + pedal_x*cos(phi)*sin(psi)*sin(theta),   pedal_x*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + pedal_y*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)), 0, 0, sin(theta)*(d6*sin(phi) + pedal_z*sin(phi) + pedal_y*cos(phi)*cos(psi) + pedal_x*cos(phi)*sin(psi)),                                                                                                                                                                                                -cos(theta)*(d6*cos(phi) + pedal_z*cos(phi) - pedal_y*cos(psi)*sin(phi) - pedal_x*sin(phi)*sin(psi)),                                                                                                                                                                                                                 -cos(theta)*(pedal_x*cos(phi)*cos(psi) - pedal_y*cos(phi)*sin(psi)), 0, 0, cos(phi)*cos(theta)*(d6*sin(phi) + pedal_z*sin(phi) + pedal_y*cos(phi)*cos(psi) + pedal_x*cos(phi)*sin(psi)) - sin(phi)*(pedal_y*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - pedal_x*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + d6*cos(phi)*cos(theta) + pedal_z*cos(phi)*cos(theta)),                                                                                                                                                               sin(phi)*(d6*sin(phi)*sin(theta) + pedal_z*sin(phi)*sin(theta) + pedal_y*cos(phi)*cos(psi)*sin(theta) + pedal_x*cos(phi)*sin(psi)*sin(theta)) - cos(phi)*(pedal_x*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - pedal_y*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + d6*cos(phi)*sin(theta) + pedal_z*cos(phi)*sin(theta)) + cos(phi)*sin(theta)*(d6*cos(phi) + pedal_z*cos(phi) - pedal_y*cos(psi)*sin(phi) - pedal_x*sin(phi)*sin(psi)) - sin(phi)*sin(theta)*(d6*sin(phi) + pedal_z*sin(phi) + pedal_y*cos(phi)*cos(psi) + pedal_x*cos(phi)*sin(psi)),                                                                                    sin(phi)*(pedal_x*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + pedal_y*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + cos(phi)*sin(theta)*(pedal_x*cos(phi)*cos(psi) - pedal_y*cos(phi)*sin(psi))]
            //          [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                  0,                                                                                                                                  0,                                                                                                                             0, 0, 0,                                                                                                   0,                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                   0, 0, 0,                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  cos(phi),                                                                                                                                                                                                                                                                                                       0]
            //          [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                  0,                                                                                                                                  0,                                                                                                                             0, 0, 0,                                                                                         -sin(theta),                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                   0, 0, 0,                                                                                                                                                                                                                                                                                         -cos(phi)*cos(theta),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       sin(phi)*sin(theta),                                                                                                                                                                                                                                                                                                       0]
            //          [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                  0,                                                                                                                                  0,                                                                                                                             0, 0, 0,                                                                                          cos(theta),                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                   0, 0, 0,                                                                                                                                                                                                                                                                                         -cos(phi)*sin(theta),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      -cos(theta)*sin(phi),                                                                                                                                                                                                                                                                                                       0]
 
            break;   
        }  
    }
 return devComJ;
}

Eigen::Matrix<float, NB_CART_AXIS, NB_CART_AXIS * NB_AXIS>
Platform::devQComRotationMatrix(link_chain link)
{

  Eigen::Matrix<float, NB_CART_AXIS, NB_CART_AXIS * NB_AXIS> devComRotationMatrix_;
  devComRotationMatrix_.setIdentity();



    switch (link)
    {
        case LINK_BASE ... LINK_X:
        {
            devComRotationMatrix_.setZero();
            break;   
        }

        case LINK_PITCH:
        {
            devComRotationMatrix_(1,2) = -_c_theta;
            devComRotationMatrix_(2,2) = -_s_theta;
            devComRotationMatrix_(1,7) =  _s_theta;
            devComRotationMatrix_(2,7) = -_c_theta;
            // devComRotationMatrix_ << [ 0, 0,           0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0, 0]
            //                          [ 0, 0, -cos(theta), 0, 0, 0, 0,  sin(theta), 0, 0, 0, 0, 0, 0, 0]
            //                          [ 0, 0, -sin(theta), 0, 0, 0, 0, -cos(theta), 0, 0, 0, 0, 0, 0, 0]
            break;  
        }

        case LINK_ROLL:
        {
            devComRotationMatrix_(1,2) = -_c_phi*_c_theta;
            devComRotationMatrix_(2,2) = -_c_phi*_s_theta;
            devComRotationMatrix_(0,3) =             _c_phi;
            devComRotationMatrix_(1,3) =  _s_phi*_s_theta;
            devComRotationMatrix_(2,3) = -_c_theta*_s_phi;
            devComRotationMatrix_(1,7) = _c_theta*_s_phi;
            devComRotationMatrix_(2,7) = _s_phi*_s_theta;
            devComRotationMatrix_(0,8) =            -_s_phi;
            devComRotationMatrix_(1,8) =  _c_phi*_s_theta;
            devComRotationMatrix_(2,8) = -_c_phi*_c_theta;
            devComRotationMatrix_(1,12) = -_s_theta;
            devComRotationMatrix_(2,12) = _c_theta;

         //devComRotationMatrix_ <<  [ 0, 0,                    0,             cos(phi), 0, 0, 0,                   0,            -sin(phi), 0, 0, 0,           0, 0, 0]
         //                          [ 0, 0, -cos(phi)*cos(theta),  sin(phi)*sin(theta), 0, 0, 0, cos(theta)*sin(phi),  cos(phi)*sin(theta), 0, 0, 0, -sin(theta), 0, 0]
         //                          [ 0, 0, -cos(phi)*sin(theta), -cos(theta)*sin(phi), 0, 0, 0, sin(phi)*sin(theta), -cos(phi)*cos(theta), 0, 0, 0,  cos(theta), 0, 0]
            break;
        }


        case LINK_YAW:
        {
            devComRotationMatrix_(1,2)  =    _s_psi*_s_theta - _c_psi*_c_theta*_s_phi ;
            devComRotationMatrix_(2,2)  =  - _c_theta*_s_psi - _c_psi*_s_phi*_s_theta ;
            devComRotationMatrix_(0,3)  =   _c_psi*_s_phi ;
            devComRotationMatrix_(1,3)  =  -_c_phi*_c_psi*_s_theta ;
            devComRotationMatrix_(2,3)  =   _c_phi*_c_psi*_c_theta ;
            devComRotationMatrix_(0,4)  =   _c_phi*_s_psi;
            devComRotationMatrix_(1,4)  =    _s_phi*_s_psi*_s_theta - _c_psi*_c_theta;
            devComRotationMatrix_(2,4)  =  - _c_psi*_s_theta - _c_theta*_s_phi*_s_psi;
            devComRotationMatrix_(1,7)  =   _c_psi*_s_theta + _c_theta*_s_phi*_s_psi;
            devComRotationMatrix_(2,7)  =   _s_phi*_s_psi*_s_theta - _c_psi*_c_theta;
            devComRotationMatrix_(0,8)  =   -_s_phi*_s_psi;
            devComRotationMatrix_(1,8)  =    _c_phi*_s_psi*_s_theta;
            devComRotationMatrix_(2,8)  =   -_c_phi*_c_theta*_s_psi;
            devComRotationMatrix_(0,9)  =    _c_phi*_c_psi;
            devComRotationMatrix_(1,9)  =   _c_theta*_s_psi + _c_psi*_s_phi*_s_theta;
            devComRotationMatrix_(2,9)  =   _s_psi*_s_theta - _c_psi*_c_theta*_s_phi;
            devComRotationMatrix_(1,12) =   -_c_phi*_c_theta;
            devComRotationMatrix_(2,12) =   -_c_phi*_s_theta;
            devComRotationMatrix_(0,13) =    _c_phi;
            devComRotationMatrix_(1,13) =   _s_phi*_s_theta;
            devComRotationMatrix_(2,13) =  -_c_theta*_s_phi;


         //devComRotationMatrix_ << [ 0, 0,                                                    0,             cos(psi)*sin(phi),                                    cos(phi)*sin(psi), 0, 0,                                                  0,            -sin(phi)*sin(psi),                                  cos(phi)*cos(psi), 0, 0,                    0,             cos(phi), 0]
         //                         [ 0, 0,   sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), -cos(phi)*cos(psi)*sin(theta),   sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta), 0, 0, cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),  cos(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), 0, 0, -cos(phi)*cos(theta),  sin(phi)*sin(theta), 0]
         //                         [ 0, 0, - cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi)*cos(theta), - cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi), 0, 0, sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta), -cos(phi)*cos(theta)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), 0, 0, -cos(phi)*sin(theta), -cos(theta)*sin(phi), 0]

            break;   
        }

        case LINK_PEDAL: // THis link is measured w.r.t to FRAME_FS
        {
            devComRotationMatrix_(1,2)  =   _c_psi*_s_theta + _c_theta*_s_phi*_s_psi;
            devComRotationMatrix_(2,2)  =   _s_phi*_s_psi*_s_theta - _c_psi*_c_theta;
            devComRotationMatrix_(0,3)  =   -_s_phi*_s_psi;
            devComRotationMatrix_(1,3)  =    _c_phi*_s_psi*_s_theta;
            devComRotationMatrix_(2,3)  =   -_c_phi*_c_theta*_s_psi;
            devComRotationMatrix_(0,4)  =   _c_phi*_c_psi;
            devComRotationMatrix_(1,4)  =   _c_theta*_s_psi + _c_psi*_s_phi*_s_theta;
            devComRotationMatrix_(2,4)  =   _s_psi*_s_theta - _c_psi*_c_theta*_s_phi;
            devComRotationMatrix_(1,7)  =   _c_psi*_c_theta*_s_phi - _s_psi*_s_theta;
            devComRotationMatrix_(2,7)  =   _c_theta*_s_psi + _c_psi*_s_phi*_s_theta;
            devComRotationMatrix_(0,8)  =   -_c_psi*_s_phi;
            devComRotationMatrix_(1,8)  =    _c_phi*_c_psi*_s_theta;
            devComRotationMatrix_(2,8)  =   -_c_phi*_c_psi*_c_theta;
            devComRotationMatrix_(0,9)  =   -_c_phi*_s_psi;
            devComRotationMatrix_(1,9)  =   _c_psi*_c_theta - _s_phi*_s_psi*_s_theta;
            devComRotationMatrix_(2,9)  =   _c_psi*_s_theta + _c_theta*_s_phi*_s_psi;
            devComRotationMatrix_(1,12) =   -_c_phi*_c_theta;
            devComRotationMatrix_(2,12) =   -_c_phi*_s_theta;
            devComRotationMatrix_(0,13) =    _c_phi;
            devComRotationMatrix_(1,13) =    _s_phi*_s_theta;
            devComRotationMatrix_(2,13) =   -_c_theta*_s_phi;

         //devComRotationMatrix_ << [ 0, 0,                                                  0,            -sin(phi)*sin(psi),                                  cos(phi)*cos(psi), 0, 0,                                                  0,            -cos(psi)*sin(phi),                                 -cos(phi)*sin(psi), 0, 0,                    0,             cos(phi), 0]
         //                         [ 0, 0, cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),  cos(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), 0, 0, cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta),  cos(phi)*cos(psi)*sin(theta), cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), 0, 0, -cos(phi)*cos(theta),  sin(phi)*sin(theta), 0]
         //                         [ 0, 0, sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta), -cos(phi)*cos(theta)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), 0, 0, cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*cos(psi)*cos(theta), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), 0, 0, -cos(phi)*sin(theta), -cos(theta)*sin(phi), 0]
            break;   
        }
    }

    return devComRotationMatrix_;

}

#endif