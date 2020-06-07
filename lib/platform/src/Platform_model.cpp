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

extern const float LINKS_COM[NB_LINKS][3] = {{0.06506077f, 0.23761375f, 0.06685000f},
                                          {0.02817123f, -0.07585496f, -0.00969366f},
                                          {-0.15754146f, 0.02306281f, 0.06946538f},
                                          {-0.00593072f, -0.00000124f, -0.00156324f}, 
                                          { 0.03719409f, -0.00002562f, -0.00002217f},
                                          {0.00000000f, 0.00000000f, 0.07495233f},
                                          {-0.03295545f, 0.00021567f, 0.02639184f}};

extern const float LINKS_MASS[NB_LINKS] = {6.20696344f, 1.86377694f, 3.45202011f, 6.40922346f, 1.17868723f, 0.09770847f,0.37200000f};

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
    {0.00004184f, -0.00000023f, 0.00000000f,
	-0.00000023f,  0.00003916f, 0.00000000f,
	 0.00000000f,  0.00000000f, 0.00000699f} // LINK_YAW W.R.T COM_YAW
    ,
    { 0.00049690f, -0.00001989f,  0.00016983f,
     -0.00001989f,  0.00335913f, -0.00000136f,
      0.00016983f, -0.00000136f,  0.00351089f} // LINK_PEDAL W.R.T IN COM_FS
};

#endif
extern const float d7 = 0.023f; 
extern const float r8 = 0.209f;


extern float const VISCOUS_K[NB_AXIS] = {3.0f, 3.0f, 46.0734f * RAD_TO_DEG,
                                  62.3174f * RAD_TO_DEG, 0.0f * RAD_TO_DEG};

// float const EMPIRICAL_INERTIA_K[NB_AXIS] = {13.6178f,13.7704f, 0.2521f * RAD_TO_DEG, 
//                                   0.1831f * RAD_TO_DEG, 0.1867f * RAD_TO_DEG};


Eigen::Vector3f Platform::positionFrame(frame_chain frame)
{
    
  Eigen::Vector3f positionFrame_;
  positionFrame_.setConstant(0.0f);

  float c_theta = cos(_position(PITCH) );
  float c_phi = cos(_position(ROLL) );
  float c_psi = cos(_position(YAW));
  float s_theta = sin(_position(PITCH) );
  float s_phi = sin(_position(ROLL) );
  float s_psi = sin(_position(YAW));

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
            positionFrame_(0) = _position(X) + d6 * s_phi;
            positionFrame_(1) = _position(Y) - d6 * c_phi * s_theta;
            positionFrame_(2) =  r3 + d6 * c_phi * c_theta;
            break;   
        }
        
        case FRAME_PEDAL:
        {
            positionFrame_(0) = _position(X) + (d6 + d7) * s_phi;
            positionFrame_(1) = _position(Y) - (d6 + d7) * c_phi * s_theta;
            positionFrame_(2) = r3 + (d6+d7) * c_phi * c_theta; 
            break;  
        }

        case FRAME_EPOINT:
        {
            positionFrame_(0) = _position(X) + (d6 + d7) * s_phi - r8 * c_phi * s_psi;
            positionFrame_(1) = _position(Y) - (d6 + d7) * c_phi * s_theta + 
                              r8 * ( c_psi*c_theta - s_phi * s_psi * s_theta) ;
            positionFrame_(2) = r3 + (d6+d7) * c_phi * c_theta + 
                              r8 * ( c_psi*s_theta + c_theta*s_phi*s_psi ) ; 
            break;   
        }
    }
    return positionFrame_;
}

Eigen::Matrix3f Platform::rotationMatrix(frame_chain frame) //! orientation of the joint frame wrt to the world frame
{
    
  Eigen::Matrix3f rotationMatrix_;
  rotationMatrix_.setIdentity();

  float c_theta = cos(_position(PITCH) );
  float c_phi = cos(_position(ROLL) );
  float c_psi = cos(_position(YAW));
  float s_theta = sin(_position(PITCH) );
  float s_phi = sin(_position(ROLL) );
  float s_psi = sin(_position(YAW));

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
                            -s_theta , 0.0f , c_theta,
                             c_theta , 0.0f , s_theta; 
            break;   
        }


        case FRAME_YAW:
        {

            
            rotationMatrix_ << -c_phi         ,  0.0f    ,          s_phi,
                             -s_phi*s_theta , -c_theta , -c_phi*s_theta,
                              c_theta*s_phi , -s_theta ,  c_phi*c_theta; 
            break;   
        }

        case FRAME_FS:
        {
            rotationMatrix_(0,0) = c_phi*s_psi;
            rotationMatrix_(1,0) = s_phi*s_psi*s_theta -  c_psi*c_theta;
            rotationMatrix_(2, 0) = - (c_psi * s_theta + c_theta * s_phi * s_psi);
            rotationMatrix_(0, 1) = c_phi*c_psi;
            rotationMatrix_(1, 1) = c_theta*s_psi + c_psi*s_phi*s_theta;
            rotationMatrix_(2, 1) = s_psi*s_theta - c_psi*c_theta*s_phi;
            rotationMatrix_(0, 2) = s_phi;
            rotationMatrix_(1, 2) = -c_phi*s_theta;
            rotationMatrix_(2, 2) = c_phi*c_theta;

        //    rotationMatrix_ <<  c_phi*s_psi                         ,  c_phi*c_psi                         ,  s_phi,
        //                     s_phi*s_psi*s_theta -  c_psi*c_theta , c_theta*s_psi + c_psi*s_phi*s_theta , -c_phi*s_theta,
        //                     -(c_psi*s_theta + c_theta*s_phi*s_psi) , s_psi*s_theta - c_psi*c_theta*s_phi  ,  c_phi*c_theta; 
            break;   
        }
        
        case FRAME_PEDAL ... FRAME_EPOINT:
        {
            rotationMatrix_(0,0) = -c_phi*s_psi;
            rotationMatrix_(1,0) = -(s_phi*s_psi*s_theta -  c_psi*c_theta);
            rotationMatrix_(2,0) = c_psi * s_theta + c_theta * s_phi * s_psi;
            rotationMatrix_(0,1) = -c_phi*c_psi;
            rotationMatrix_(1,1) = -( c_theta*s_psi + c_psi*s_phi*s_theta);
            rotationMatrix_(2,1) = -( s_psi*s_theta - c_psi*c_theta*s_phi);
            rotationMatrix_(0,2) = s_phi;
            rotationMatrix_(1,2) = -c_phi*s_theta;
            rotationMatrix_(2,2) = c_phi*c_theta;
            //    rotationMatrix_ <<  -c_phi*s_psi                            , -c_phi*c_psi                            ,  s_phi,
            //  -(s_phi*s_psi*s_theta -  c_psi*c_theta) , -( c_theta*s_psi + c_psi*s_phi*s_theta) , -c_phi*s_theta,
            //    c_psi*s_theta + c_theta*s_phi*s_psi    , -( s_psi*s_theta - c_psi*c_theta*s_phi) ,  c_phi*c_theta;
                break;
        }

    }

    return rotationMatrix_;
}


Eigen::Matrix<float,6,NB_AXIS> Platform::geometricJacobian(frame_chain frame)
{

    Eigen::Matrix<float, 6, NB_AXIS> J;
    J.setConstant(0.0f);

    float c_theta = cos(_position(PITCH) );
    float c_phi = cos(_position(ROLL) );
    float c_psi = cos(_position(YAW));
    float s_theta = sin(_position(PITCH) );
    float s_phi = sin(_position(ROLL) );
    float s_psi = sin(_position(YAW));

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
            J(4, 3) = c_theta;
            J(5, 3) = s_theta;
            // J<< 0.0f, 1.0f, 0.0f,    0.0f, 0.0f,
            //     1.0f, 0.0f, 0.0f,    0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f,    0.0f, 0.0f,
            //     0.0f, 0.0f, 1.0f,    0.0f, 0.0f,
            //     0.0f, 0.0f, 0.0f, c_theta, 0.0f,
            //     0.0f, 0.0f, 0.0f, s_theta, 0.0f;


            
             break;   
        }

        case FRAME_FS:
        {
            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(1, 2) = -d6 * c_phi * c_theta;
            J(2, 2) = -d6 * c_phi * s_theta;
            J(3, 2) = 1.0f;
            J(0, 3) = d6 * c_phi;
            J(1, 3) = d6 * s_phi * s_theta;
            J(2, 3) = -d6 * c_theta * s_phi;
            J(4, 3) = c_theta;
            J(5, 3) = s_theta;
            J(3, 4) = s_phi;
            J(4, 4) = -c_phi * s_theta;
            J(5, 4) = c_phi * c_theta;

            // J<< 0.0f, 1.0f,              0.0f, d6*c_phi*c_theta*c_theta + d6*c_phi*s_theta*s_theta,           0.0f,
            //     1.0f, 0.0f, -d6*c_phi*c_theta,                                    d6*s_phi*s_theta,           0.0f,
            //     0.0f, 0.0f, -d6*c_phi*s_theta,                                   -d6*c_theta*s_phi,           0.0f,
            //     0.0f, 0.0f,              1.0f,                                                0.0f,          s_phi,
            //     0.0f, 0.0f,              0.0f,                                             c_theta, -c_phi*s_theta,
            //     0.0f, 0.0f,              0.0f,                                             s_theta,  c_phi*c_theta;
            break;   
        }
        
        case FRAME_PEDAL:
        {

            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(1, 2) = -(d6+d7) * c_phi * c_theta;
            J(2, 2) = -(d6+d7) * c_phi * s_theta;
            J(3, 2) = 1.0f;
            J(0, 3) = (d6+d7) * c_phi ;
            J(1, 3) = (d6+d7) * s_phi * s_theta;
            J(2, 3) = -(d6+d7) * c_theta * s_phi;
            J(4, 3) = c_theta;
            J(5, 3) = s_theta;
            J(0, 4) = 0.0f;
            J(1, 4) = 0.0f;
            J(2, 4) = 0.0f;
            J(3, 4) = s_phi;
            J(4, 4) = -c_phi * s_theta;
            J(5, 4) = c_phi * c_theta;
            // J<< 0.0f, 1.0f,                                  0.0f, c_theta*(d6*c_phi*c_theta + d7*c_phi*c_theta) + s_theta*(d6*c_phi*s_theta + d7*c_phi*s_theta), c_phi*c_theta*(d6*c_phi*s_theta + d7*c_phi*s_theta) - c_phi*s_theta*(d6*c_phi*c_theta + d7*c_phi*c_theta),
            //     1.0f, 0.0f, - d6*c_phi*c_theta - d7*c_phi*c_theta,                                                                 s_theta*(d6*s_phi + d7*s_phi),                         c_phi*c_theta*(d6*s_phi + d7*s_phi) - s_phi*(d6*c_phi*c_theta + d7*c_phi*c_theta),
            //     0.0f, 0.0f, - d6*c_phi*s_theta - d7*c_phi*s_theta,                                                                -c_theta*(d6*s_phi + d7*s_phi),                         c_phi*s_theta*(d6*s_phi + d7*s_phi) - s_phi*(d6*c_phi*s_theta + d7*c_phi*s_theta),
            //     0.0f, 0.0f,                                  1.0f,                                                                                          0.0f,                                                                                                     s_phi,
            //     0.0f, 0.0f,                                  0.0f,                                                                                       c_theta,                                                                                            -c_phi*s_theta,
            //     0.0f, 0.0f,                                  0.0f,                                                                                       s_theta,                                                                                             c_phi*c_theta;
            break; 
        }
         case FRAME_EPOINT:
        {

            J(1, 0) = 1.0f;
            J(0, 1) = 1.0f;
            J(1, 2) = -(d6 + d7) * c_phi * c_theta - r8 * (c_psi * s_theta + c_theta * s_phi * s_psi);
            J(2, 2) = -(d6 + d7) * c_phi * s_theta - r8 * (c_psi * c_theta + s_theta * s_phi * s_psi);
            J(3, 2) = 1.0f;
            J(0, 3) = c_theta * (d6 * c_phi * c_theta + d7 * c_phi * c_theta + r8 * c_psi * s_theta +
                      r8 * c_theta * s_phi * s_psi) + s_theta * (d6 * c_phi * s_theta - 
                      r8 * c_psi * c_theta + d7 * c_phi * s_theta + r8 * s_phi * s_psi * s_theta);
            J(1, 3) = s_theta * (d6 * s_phi + d7 * s_phi - r8 * c_phi * s_psi);
            J(2, 3) = -c_theta * (d6 * s_phi + d7 * s_phi - r8 * c_phi * s_psi);
            J(4, 3) = c_theta;
            J(5, 3) = s_theta;
            J(0, 4) = c_phi * c_theta * (d6 * c_phi * s_theta - 
                      r8 * c_psi * c_theta + d7 * c_phi * s_theta + 
                      r8 * s_phi * s_psi * s_theta) - c_phi * s_theta * (d6 * c_phi * c_theta + 
                      d7 * c_phi * c_theta + r8 * c_psi * s_theta + 
                      r8 * c_theta * s_phi * s_psi);
            J(1, 4) = c_phi*c_theta*(d6*s_phi + d7*s_phi - 
                      r8*c_phi*s_psi) - s_phi*(d6*c_phi*c_theta + 
                      d7*c_phi*c_theta + r8*c_psi*s_theta + 
                      r8*c_theta*s_phi*s_psi);
            J(2, 4) = c_phi * s_theta * (d6 * s_phi + d7 * s_phi - r8 * c_phi * s_psi) - 
                    s_phi * (d6 * c_phi * s_theta - 
                    r8 * c_psi * c_theta + d7 * c_phi * s_theta + 
                    r8 * s_phi * s_psi * s_theta);
            J(3, 4) = s_phi;
            J(4, 4) = -c_phi * s_theta;
            J(5, 4) = c_phi * c_theta;

            // J<< 0.0f, 1.0f,                                                                              0.0f, c_theta*(d6*c_phi*c_theta + d7*c_phi*c_theta + r8*c_psi*s_theta + r8*c_theta*s_phi*s_psi) + s_theta*(d6*c_phi*s_theta - r8*c_psi*c_theta + d7*c_phi*s_theta + r8*s_phi*s_psi*s_theta) , c_phi*c_theta*(d6*c_phi*s_theta - r8*c_psi*c_theta + d7*c_phi*s_theta + r8*s_phi*s_psi*s_theta) - c_phi*s_theta*(d6*c_phi*c_theta + d7*c_phi*c_theta + r8*c_psi*s_theta + r8*c_theta*s_phi*s_psi),
            //     1.0f, 0.0f, - d6*c_phi*c_theta - d7*c_phi*c_theta - r8*c_psi*s_theta - r8*c_theta*s_phi*s_psi,                                                                                                                                        s_theta*(d6*s_phi + d7*s_phi - r8*c_phi*s_psi) ,                                                    c_phi*c_theta*(d6*s_phi + d7*s_phi - r8*c_phi*s_psi) - s_phi*(d6*c_phi*c_theta + d7*c_phi*c_theta + r8*c_psi*s_theta + r8*c_theta*s_phi*s_psi),
            //     0.0f, 0.0f,   r8*c_psi*c_theta - d6*c_phi*s_theta - d7*c_phi*s_theta - r8*s_phi*s_psi*s_theta,                                                                                                                                       -c_theta*(d6*s_phi + d7*s_phi - r8*c_phi*s_psi) ,                                                    c_phi*s_theta*(d6*s_phi + d7*s_phi - r8*c_phi*s_psi) - s_phi*(d6*c_phi*s_theta - r8*c_psi*c_theta + d7*c_phi*s_theta + r8*s_phi*s_psi*s_theta),
            //     0.0f, 0.0f,                                                                              1.0f,                                                                                                                                                                                  0.0f ,                                                                                                                                                                                             s_phi,
            //     0.0f, 0.0f,                                                                              0.0f,                                                                                                                                                                               c_theta ,                                                                                                                                                                                    -c_phi*s_theta,
            //     0.0f, 0.0f,                                                                              0.0f,                                                                                                                                                                               s_theta ,                                                                                                                                                                                     c_phi*c_theta;
 

                    break; 
        }   
    }
  return J;
}

Eigen::Vector3f Platform::comLinkWRTBase(link_chain link) {

  Eigen::Vector3f positionCOMLink_;
  positionCOMLink_.setConstant(0.0f);

  float c_theta = cos(_position(PITCH));
  float c_phi = cos(_position(ROLL));
  float c_psi = cos(_position(YAW));
  float s_theta = sin(_position(PITCH));
  float s_phi = sin(_position(ROLL));
  float s_psi = sin(_position(YAW));

  switch (link) {
    case LINK_BASE: {

        positionCOMLink_(0) = LINKS_COM[LINK_BASE][0];
        positionCOMLink_(1) = LINKS_COM[LINK_BASE][1];
        positionCOMLink_(2) = LINKS_COM[LINK_BASE][2];
        break;
    }

    case LINK_Y:

    {
        positionCOMLink_(0) = LINKS_COM[LINK_Y][0];
        positionCOMLink_(1) = _position(Y) + LINKS_COM[LINK_Y][2];
        positionCOMLink_(2) = -LINKS_COM[LINK_Y][0];
        break;
    }

    case LINK_X:

    {
        positionCOMLink_(0) = _position(X) + LINKS_COM[LINK_X][2];
        positionCOMLink_(1) = _position(Y) + LINKS_COM[LINK_X][1];
        positionCOMLink_(2) = -LINKS_COM[LINK_X][0];
        break;
    }

    case LINK_PITCH:

    {
        positionCOMLink_(0) = LINKS_COM[LINK_PITCH][2] + _position(X);
        positionCOMLink_(1) = _position(Y) - LINKS_COM[LINK_PITCH][1] * c_theta -
                            LINKS_COM[LINK_PITCH][0] * s_theta;
        positionCOMLink_(2) = LINKS_COM[LINK_PITCH][0] * c_theta -
                            LINKS_COM[LINK_PITCH][1] * s_theta + r3;
        break;
    }

    case LINK_ROLL:

    {
        positionCOMLink_(0) = _position(X) + LINKS_COM[LINK_ROLL][1] * c_phi +
                            LINKS_COM[LINK_ROLL][0] * s_phi;
        positionCOMLink_(1) = _position(Y) -
                            s_theta * (LINKS_COM[LINK_ROLL][0] * c_phi -
                                        LINKS_COM[LINK_ROLL][1] * s_phi) +
                            LINKS_COM[LINK_ROLL][2] * c_theta;
        positionCOMLink_(2) = c_theta * (LINKS_COM[LINK_ROLL][0] * c_phi -
                                        LINKS_COM[LINK_ROLL][1] * s_phi) +
                            LINKS_COM[LINK_ROLL][2] * s_theta + r3;
        break;
    }

    case LINK_YAW:

    {
        positionCOMLink_(0) = _position(X) -
                            c_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                    LINKS_COM[LINK_YAW][1] * s_psi) +
                            LINKS_COM[LINK_YAW][2] * s_phi;
        positionCOMLink_(1) =_position(Y) -
                            c_theta *
                                (LINKS_COM[LINK_YAW][1] * c_psi + LINKS_COM[LINK_YAW][0] * s_psi) -
                            s_phi * s_theta *
                                (LINKS_COM[LINK_YAW][0] * c_psi - LINKS_COM[LINK_YAW][1] * s_psi) -
                            LINKS_COM[LINK_YAW][2] * c_phi * s_theta;
        positionCOMLink_(2) =
                            c_theta * s_phi *
                                (LINKS_COM[LINK_YAW][0] * c_psi - LINKS_COM[LINK_YAW][1] * s_psi) -
                            s_theta *
                                (LINKS_COM[LINK_YAW][1] * c_psi + LINKS_COM[LINK_YAW][0] * s_psi) +
                            LINKS_COM[LINK_YAW][2] * c_phi * c_theta + r3;
        break;
    }

    case LINK_PEDAL:

    {
        positionCOMLink_(0) = _position(X) + d6 * s_phi +
                            LINKS_COM[LINK_PEDAL][2] * s_phi +
                            LINKS_COM[LINK_PEDAL][1] * c_phi * c_psi +
                            LINKS_COM[LINK_PEDAL][0] * c_phi * s_psi;
        positionCOMLink_(1) =
            _position(Y) - d6 * c_phi * s_theta +
            LINKS_COM[LINK_PEDAL][1] * (c_theta * s_psi + c_psi * s_phi * s_theta) -
            LINKS_COM[LINK_PEDAL][0] * (c_psi * c_theta - s_phi * s_psi * s_theta) -
            LINKS_COM[LINK_PEDAL][2] * c_phi * s_theta;
        positionCOMLink_(2) =
            d6 * c_phi * c_theta -
            LINKS_COM[LINK_PEDAL][0] * (c_psi * s_theta + c_theta * s_phi * s_psi) +
            LINKS_COM[LINK_PEDAL][1] * (s_psi * s_theta - c_psi * c_theta * s_phi) +
            LINKS_COM[LINK_PEDAL][2] * c_phi * c_theta + r3;

        break;
    }
  }
  return positionCOMLink_;
}

Eigen::Matrix3f Platform::comRotationMatrix(link_chain link){

  Eigen::Matrix3f comRotationMatrix_;
  comRotationMatrix_.setIdentity();

  float c_theta = cos(_position(PITCH) );
  float c_phi = cos(_position(ROLL) );
  float c_psi = cos(_position(YAW));
  float s_theta = sin(_position(PITCH) );
  float s_phi = sin(_position(ROLL) );
  float s_psi = sin(_position(YAW));

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
                                   -s_theta,    -c_theta, 0.0f,
                                    c_theta,    -s_theta, 0.0f;  
            break;  
        }

        case LINK_ROLL:
        {
            comRotationMatrix_ <<             s_phi,          c_phi,    0.0f,
                                     -c_phi*s_theta,  s_phi*s_theta, c_theta,
                                      c_phi*c_theta, -c_theta*s_phi, s_theta;
            break;
        }


        case LINK_YAW:
        {

            
            comRotationMatrix_ <<                         -c_phi*c_psi,                           c_phi*s_psi,          s_phi,
                                 - c_theta*s_psi - c_psi*s_phi*s_theta,   s_phi*s_psi*s_theta - c_psi*c_theta, -c_phi*s_theta,
                                   c_psi*c_theta*s_phi - s_psi*s_theta, - c_psi*s_theta - c_theta*s_phi*s_psi,  c_phi*c_theta;
 
            break;   
        }

        case LINK_PEDAL: // THis link is measured w.r.t to FRAME_FS
        {

        comRotationMatrix_ <<                          c_phi*s_psi,                         c_phi*c_psi,          s_phi,
                               s_phi*s_psi*s_theta - c_psi*c_theta, c_theta*s_psi + c_psi*s_phi*s_theta, -c_phi*s_theta,
                             - c_psi*s_theta - c_theta*s_phi*s_psi, s_psi*s_theta - c_psi*c_theta*s_phi,  c_phi*c_theta;
            break;   
        }

    }

    return comRotationMatrix_;
}

Eigen::Matrix<float, 6, NB_AXIS> Platform::comGeometricJacobian(link_chain link){

    Eigen::Matrix<float, 6, NB_AXIS> comJ;
    comJ.setConstant(0.0f);

    float c_theta = cos(_position(PITCH) );
    float c_phi = cos(_position(ROLL) );
    float c_psi = cos(_position(YAW));
    float s_theta = sin(_position(PITCH) );
    float s_phi = sin(_position(ROLL) );
    float s_psi = sin(_position(YAW));

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
                 1.0f, 0.0f,   LINKS_COM[LINK_PITCH][1]*s_theta - LINKS_COM[LINK_PITCH][0]*c_theta, 0.0f, 0.0f,
                 0.0f, 0.0f, - LINKS_COM[LINK_PITCH][1]*c_theta - LINKS_COM[LINK_PITCH][0]*s_theta, 0.0f, 0.0f,
                 0.0f, 0.0f,                                                                  1.0f, 0.0f, 0.0f,
                 0.0f, 0.0f,                                                                  0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f,                                                                  0.0f, 0.0f, 0.0f;
;
            break;  
        }

        case LINK_ROLL:
        {
            comJ(1, 0) = 1.0f;
            comJ(0, 1) = 1.0f;
            comJ(1, 2) = -LINKS_COM[LINK_ROLL][2] * s_theta -
                         c_theta * (LINKS_COM[LINK_ROLL][0] * c_phi -
                         LINKS_COM[LINK_ROLL][1] * s_phi);
            
            comJ(2, 2) = LINKS_COM[LINK_ROLL][2] * c_theta -
                         s_theta * (LINKS_COM[LINK_ROLL][0] * c_phi -
                                    LINKS_COM[LINK_ROLL][1] * s_phi);
            comJ(3, 2) = 1.0f;

            comJ(0, 3) =
                        c_theta * (LINKS_COM[LINK_ROLL][2] * s_theta +
                        c_theta * (LINKS_COM[LINK_ROLL][0] * c_phi -
                        LINKS_COM[LINK_ROLL][1] * s_phi)) +
                        s_theta * (s_theta * (LINKS_COM[LINK_ROLL][0] * c_phi -
                        LINKS_COM[LINK_ROLL][1] * s_phi) -
                        LINKS_COM[LINK_ROLL][2] * c_theta);
            comJ(1, 3) = s_theta * (LINKS_COM[LINK_ROLL][1] * c_phi +
                                    LINKS_COM[LINK_ROLL][0] * s_phi);
            comJ(2, 3) = -c_theta * (LINKS_COM[LINK_ROLL][1] * c_phi +
                                     LINKS_COM[LINK_ROLL][0] * s_phi);
            comJ(4, 3) = c_theta;
            comJ(5, 3) = s_theta;
            // comJ<<  0.0f, 1.0f,                                                                                                        0.0f, c_theta*(LINKS_COM[LINK_ROLL][2]*s_theta + c_theta*(LINKS_COM[LINK_ROLL][0]*c_phi - LINKS_COM[LINK_ROLL][1]*s_phi)) + s_theta*(s_theta*(LINKS_COM[LINK_ROLL][0]*c_phi - LINKS_COM[LINK_ROLL][1]*s_phi) - LINKS_COM[LINK_ROLL][2]*c_theta), 0.0f,
            //         1.0f, 0.0f, - LINKS_COM[LINK_ROLL][2]*s_theta - c_theta*(LINKS_COM[LINK_ROLL][0]*c_phi - LINKS_COM[LINK_ROLL][1]*s_phi),                                                                                                                                                                   s_theta*(LINKS_COM[LINK_ROLL][1]*c_phi + LINKS_COM[LINK_ROLL][0]*s_phi), 0.0f,
            //         0.0f, 0.0f,   LINKS_COM[LINK_ROLL][2]*c_theta - s_theta*(LINKS_COM[LINK_ROLL][0]*c_phi - LINKS_COM[LINK_ROLL][1]*s_phi),                                                                                                                                                                  -c_theta*(LINKS_COM[LINK_ROLL][1]*c_phi + LINKS_COM[LINK_ROLL][0]*s_phi), 0.0f,
            //         0.0f, 0.0f,                                                                                                        1.0f,                                                                                                                                                                                                                                      0.0f, 0.0f,
            //         0.0f, 0.0f,                                                                                                        0.0f,                                                                                                                                                                                                                                   c_theta, 0.0f,
            //         0.0f, 0.0f,                                                                                                        0.0f,                                                                                                                                                                                                                                   s_theta, 0.0f;
            break;   
        }


        case LINK_YAW:
        {
            comJ(1, 0) = 1.0f;
            comJ(0, 1) = 1.0f;
            comJ(1, 2) = s_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                    LINKS_COM[LINK_YAW][0] * s_psi) -
                         LINKS_COM[LINK_YAW][2] * c_phi * c_theta -
                         c_theta * s_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                            LINKS_COM[LINK_YAW][1] * s_psi);
            comJ(2, 2) = -c_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                     LINKS_COM[LINK_YAW][0] * s_psi) -
                         LINKS_COM[LINK_YAW][2] * c_phi * s_theta -
                         s_phi * s_theta * (LINKS_COM[LINK_YAW][0] * c_psi -
                                            LINKS_COM[LINK_YAW][1] * s_psi);
            comJ(3,2) = 1.0f;
            comJ(0,3) =
                c_theta * (LINKS_COM[LINK_YAW][2] * c_phi * c_theta -
                           s_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                      LINKS_COM[LINK_YAW][0] * s_psi) +
                           c_theta * s_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                              LINKS_COM[LINK_YAW][1] * s_psi)) +
                s_theta * (c_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                      LINKS_COM[LINK_YAW][0] * s_psi) +
                           LINKS_COM[LINK_YAW][2] * c_phi * s_theta +
                           s_phi * s_theta * (LINKS_COM[LINK_YAW][0] * c_psi -
                                              LINKS_COM[LINK_YAW][1] * s_psi));

            comJ(1,3) = s_theta * (LINKS_COM[LINK_YAW][2] * s_phi -
                                    c_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                             LINKS_COM[LINK_YAW][1] * s_psi));
            comJ(2, 3) = -c_theta * (LINKS_COM[LINK_YAW][2] * s_phi -
                                     c_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                              LINKS_COM[LINK_YAW][1] * s_psi));
            comJ(4, 3) = c_theta;
            comJ(5, 3) = s_theta;
            comJ(0, 4) =
                c_phi * c_theta *
                    (c_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                LINKS_COM[LINK_YAW][0] * s_psi) +
                     LINKS_COM[LINK_YAW][2] * c_phi * s_theta +
                     s_phi * s_theta * (LINKS_COM[LINK_YAW][0] * c_psi -
                                        LINKS_COM[LINK_YAW][1] * s_psi)) -
                c_phi * s_theta *
                    (LINKS_COM[LINK_YAW][2] * c_phi * c_theta -
                     s_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                LINKS_COM[LINK_YAW][0] * s_psi) +
                     c_theta * s_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                        LINKS_COM[LINK_YAW][1] * s_psi));
            comJ(1, 4) =
                c_phi * c_theta * (LINKS_COM[LINK_YAW][2] * s_phi -
                                   c_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                            LINKS_COM[LINK_YAW][1] * s_psi)) -
                s_phi * (LINKS_COM[LINK_YAW][2] * c_phi * c_theta -
                         s_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                    LINKS_COM[LINK_YAW][0] * s_psi) +
                         c_theta * s_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                            LINKS_COM[LINK_YAW][1] * s_psi));
            comJ(2, 4) =
                c_phi * s_theta * (LINKS_COM[LINK_YAW][2] * s_phi -
                                   c_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                            LINKS_COM[LINK_YAW][1] * s_psi)) -
                s_phi * (c_theta * (LINKS_COM[LINK_YAW][1] * c_psi +
                                    LINKS_COM[LINK_YAW][0] * s_psi) +
                         LINKS_COM[LINK_YAW][2] * c_phi * s_theta +
                         s_phi * s_theta * (LINKS_COM[LINK_YAW][0] * c_psi -
                                            LINKS_COM[LINK_YAW][1] * s_psi));

            comJ(3,4) = s_phi;
            comJ(4,4) = -c_phi*s_theta;
            comJ(5,4) = c_phi*c_theta;;                                            
            // comJ<< 0.0f, 1.0f,                                                                                                                                                                                         0.0f, c_theta*(LINKS_COM[LINK_YAW][2]*c_phi*c_theta - s_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) + c_theta*s_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)) + s_theta*(c_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) + LINKS_COM[LINK_YAW][2]*c_phi*s_theta + s_phi*s_theta*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)), c_phi*c_theta*(c_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) + LINKS_COM[LINK_YAW][2]*c_phi*s_theta + s_phi*s_theta*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)) - c_phi*s_theta*(LINKS_COM[LINK_YAW][2]*c_phi*c_theta - s_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) + c_theta*s_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)),
            //        1.0f, 0.0f,   s_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) - LINKS_COM[LINK_YAW][2]*c_phi*c_theta - c_theta*s_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi),                                                                                                                                                                                                                                                                                                s_theta*(LINKS_COM[LINK_YAW][2]*s_phi - c_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)),                                                                                                 c_phi*c_theta*(LINKS_COM[LINK_YAW][2]*s_phi - c_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)) - s_phi*(LINKS_COM[LINK_YAW][2]*c_phi*c_theta - s_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) + c_theta*s_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)),
            //        0.0f, 0.0f, - c_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) - LINKS_COM[LINK_YAW][2]*c_phi*s_theta - s_phi*s_theta*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi),                                                                                                                                                                                                                                                                                               -c_theta*(LINKS_COM[LINK_YAW][2]*s_phi - c_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)),                                                                                                 c_phi*s_theta*(LINKS_COM[LINK_YAW][2]*s_phi - c_phi*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)) - s_phi*(c_theta*(LINKS_COM[LINK_YAW][1]*c_psi + LINKS_COM[LINK_YAW][0]*s_psi) + LINKS_COM[LINK_YAW][2]*c_phi*s_theta + s_phi*s_theta*(LINKS_COM[LINK_YAW][0]*c_psi - LINKS_COM[LINK_YAW][1]*s_psi)),
            //        0.0f, 0.0f,                                                                                                                                                                                         1.0f,                                                                                                                                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                   s_phi,
            //        0.0f, 0.0f,                                                                                                                                                                                         0.0f,                                                                                                                                                                                                                                                                                                                                                                                                     c_theta,                                                                                                                                                                                                                                                                                                                                                                                                          -c_phi*s_theta,
            //        0.0f, 0.0f,                                                                                                                                                                                         0.0f,                                                                                                                                                                                                                                                                                                                                                                                                     s_theta,                                                                                                                                                                                                                                                                                                                                                                                                           c_phi*c_theta;
;

            
             break;   
        }

        case LINK_PEDAL:
        {
            comJ(1, 0) = 1.0f;
            comJ(0, 1) = 1.0f;
            comJ(1, 2) = LINKS_COM[LINK_PEDAL][0] *
                             (c_psi * s_theta + c_theta * s_phi * s_psi) -
                         LINKS_COM[LINK_PEDAL][1] *
                             (s_psi * s_theta - c_psi * c_theta * s_phi) -
                         d6 * c_phi * c_theta -
                         LINKS_COM[LINK_PEDAL][2] * c_phi * c_theta;
            comJ(2, 2) = LINKS_COM[LINK_PEDAL][1] *
                             (c_theta * s_psi + c_psi * s_phi * s_theta) -
                         LINKS_COM[LINK_PEDAL][0] *
                             (c_psi * c_theta - s_phi * s_psi * s_theta) -
                         d6 * c_phi * s_theta -
                         LINKS_COM[LINK_PEDAL][2] * c_phi * s_theta;
            comJ(3,2) = 1.0f;
            comJ(1, 3) =
                c_theta * (LINKS_COM[LINK_PEDAL][1] *
                               (s_psi * s_theta - c_psi * c_theta * s_phi) -
                           LINKS_COM[LINK_PEDAL][0] *
                               (c_psi * s_theta + c_theta * s_phi * s_psi) +
                           d6 * c_phi * c_theta +
                           LINKS_COM[LINK_PEDAL][2] * c_phi * c_theta) +
                s_theta * (LINKS_COM[LINK_PEDAL][0] *
                               (c_psi * c_theta - s_phi * s_psi * s_theta) -
                           LINKS_COM[LINK_PEDAL][1] *
                               (c_theta * s_psi + c_psi * s_phi * s_theta) +
                           d6 * c_phi * s_theta +
                           LINKS_COM[LINK_PEDAL][2] * c_phi * s_theta);
            comJ(2, 3) =
                -c_theta * (d6 * s_phi + LINKS_COM[LINK_PEDAL][2] * s_phi +
                            LINKS_COM[LINK_PEDAL][1] * c_phi * c_psi +
                            LINKS_COM[LINK_PEDAL][0] * c_phi * s_psi);
            comJ(4, 3) = c_theta;
            comJ(5, 3) = s_theta;
            comJ(0, 4) = c_phi * c_theta *
                             (LINKS_COM[LINK_PEDAL][0] *
                                  (c_psi * c_theta - s_phi * s_psi * s_theta) -
                              LINKS_COM[LINK_PEDAL][1] *
                                  (c_theta * s_psi + c_psi * s_phi * s_theta) +
                              d6 * c_phi * s_theta +
                              LINKS_COM[LINK_PEDAL][2] * c_phi * s_theta) -
                         c_phi * s_theta *
                             (LINKS_COM[LINK_PEDAL][1] *
                                  (s_psi * s_theta - c_psi * c_theta * s_phi) -
                              LINKS_COM[LINK_PEDAL][0] *
                                  (c_psi * s_theta + c_theta * s_phi * s_psi) +
                              d6 * c_phi * c_theta +
                              LINKS_COM[LINK_PEDAL][2] * c_phi * c_theta);
            comJ(1,4) =
                      c_phi * c_theta *
                          (d6 * s_phi + LINKS_COM[LINK_PEDAL][2] * s_phi +
                           LINKS_COM[LINK_PEDAL][1] * c_phi * c_psi +
                           LINKS_COM[LINK_PEDAL][0] * c_phi * s_psi) -
                      s_phi * (LINKS_COM[LINK_PEDAL][1] *
                                   (s_psi * s_theta - c_psi * c_theta * s_phi) -
                               LINKS_COM[LINK_PEDAL][0] *
                                   (c_psi * s_theta + c_theta * s_phi * s_psi) +
                               d6 * c_phi * c_theta +
                               LINKS_COM[LINK_PEDAL][2] * c_phi * c_theta);
            comJ(2,4) = 
                     c_phi * s_theta *
                         (d6 * s_phi + LINKS_COM[LINK_PEDAL][2] * s_phi +
                          LINKS_COM[LINK_PEDAL][1] * c_phi * c_psi +
                          LINKS_COM[LINK_PEDAL][0] * c_phi * s_psi) -
                     s_phi * (LINKS_COM[LINK_PEDAL][0] *
                                  (c_psi * c_theta - s_phi * s_psi * s_theta) -
                              LINKS_COM[LINK_PEDAL][1] *
                                  (c_theta * s_psi + c_psi * s_phi * s_theta) +
                              d6 * c_phi * s_theta +
                              LINKS_COM[LINK_PEDAL][2] * c_phi * s_theta);
            comJ(3,4) =          s_phi;
            comJ(4,4) = -c_phi*s_theta;
            comJ(5,4) =  c_phi*c_theta;                  
            //comJ<< 0.0f, 1.0f,                                                                                                                                                                                        0.0f, c_theta*(LINKS_COM[LINK_PEDAL][1]*(s_psi*s_theta - c_psi*c_theta*s_phi) - LINKS_COM[LINK_PEDAL][0]*(c_psi*s_theta + c_theta*s_phi*s_psi) + d6*c_phi*c_theta + LINKS_COM[LINK_PEDAL][2]*c_phi*c_theta) + s_theta*(LINKS_COM[LINK_PEDAL][0]*(c_psi*c_theta - s_phi*s_psi*s_theta) - LINKS_COM[LINK_PEDAL][1]*(c_theta*s_psi + c_psi*s_phi*s_theta) + d6*c_phi*s_theta + LINKS_COM[LINK_PEDAL][2]*c_phi*s_theta), c_phi*c_theta*(LINKS_COM[LINK_PEDAL][0]*(c_psi*c_theta - s_phi*s_psi*s_theta) - LINKS_COM[LINK_PEDAL][1]*(c_theta*s_psi + c_psi*s_phi*s_theta) + d6*c_phi*s_theta + LINKS_COM[LINK_PEDAL][2]*c_phi*s_theta) - c_phi*s_theta*(LINKS_COM[LINK_PEDAL][1]*(s_psi*s_theta - c_psi*c_theta*s_phi) - LINKS_COM[LINK_PEDAL][0]*(c_psi*s_theta + c_theta*s_phi*s_psi) + d6*c_phi*c_theta + LINKS_COM[LINK_PEDAL][2]*c_phi*c_theta),
            //       1.0f, 0.0f, LINKS_COM[LINK_PEDAL][0]*(c_psi*s_theta + c_theta*s_phi*s_psi) - LINKS_COM[LINK_PEDAL][1]*(s_psi*s_theta - c_psi*c_theta*s_phi) - d6*c_phi*c_theta - LINKS_COM[LINK_PEDAL][2]*c_phi*c_theta,                                                                                                                                                                                                                                                                             s_theta*(d6*s_phi + LINKS_COM[LINK_PEDAL][2]*s_phi + LINKS_COM[LINK_PEDAL][1]*c_phi*c_psi + LINKS_COM[LINK_PEDAL][0]*c_phi*s_psi),                                                                             c_phi*c_theta*(d6*s_phi + LINKS_COM[LINK_PEDAL][2]*s_phi + LINKS_COM[LINK_PEDAL][1]*c_phi*c_psi + LINKS_COM[LINK_PEDAL][0]*c_phi*s_psi) - s_phi*(LINKS_COM[LINK_PEDAL][1]*(s_psi*s_theta - c_psi*c_theta*s_phi) - LINKS_COM[LINK_PEDAL][0]*(c_psi*s_theta + c_theta*s_phi*s_psi) + d6*c_phi*c_theta + LINKS_COM[LINK_PEDAL][2]*c_phi*c_theta),
            //       0.0f, 0.0f, LINKS_COM[LINK_PEDAL][1]*(c_theta*s_psi + c_psi*s_phi*s_theta) - LINKS_COM[LINK_PEDAL][0]*(c_psi*c_theta - s_phi*s_psi*s_theta) - d6*c_phi*s_theta - LINKS_COM[LINK_PEDAL][2]*c_phi*s_theta,                                                                                                                                                                                                                                                                            -c_theta*(d6*s_phi + LINKS_COM[LINK_PEDAL][2]*s_phi + LINKS_COM[LINK_PEDAL][1]*c_phi*c_psi + LINKS_COM[LINK_PEDAL][0]*c_phi*s_psi),                                                                             c_phi*s_theta*(d6*s_phi + LINKS_COM[LINK_PEDAL][2]*s_phi + LINKS_COM[LINK_PEDAL][1]*c_phi*c_psi + LINKS_COM[LINK_PEDAL][0]*c_phi*s_psi) - s_phi*(LINKS_COM[LINK_PEDAL][0]*(c_psi*c_theta - s_phi*s_psi*s_theta) - LINKS_COM[LINK_PEDAL][1]*(c_theta*s_psi + c_psi*s_phi*s_theta) + d6*c_phi*s_theta + LINKS_COM[LINK_PEDAL][2]*c_phi*s_theta),
            //       0.0f, 0.0f,                                                                                                                                                                                        1.0f,                                                                                                                                                                                                                                                                                                                                                                                                          0.0f,                                                                                                                                                                                                                                                                                                                                                                                                                     s_phi,
            //       0.0f, 0.0f,                                                                                                                                                                                        0.0f,                                                                                                                                                                                                                                                                                                                                                                                                       c_theta,                                                                                                                                                                                                                                                                                                                                                                                                            -c_phi*s_theta,
            //       0.0f, 0.0f,                                                                                                                                                                                        0.0f,                                                                                                                                                                                                                                                                                                                                                                                                       s_theta,                                                                                                                                                                                                                                                                                                                                                                                                             c_phi*c_theta;
 
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
