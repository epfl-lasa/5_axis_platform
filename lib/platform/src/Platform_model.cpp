#include <Platform.h>
#include <LP_Filter.h>
#include <definitions.h>
#include <definitions_2.h>

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

#endif
extern const float d7 = 0.023f; 
extern const float r8 = 0.209f;



Eigen::Vector3f Platform::positionFrame(frame_chain frame)
{
    
  Eigen::Vector3f frame_origin;
  frame_origin.setConstant(0.0f);

  float c_theta = cos(_position[PITCH] * DEG_TO_RAD);
  float c_phi = cos(_position[ROLL] * DEG_TO_RAD);
  float c_psi = cos(_position[YAW] * DEG_TO_RAD);
  float s_theta = sin(_position[PITCH] * DEG_TO_RAD);
  float s_phi = sin(_position[ROLL] * DEG_TO_RAD);
  float s_psi = sin(_position[YAW] * DEG_TO_RAD);

    switch (frame)
    {
        case FRAME_BASE:
        {
            frame_origin.setConstant(0.0f);
            break;   
        }

        case FRAME_Y:
        {
            frame_origin.setConstant(0.0f);
            break;   
        }

        case FRAME_X:
        {
            frame_origin<<0.0f,_position[Y],0.0f; 
            break;  
        }

        case FRAME_Z:
        {
            frame_origin<<_position[X],_position[Y],0.0f; 
            break;  
        }

        case FRAME_PITCH ... FRAME_YAW:
        {
            frame_origin<<_position[X] , _position[Y] , r3;
            break;   
        }

        case FRAME_FS:
        {
            frame_origin(0) = _position[X] + d6 * s_phi;
            frame_origin(1) = _position[Y] - d6 * c_phi * s_theta;
            frame_origin(2) =  r3 + d6 * c_phi * c_theta;
            break;   
        }
        
        case FRAME_PEDAL:
        {
            frame_origin(0) = _position[X] + (d6 + d7) * s_phi;
            frame_origin(1) = _position[Y] - (d6 + d7) * c_phi * s_theta;
            frame_origin(2) = r3 + (d6+d7) * c_phi * c_theta; 
            break;  
        }

        case FRAME_EPOINT:
        {
            frame_origin(0) = _position[X] + (d6 + d7) * s_phi - r8 * c_phi * s_psi;
            frame_origin(1) = _position[Y] - (d6 + d7) * c_phi * s_theta + 
                              r8 * ( c_psi*c_theta - s_phi * s_psi * s_theta) ;
            frame_origin(2) = r3 + (d6+d7) * c_phi * c_theta + 
                              r8 * ( c_psi*s_theta + c_theta*s_phi*s_psi ) ; 
            break;   
        }    
    }

    return frame_origin;
}

Eigen::Matrix3f Platform::rotationMatrix(frame_chain frame)
{
    
  Eigen::Matrix3f frame_vectors;
  frame_vectors.setIdentity();

  float c_theta = cos(_position[PITCH] * DEG_TO_RAD);
  float c_phi = cos(_position[ROLL] * DEG_TO_RAD);
  float c_psi = cos(_position[YAW] * DEG_TO_RAD);
  float s_theta = sin(_position[PITCH] * DEG_TO_RAD);
  float s_phi = sin(_position[ROLL] * DEG_TO_RAD);
  float s_psi = sin(_position[YAW] * DEG_TO_RAD);

    switch (frame)
    {
        case FRAME_BASE:
        {
            frame_vectors.setIdentity();
            break;   
        }

        case FRAME_Y:
        {
            frame_vectors << 1.0f , 0.0f , 0.0f,
                             0.0f , 0.0f , 1.0f,
                             0.0f ,-1.0f , 0.0f; 
            break;   
        }

        case FRAME_X:   
        {
            frame_vectors << 0.0f , 0.0f , 1.0f,
                             0.0f , 1.0f , 0.0f,
                            -1.0f , 0.0f , 0.0f;  
            break;  
        }

        case FRAME_Z ... FRAME_PITCH:
        {
            frame_vectors << 0.0f , 0.0f , 1.0f,
                             0.0f ,-1.0f , 0.0f,
                             1.0f , 0.0f , 0.0f;  
            break;  
        }

        case FRAME_ROLL:
        {
            frame_vectors << 0.0f    , 1.0f ,    0.0f,
                            -s_theta , 0.0f , c_theta,
                             c_theta , 0.0f , s_theta; 
            break;   
        }


        case FRAME_YAW:
        {

            
            frame_vectors << -c_phi         ,  0.0f    ,          s_phi,
                             -s_phi*s_theta , -c_theta , -c_phi*s_theta,
                              c_theta*s_phi , -s_theta ,  c_phi*c_theta; 
            break;   
        }

        case FRAME_FS:
        {
            frame_vectors(0,0) = c_phi*s_psi;
            frame_vectors(1,0) = s_phi*s_psi*s_theta -  c_psi*c_theta;
            frame_vectors(2, 0) = - (c_psi * s_theta + c_theta * s_phi * s_psi);
            frame_vectors(0, 1) = c_phi*c_psi;
            frame_vectors(1, 1) = c_theta*s_psi + c_psi*s_phi*s_theta;
            frame_vectors(2, 1) = s_psi*s_theta - c_psi*c_theta*s_phi;
            frame_vectors(0, 2) = s_phi;
            frame_vectors(1, 2) = -c_phi*s_theta;
            frame_vectors(2, 2) = c_phi*c_theta;

        //    frame_vectors <<  c_phi*s_psi                         ,  c_phi*c_psi                         ,  s_phi,
        //                     s_phi*s_psi*s_theta -  c_psi*c_theta , c_theta*s_psi + c_psi*s_phi*s_theta , -c_phi*s_theta,
        //                     -(c_psi*s_theta + c_theta*s_phi*s_psi) , s_psi*s_theta - c_psi*c_theta*s_phi  ,  c_phi*c_theta; 
            break;   
        }
        
        case FRAME_PEDAL ... FRAME_EPOINT:
        {
            frame_vectors(0,0) = -c_phi*s_psi;
            frame_vectors(1,0) = -(s_phi*s_psi*s_theta -  c_psi*c_theta);
            frame_vectors(2,0) = c_psi * s_theta + c_theta * s_phi * s_psi;
            frame_vectors(0,1) = -c_phi*c_psi;
            frame_vectors(1,1) = -( c_theta*s_psi + c_psi*s_phi*s_theta);
            frame_vectors(2,1) = -( s_psi*s_theta - c_psi*c_theta*s_phi);
            frame_vectors(0,2) = s_phi;
            frame_vectors(1,2) = -c_phi*s_theta;
            frame_vectors(2,2) = c_phi*c_theta;
            //    frame_vectors <<  -c_phi*s_psi                            , -c_phi*c_psi                            ,  s_phi,
            //  -(s_phi*s_psi*s_theta -  c_psi*c_theta) , -( c_theta*s_psi + c_psi*s_phi*s_theta) , -c_phi*s_theta,
            //    c_psi*s_theta + c_theta*s_phi*s_psi    , -( s_psi*s_theta - c_psi*c_theta*s_phi) ,  c_phi*c_theta;
                break;
        }

    }

    return frame_vectors;
}


Eigen::Matrix<float,6,NB_AXIS> Platform::geometricJacobian(frame_chain frame)
{

    Eigen::Matrix<float, 6, NB_AXIS> J;
    J.setConstant(0.0f);

    float c_theta = cos(_position[PITCH] * DEG_TO_RAD);
    float c_phi = cos(_position[ROLL] * DEG_TO_RAD);
    float c_psi = cos(_position[YAW] * DEG_TO_RAD);
    float s_theta = sin(_position[PITCH] * DEG_TO_RAD);
    float s_phi = sin(_position[ROLL] * DEG_TO_RAD);
    float s_psi = sin(_position[YAW] * DEG_TO_RAD);

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

Eigen::Vector3f Platform::comLinkWRTBase(link_chain link)
{
    Eigen::Vector3f link_comWRTBase;
    link_comWRTBase.setConstant(0.0f);
    float c_theta = cos(_position[PITCH] * DEG_TO_RAD);
    float c_phi = cos(_position[ROLL] * DEG_TO_RAD);
    float c_psi = cos(_position[YAW] * DEG_TO_RAD);
    float s_theta = sin(_position[PITCH] * DEG_TO_RAD);
    float s_phi = sin(_position[ROLL] * DEG_TO_RAD);
    float s_psi = sin(_position[YAW] * DEG_TO_RAD);


    switch(link)
    {
        case LINK_BASE:
        {
            link_comWRTBase(0)= LINKS_COM[LINK_BASE][0];
            link_comWRTBase(1) = LINKS_COM[LINK_BASE][1];
            link_comWRTBase(2) = LINKS_COM[LINK_BASE][2];
            break;
        }

        case LINK_Y:

        {
            link_comWRTBase(0) = LINKS_COM[LINK_Y][0];
            link_comWRTBase(1) = _position[Y] + LINKS_COM[LINK_Y][2];
            link_comWRTBase(2) = -LINKS_COM[LINK_Y][0];
            break;
        }

        case LINK_X:

        {
            link_comWRTBase(0) = _position[X] + LINKS_COM[LINK_X][2];
            link_comWRTBase(1) = _position[Y] + LINKS_COM[LINK_X][1];
            link_comWRTBase(2) = -LINKS_COM[LINK_X][0];
            break;
        }

        case LINK_PITCH:

        {
            link_comWRTBase(0) = LINKS_COM[LINK_PITCH][2] + _position[X];
            link_comWRTBase(1) = _position[Y] - LINKS_COM[LINK_PITCH][1] * c_theta -
                                 LINKS_COM[LINK_PITCH][0] * s_theta;
            link_comWRTBase(2) = LINKS_COM[LINK_PITCH][0] * c_theta -
                                 LINKS_COM[LINK_PITCH][1] * s_theta + r3;
            break;
        }

        case LINK_ROLL:

        {
            link_comWRTBase(0) = _position[X] + 
                                 LINKS_COM[LINK_ROLL][1] * c_phi + LINKS_COM[LINK_ROLL][0] * s_phi;
            link_comWRTBase(1) = _position[Y] - 
                                 s_theta * (LINKS_COM[LINK_ROLL][0] * c_phi - 
                                 LINKS_COM[LINK_ROLL][1] * s_phi) + LINKS_COM[LINK_ROLL][2] * c_theta;
            link_comWRTBase(2) = c_theta * (LINKS_COM[LINK_ROLL][0] * c_phi - 
                                 LINKS_COM[LINK_ROLL][1] * s_phi) + 
                                 LINKS_COM[LINK_ROLL][2] * s_theta + r3;
            break;
        }

        case LINK_YAW:

        {
            link_comWRTBase(0) = _position[X] - c_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                 LINKS_COM[LINK_YAW][1] * s_psi) +
                                 LINKS_COM[LINK_YAW][2] * s_phi;
            link_comWRTBase(1) = _position[Y] - 
                                c_theta * (LINKS_COM[LINK_YAW][1] * c_psi + 
                                LINKS_COM[LINK_YAW][0] * s_psi) - 
                                s_phi * s_theta * (LINKS_COM[LINK_YAW][0] * c_psi - 
                                LINKS_COM[LINK_YAW][1] * s_psi) - 
                                LINKS_COM[LINK_YAW][2] * c_phi * s_theta;
            link_comWRTBase(2) = c_theta * s_phi * (LINKS_COM[LINK_YAW][0] * c_psi -
                                 LINKS_COM[LINK_YAW][1] * s_psi) - 
                                 s_theta * (LINKS_COM[LINK_YAW][1] * c_psi + 
                                 LINKS_COM[LINK_YAW][0] * s_psi) + 
                                 LINKS_COM[LINK_YAW][2] * c_phi * c_theta + r3;
            break;
        }

        case LINK_PEDAL:

        {
            link_comWRTBase(0) = _position[X] +
                                 d6 * s_phi + LINKS_COM[LINK_PEDAL][2] * s_phi + 
                                 LINKS_COM[LINK_PEDAL][1] * c_phi * c_psi + 
                                 LINKS_COM[LINK_PEDAL][0] * c_phi * s_psi;
            link_comWRTBase(1) = _position[Y] -
                                 d6 * c_phi * s_theta +
                                 LINKS_COM[LINK_PEDAL][1] * (c_theta * s_psi + c_psi * s_phi * s_theta) -
                                 LINKS_COM[LINK_PEDAL][0] * (c_psi * c_theta - s_phi * s_psi * s_theta) -
                                 LINKS_COM[LINK_PEDAL][2] * c_phi * s_theta;
            link_comWRTBase(2) = d6 * c_phi * c_theta - 
                                 LINKS_COM[LINK_PEDAL][0] * (c_psi * s_theta + c_theta * s_phi * s_psi) +
                                 LINKS_COM[LINK_PEDAL][1] * (s_psi * s_theta - c_psi * c_theta * s_phi) +
                                 LINKS_COM[LINK_PEDAL][2] * c_phi * c_theta + r3;

            break;
        }

    }

}