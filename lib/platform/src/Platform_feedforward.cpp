#include "Platform.h"

const float vibMagnitude = -240; //![N / m/s]
const float vibDecayRate = 60; //! [1/s]
const float vibFrequency = 32.6; //! [Hz]


LP_Filter vibFilter(0.1f);

using namespace std;
using namespace Eigen;


float const VIB_EFFORT_LIMS[NB_AXIS] = {5.0f, 5.0f , 1.0 , 1.0 , 1.0};


void Platform::feedForwardControl()
{   
    _feedForwardTorque.setConstant(0.0f);
    eventVibration(FRAME_PEDAL);
    _effortD_ADD.col(FEEDFORWARD) = _feedForwardTorque.rowwise().sum();
}


void Platform::eventVibration(frame_chain frame_)
{
    
    if (_flagVibration)
    {

        Eigen::Matrix<float, NB_AXIS, 1> vibTorques, limTorques;
        Eigen::Matrix<float, 6, 1> vibTau, vibTauTemp;
        Eigen::Matrix<float, 6, NB_AXIS> jacobianPedalFrame;
        vibTorques.setConstant(0.0f);
        limTorques<<VIB_EFFORT_LIMS[X], VIB_EFFORT_LIMS[Y], VIB_EFFORT_LIMS[PITCH],VIB_EFFORT_LIMS[ROLL],VIB_EFFORT_LIMS[YAW];
        vibTau.setConstant(0.0f);
        vibTauTemp.setConstant(0.0f);
        jacobianPedalFrame.setConstant(0.0f);

        jacobianPedalFrame = geometricJacobian(FRAME_PEDAL);

        float uniVib = vibMagnitude * exp(-vibDecayRate * (_timestamp - _vibGenStamp) * 1e-6 ) *
                                 sin(2 * M_PI * vibFrequency * (_timestamp - _vibGenStamp) );

        uniVib = vibFilter.update(uniVib);
        
        vibTauTemp(X) = uniVib;
        vibTauTemp(Y) = uniVib;

        vibTau = vibTauTemp.cwiseProduct(jacobianPedalFrame * _speed);
        
        vibTorques = jacobianPedalFrame.transpose() * vibTau;
        
        if ( abs(uniVib) < 0.01f)
        {
            _flagVibration = false;
        }

        _feedForwardTorque.col(FF_VIB) = boundMat(vibTorques,-limTorques,limTorques);
         
    }
    else
    {
        _vibGenStamp = _timestamp;
    }
    
}