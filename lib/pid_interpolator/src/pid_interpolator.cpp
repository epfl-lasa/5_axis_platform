#include <pid_interpolator.h>

PIDInterpolator::PIDInterpolator(unsigned int nPoints, unsigned int freqRatio, float* input) {
  _ctrlSteps = 0;
  _freqRatio = freqRatio;
  _sensitivity = 1.0 / _freqRatio;
  _output=0.0f;
  _nbPoints = nPoints;
  _nbPointsInv=1/_nbPoints;
  _currentPoint = 0u;
  _old_input = new float[_nbPoints];
  _last_input = input;
  for (unsigned int k = 0; k<_nbPoints; k++)  {_old_input[k]=0.0f;};

}

PIDInterpolator::~PIDInterpolator()
{
  delete(_old_input);
  delete(_last_input);
}

float PIDInterpolator::update_output(){
  
  float av_delta = 0; 
  for (unsigned int k=0; k<_nbPoints; k++)
  {
    av_delta += _old_input[k + 1] - _old_input[k];
  }

  _output = _output + av_delta * _nbPointsInv * _sensitivity;
  _ctrlSteps++;

  if (_ctrlSteps >= _freqRatio) {
    update_model(*_last_input);
    _ctrlSteps =0;
  }
  return _output;
}
 
void PIDInterpolator::update_model(float new_input) //! Every time a new position command is send
{
  _currentPoint++;
  if (_currentPoint == _nbPoints) { _currentPoint=0; };
  _old_input[_currentPoint] = new_input;
}

void PIDInterpolator::reset()
{
  _ctrlSteps = 0;
  _output = 0.0f;
  _currentPoint = 0u;
  for (unsigned int k = 0; k < _nbPoints; k++) {
    _old_input[k] = 0.0f;
  }
}