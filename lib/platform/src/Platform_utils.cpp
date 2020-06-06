#include "Platform.h"
#include "definitions.h"
#include "definitions_2.h"


//! 1
float Platform::map(float x, float in_min, float in_max, float out_min, float out_max)
{
  float mapping = 0.0f;
  if (!isnanf(x) && !isinf(x))
  {  
    mapping = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
  else
  {_nh.logfatal("You have inf or nan numbers");}

  return mapping < out_min ? out_min : (mapping>out_max ? out_max : mapping);
}

float Platform::clip(float x, float out_min, float out_max)
{
  float clip_ = 0.0f;

  if (!isnanf(x) && !isinf(x)) {
    clip_ = x;
  } 
  
  else {
  _nh.logfatal("You have inf or nan numbers");
  }
  return clip_ < out_min ? out_min : (clip_ > out_max ? out_max : clip_);
}


float Platform::smoothRise(float x, float a, float b)
{
  float y;
  if (x < a)
  {
    y = 0.0f;
  }
  else if (x > b)
  {
    y = 1.0f;
  }
  else
  {
    y = (1.0f + sin(M_PI * (x - a) / (b - a) - M_PI / 2.0f)) / 2.0f;
  }

  return y;
}

float Platform::smoothFall(float x, float a, float b)
{
  return 1.0f - smoothRise(x, a, b);
}

float Platform::smoothRiseFall(float x, float a, float b, float c, float d)
{
  return smoothRise(x, a, b) * smoothFall(x, c, d);
}

Eigen::Matrix<float, Eigen::Dynamic, 1> Platform::boundMat(Eigen::Matrix<float, Eigen::Dynamic, 1> x, Eigen::Matrix<float, Eigen::Dynamic, 1> limit)
{
  
  return x.cwiseMin(limit).cwiseMax(-limit);

}
