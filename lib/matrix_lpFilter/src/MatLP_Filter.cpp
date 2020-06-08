#include <MatLP_Filter.h>

MatLP_Filter::MatLP_Filter()
{
  _output.setConstant(0.0f);
  _old_output.setConstant(0.0f);
  _alpha = 0.0f;
  _bias.setConstant(0.0f);
}

MatLP_Filter::MatLP_Filter(float alpha)
{
  _output.setConstant(0.0f);
  _old_output.setConstant(0.0f);
  _alpha=alpha;
  _bias.setConstant(0.0f);
}


Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> MatLP_Filter::update(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> raw_matrix){
  _output=_alpha*_old_output + (1.0f-_alpha)*raw_matrix;
  _old_output=_output;
  return _output - _bias;
}

void MatLP_Filter::setBias(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> bias_)
{
  _bias=bias_;
}

void MatLP_Filter::reset()
{
  _output.setConstant(0.0f);
  _old_output.setConstant(0.0f);
}

void MatLP_Filter::setAlpha(float alpha)
{
  _alpha=alpha;
}