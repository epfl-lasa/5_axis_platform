#include <MatLP_Filter.h>

MatLP_Filter::MatLP_Filter(MatrixXf alphas)
{
  _output = Eigen::MatrixXf::Zero(alphas.rows(),alphas.cols());
  _old_output = Eigen::MatrixXf::Zero(alphas.rows(), alphas.cols());
  _alphas=alphas;
  _alphasComp = (1.0f - alphas.array()).matrix();
  _bias = Eigen::MatrixXf::Zero(alphas.rows(), alphas.cols());
}


MatrixXf MatLP_Filter::update(MatrixXf raw_matrix){
  _output=_alphas.cwiseProduct(_old_output) + _alphasComp.cwiseProduct(raw_matrix);
  _old_output=_output;
  return _output - _bias;
}

void MatLP_Filter::setBias(MatrixXf bias_)
{
  _bias=bias_;
}

void MatLP_Filter::reset()
{
  _output.setConstant(0.0f);
  _old_output.setConstant(0.0f);
}

void MatLP_Filter::setAlphas(MatrixXf alphas)
{
  _alphas=alphas;
}