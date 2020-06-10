#ifndef __MATLP_FILTER__
#define __MATLP_FILTER__

#include "/home/lsrob107772/.platformio/lib/Eigen_ID3522/Dense.h"

using namespace Eigen;

class MatLP_Filter
{
	public:
		MatLP_Filter(float alpha, int length,int width);
		Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> update(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> raw_matrix);
		void setAlpha(float alpha);		
		void reset();
		void setBias(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> bias_);
	private:
		Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> _bias;
		float _alpha;
		Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> _old_output;
		Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> _output;
};

#endif /*__MATLP_FILTER__*/
