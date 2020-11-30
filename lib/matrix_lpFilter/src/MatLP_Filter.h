#ifndef __MATLP_FILTER__
#define __MATLP_FILTER__

#include <Dense>
#include <Core>

using namespace Eigen;

class MatLP_Filter
{
	public:
        MatLP_Filter(MatrixXf alphas);
        ~MatLP_Filter();
        MatrixXf update(MatrixXf raw_matrix);
        void setAlphas(MatrixXf alphas);
        void reset();
        void setBias(MatrixXf bias_);
	private:
		MatrixXf _bias;
		MatrixXf _alphas;
		MatrixXf _alphasComp;
		MatrixXf _old_output;
		MatrixXf _output;
};

#endif /*__MATLP_FILTER__*/
