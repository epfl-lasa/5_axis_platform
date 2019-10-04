#ifndef __PID_INTERPOLATOR__
#define __PID_INTERPOLATOR__

class PIDInterpolator
{
	public:
          PIDInterpolator(unsigned int nPoints, unsigned int freqRatio, float* input) ;
          ~PIDInterpolator();
          float update_output();
          void update_model(float new_input);
          void reset();
	private:
		float _alpha;
		float* _old_input;
		float* _last_input;
		float _output;
		float _sensitivity;
		float _freqRatio;
		unsigned int _nbPoints;
		float _nbPointsInv;
		unsigned int _currentPoint;
		unsigned int _ctrlSteps;
};

#endif /*__PID_INTERPOLATOR__*/
