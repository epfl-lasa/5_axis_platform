#ifndef __LP_FILTER__
#define __LP_FILTER__

class LP_Filter
{
	public:
		LP_Filter(float alpha); 
		float update(float raw_input);
		//void setValue(float alpha);		
		void reset();
	private:
		float _alpha;
		float _old_output;
		float _output;
};

#endif /*__LP_FILTER__*/
