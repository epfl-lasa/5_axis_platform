#include <LP_Filter.h>

LP_Filter::LP_Filter(float alpha)
{
  _output=0.0f;
  _old_output=0.0f;
  _alpha=alpha;
}


/*void LP_Filter::LP_Filter_init() {

   // this->QEC_config();
}
*/
float LP_Filter::Update(float raw_input){
  _output=_alpha*_old_output + (1.0-_alpha)*raw_input;
  _old_output=_output;
  return _output;
}