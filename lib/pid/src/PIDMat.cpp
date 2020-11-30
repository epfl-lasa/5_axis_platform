/**********************************************************************************************
 * PIDMat Library 
 * Based on the Brett Beauregard library 
 * This Library is licensed under the MIT License
 **********************************************************************************************/
/*Modified by Jacob 13.06.2020*/
#include <PIDMat.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/

PIDMat::PIDMat(Timer *timer_, MatrixXf *Input, MatrixXf *Output, MatrixXf *Setpoint,
               MatrixXf Kp, MatrixXf Ki, MatrixXf Kd, int POn, int ControllerDirection,
               MatrixXf filterGains) {
  _myOutput = Output;
  _myInput = Input;
  _mySetpoint = Setpoint;
  _myTimer = timer_;
  _inAuto = false;

  _SampleTime = 100; // default Controller Sample Time is 100 microseconds

  PIDMat::setControllerDirection(ControllerDirection);

  PIDMat::setTunings(Kp, Ki, Kd, POn);

  _lastTime = _myTimer->read_us() - _SampleTime;

  _dInputFilter = new MatLP_Filter(filterGains);
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PIDMat::PIDMat(Timer *timer_, MatrixXf *Input, MatrixXf *Output, MatrixXf *Setpoint,
         MatrixXf Kp, MatrixXf Ki, MatrixXf Kd, int ControllerDirection, MatrixXf filterGains)
    : PIDMat::PIDMat(timer_, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, filterGains)
{

}

PIDMat::PIDMat(Timer *timer_, MatrixXf *Input, MatrixXf *Output, MatrixXf *Setpoint,
         MatrixXf Kp, MatrixXf Ki, MatrixXf Kd, int ControllerDirection)
    : PIDMat::PIDMat(timer_, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, MatrixXf::Zero(Kp.rows(),Kp.cols()))
{
}

PIDMat::~PIDMat()
{
   delete(_myOutput);
   delete(_myTimer);
   delete(_myInput);
   delete(_mySetpoint);
   _dInputFilter->~MatLP_Filter();
   delete(_dInputFilter);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PIDMat::compute()
{
   if(!_inAuto) {return false;}
   unsigned long now = _myTimer->read_us();
   unsigned long timeChange = (now - _lastTime);
   if(timeChange>=_SampleTime)
   {
      /*Compute all the working error variables*/
      MatrixXf input = *_myInput;
      MatrixXf error = *_mySetpoint - input;
      
      _dInput = _dInputFilter->update(input - _lastInput);
      _outputSum += (error.cwiseProduct(_ki));

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!_pOnE) _outputSum-= _dInput.cwiseProduct(_kp);

      /*Add Proportional on Error, if P_ON_E is specified*/
	   MatrixXf output;
      if(_pOnE) output = error.cwiseProduct(_kp);
      else output.setConstant(0.0f);

      /*Compute Rest of PIDMat Output*/
      output += _outputSum - _kd * _dInput;

      Array<bool, Eigen::Dynamic, Eigen::Dynamic> checkOutput;
      Array<bool, Eigen::Dynamic, Eigen::Dynamic> checkKi;
      ArrayXf checkTemp;

      // Anti-windup reset of I term
      
      checkKi = _ki.array() != 0.0f;
      if (checkKi.any())
      {
        checkOutput = output.array() > _outMax.array();
        checkTemp = (checkOutput * checkKi).template cast<float>();
        _outputSum -= ((output - _outMax).array() * checkTemp).matrix();
        checkOutput = output.array() < _outMin.array();
        checkTemp = (checkOutput * checkKi).template cast<float>();
        _outputSum -= ((output - _outMin).array() * checkTemp).matrix();
      }
      output = output.cwiseMin(_outMax).cwiseMax(_outMin);
      _outputSum = _outputSum.cwiseMin(_outSumMax).cwiseMax(_outSumMin);

	   *_myOutput = output; 

      /*Remember some variables for next time*/
      _lastInput = input;
      _lastTime = now;
      _errorM = error;

      return true;
   }
   else return false;
   
   return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PIDMat::setTunings(MatrixXf Kp, MatrixXf Ki, MatrixXf Kd, int POn)
{
   // MatrixXf Ki=Kp/Ti;
   // MatrixXf Kd=Kp*Td;

   if ((Kp.array() < 0.0f).all() || (Ki.array() < 0.0f).all() || (Kd.array()<0.0f).all()) {return;}

   _pOn = POn;
   _pOnE = POn == P_ON_E;

   _dispKp = Kp; _dispKi = Ki; _dispKd = Kd;

   float SampleTimeInSec = ((float)_SampleTime)*1e-6f;
   _kp = Kp;
   _ki = Ki * SampleTimeInSec;
   _kd = Kd / SampleTimeInSec;

  if(_controllerDirection ==REVERSE)
   {
      _kp = (0.0f - _kp.array()).matrix();
      _ki = (0.0f - _ki.array()).matrix();
      _kd = (0.0f - _kd.array()).matrix();
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PIDMat::setTunings(MatrixXf Kp, MatrixXf Ki, MatrixXf Kd){
    setTunings(Kp, Ki, Kd, _pOn); 
}

/*SetFilterGains(...)**********************************************************
 *Set the gains of the exponential smoothing first order filter of the dInput
 *****************************************************************************/
void PIDMat::setFilterGains(MatrixXf filterGains)
{
   _dInputFilter->setAlphas(filterGains);
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in micros, at which the calculation is performed
 ******************************************************************************/
void PIDMat::setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = NewSampleTime/_SampleTime;
      _ki *= ratio;
      _kd /= ratio;
      _SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clip it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PIDMat::setOutputLimits(MatrixXf Min, MatrixXf Max)
{
   _outMin = Min.cwiseMin(Max);
   _outMax = Max.cwiseMax(Min);
   _outSumMax = 0.9f*_outMax;
   _outSumMin = 0.9f*_outMin;


   if(_inAuto)
   {
     MatrixXf output = *_myOutput;
     *_myOutput = output.cwiseMin(_outMax).cwiseMax(_outMin);

     _outputSum = _outputSum.cwiseMin(_outSumMax).cwiseMax(_outSumMin);
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PIDMat::setMode(int Mode)
{
    bool newAuto = (Mode == IDLE_CONTROL);
    if(newAuto && !_inAuto)
    {  /*we just went from manual to auto*/
        PIDMat::initialize();
    }
    _inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PIDMat::initialize()
{
   _outputSum = *_myOutput * 0.5f;
   _lastInput = *_myInput;
   _outputSum = _outputSum.cwiseMin(_outSumMax).cwiseMax(_outSumMin);
}

/* SetControllerDirection(...)*************************************************
 * The PIDMat will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PIDMat::setControllerDirection(int Direction)
{
   if(_inAuto && Direction !=_controllerDirection)
   {
	   _kp = (0.0f - _kp.array()).matrix();
      _ki = (0.0f - _ki.array()).matrix();
      _kd = (0.0f - _kd.array()).matrix();
   }
   _controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PIDMat.  they're here for display
 * purposes.  this are the functions the PIDMat Front-end uses for example
 ******************************************************************************/
MatrixXf PIDMat::getKp(){ return  _dispKp;}
MatrixXf PIDMat::getKi(){ return  _dispKi;}
MatrixXf PIDMat::getKd(){ return  _dispKd;}
MatrixXf PIDMat::getProportionalTerm() 
{ 
   if (_pOnE){
      return _errorM.cwiseProduct(_kp);
      }
   else {
      return _dInput.cwiseProduct(-_kp);
      }
}
MatrixXf PIDMat::getDerivativeTerm() { return _dInput.cwiseProduct(_kd); }
MatrixXf PIDMat::getIntegralTerm(){return _outputSum;}
MatrixXf PIDMat::getError() {return _errorM;}
int PIDMat::getMode(){ return  _inAuto ? IDLE_CONTROL : ENABLE_CONTROL;}
int PIDMat::getDirection(){ return _controllerDirection;}

void PIDMat::reset() {
   _lastInput=*_myInput;
   _errorM=*_mySetpoint - *_myInput;
   _outputSum.setConstant(0.0f);
}