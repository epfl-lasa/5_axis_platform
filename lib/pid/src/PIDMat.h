#ifndef PIDMAT_H
#define PIDMAT_H
#include "LP_Filter.h"
#include "MatLP_Filter.h"
#include <mbed.h>
#include <Dense>
#include <Core>

using namespace Eigen;

class PIDMat {
public:
// Constants used in some of the functions below
#define IDLE_CONTROL 1
#define ENABLE_CONTROL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

  // commonly used functions
  // **************************************************************************
  PIDMat(Timer *, MatrixXf *, MatrixXf *,
      MatrixXf *, // * constructor.  links the PIDMat to the Input, Output, and
      MatrixXf, MatrixXf, MatrixXf, int, int,
      MatrixXf); //   Setpoint.  Initial tuning parameters are also set here.
              //   (overload for specifying proportional mode)

  PIDMat(Timer *, MatrixXf *, MatrixXf *,
      MatrixXf *, // * constructor.  links the PIDMat to the Input, Output, and
      MatrixXf, MatrixXf, MatrixXf, int,
      MatrixXf); //   Setpoint.  Initial tuning parameters are also set here

  PIDMat(Timer *, MatrixXf *, MatrixXf *, MatrixXf *, MatrixXf, MatrixXf, MatrixXf, int);

  ~PIDMat();

  void setMode(int Mode); // * sets PIDMat to either Manual (0) or Auto (non-0)

  bool compute(); // * performs the PIDMat calculation.  it should be
                  //   called every time loop() cycles. ON/OFF and
                  //   calculation frequency can be set using SetMode
                  //   SetSampleTime respectively

  void setOutputLimits(
      MatrixXf,
      MatrixXf); // * clips the output to a specific range. 0-255 by default, but
              //   it's likely the user will want to change this depending on
              //   the application

  // available but not commonly used functions
  // ********************************************************
  void
  setTunings(MatrixXf,
             MatrixXf,  // * While most users will set the tunings once in the
             MatrixXf); //   constructor, this function gives the user the option
  //   of changing tunings during runtime for Adaptive control
  void setTunings(MatrixXf, MatrixXf, // * overload for specifying proportional mode
                  MatrixXf, int);
  
  void setFilterGains(MatrixXf);

  void setControllerDirection(
      int); // * Sets the Direction, or "Action" of the controller. DIRECT
            //   means the output will increase when error is positive. REVERSE
  //   means the opposite.  it's very unlikely that this will be needed
  //   once it is set in the constructor.
  void setSampleTime(int); // * sets the frequency, in Microseconds, with which
  //   the PIDMat calculation is performed.  default is 100

  // Display functions
  // ****************************************************************
  MatrixXf getKp(); // These functions query the pid for interal values.
  MatrixXf getKi(); //  they were created mainly for the pid front-end,
  MatrixXf getKd(); // where it's important to know what is actually
  MatrixXf getProportionalTerm();
  MatrixXf getIntegralTerm();
  MatrixXf getDerivativeTerm();
  MatrixXf getError();
  int getMode();      //  inside the PIDMat.
  int getDirection(); //
  void reset();

private:
  void initialize();

  MatLP_Filter* _dInputFilter;

  MatrixXf _outputSum, _dInput;
  MatrixXf _errorM;

  MatrixXf _dispKp; // * we'll hold on to the tuning parameters in user-entered
  MatrixXf _dispKi; //   format for display purposes
  MatrixXf _dispKd; //

  MatrixXf _kp; // * (P)roportional Tuning Parameter
  MatrixXf _ki; // * (I)ntegral Tuning Parameter
  MatrixXf _kd; // * (D)erivative Tuning Parameter

  int _controllerDirection;
  int _pOn;

  MatrixXf *_myInput;  // * Pointers to the Input, Output, and Setpoint variables
  MatrixXf *_myOutput; //   This creates a hard link between the variables and the
  MatrixXf *_mySetpoint; //   PIDMat, freeing the user from having to constantly tell us
                   //   what these values are.  with pointers we'll just know.
  Timer *_myTimer;

  unsigned long _lastTime;
  MatrixXf _lastInput;

  unsigned long _SampleTime;
  MatrixXf _outMin, _outMax;
  MatrixXf _outSumMin, _outSumMax;
  bool _inAuto, _pOnE;
};
#endif
