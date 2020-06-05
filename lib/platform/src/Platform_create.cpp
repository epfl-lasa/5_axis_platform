#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

const char *Platform_Names[]{"UNKNOWN", "RIGHT PLATFORM", "LEFT PLATFORM"};

extern float LINKS_MASS[NB_LINKS];

const float POS_PID_FILTER_GAINS[NB_AXIS] = {0.5f, 0.5f, 0.8f, 0.8f, 0.8f };
const float VEL_PID_FILTER_GAINS[NB_AXIS] = {0.5f, 0.5f, 0.8f, 0.8f, 0.8f};

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
    AXES};
#undef ListofAxes

Platform *Platform::me = NULL;

Platform::Platform()
{
  me = this;
  _stop=false;
  _flagEmergencyCalled = false;
  _effortD_ADD.setConstant(0.0f);
  _transmisions[X] = X_TRANSMISSION;
  _transmisions[Y] = Y_TRANSMISSION;
  _transmisions[PITCH] = PITCH_REDUCTION_R;
  _transmisions[ROLL] = ROLL_YAW_REDUCTION_R;
  _transmisions[YAW] = ROLL_YAW_REDUCTION_R;

  _maxCurrent[X] = MAX_CURRENT_X;
  _maxCurrent[Y] = MAX_CURRENT_Y;
  _maxCurrent[PITCH] = MAX_CURRENT_PITCH_ROLL_YAW;
  _maxCurrent[ROLL] = MAX_CURRENT_PITCH_ROLL_YAW;
  _maxCurrent[YAW] = MAX_CURRENT_PITCH_ROLL_YAW;

  _torqueConstants[X] = TORQUE_CONSTANT_X / 1000;
  _torqueConstants[Y] = TORQUE_CONSTANT_Y / 1000;
  _torqueConstants[PITCH] = TORQUE_CONSTANT_PITCH_ROLL_YAW / 1000;
  _torqueConstants[ROLL] = TORQUE_CONSTANT_PITCH_ROLL_YAW / 1000; //[Nm/A]
  _torqueConstants[YAW] = TORQUE_CONSTANT_PITCH_ROLL_YAW / 1000;

  for (uint k = 0; k < NB_AXIS; k++) {
    _maxEffort[k] = _torqueConstants[k] * _maxCurrent[k] * _transmisions[k]; //DO NOT USE IN A CONTROLLER, JUST IN A MEASUREMENT FOR MAPPING
    }

  _encoderScale[X] =ENCODERSCALE_X;
  _encoderScale[Y] =ENCODERSCALE_Y;
  _encoderScale[PITCH] =ENCODERSCALE_PITCH;
  _encoderScale[ROLL] =ENCODERSCALE_ROLL;
  _encoderScale[YAW] =ENCODERSCALE_YAW;

  _encoderSign[X] =ENCODERSIGN_X;
  _encoderSign[Y] =ENCODERSIGN_Y;
  _encoderSign[PITCH] =ENCODERSIGN_PITCH;
  _encoderSign[ROLL] =ENCODERSIGN_ROLL;
  _encoderSign[YAW] =ENCODERSIGN_YAW;

  _motorSign[X] =MOTORSIGN_X;
  _motorSign[Y] =MOTORSIGN_Y;
  _motorSign[PITCH] =MOTORSIGN_PITCH;
  _motorSign[ROLL] =MOTORSIGN_ROLL;
  _motorSign[YAW] =MOTORSIGN_YAW;

  _wsRange[X]=X_RANGE;
  _wsRange[Y]=Y_RANGE;
  _wsRange[PITCH]=PITCH_RANGE;
  _wsRange[ROLL]=ROLL_RANGE;
  _wsRange[YAW]=YAW_RANGE;
  
  for(int k = 0; k < NB_AXIS; k++)
  {
    _effortD[k] = 0.0f;
    _position[k] = 0.0f;
    _speed(k) = 0.0f;
    _speedPrev(k) = 0.0f;
    _positionOffsets[k] = 0.0f;
    _positionPrev[k] = 0.0f;
    
    positionCtrlClear(k);
    speedCtrlClear(k);
    totalEffortDClear(k);
    _effortM[k] = 0.0f;
    
    _posDesiredFilters[k].setAlpha(0.0);
    _effortMFilters[k].setAlpha(0.8);
    _adc_sum[k]=0.0f;

    limitSwitchesClear();

    _ros_position[k]=0.0f;
    _ros_speed[k]=0.0f;
    _ros_effort[k] = 0.0f;

    _pidPosition[k] = new PID(&_innerTimer, &_position[k], &_positionCtrlOut[k], &_positionD_filtered[k], _kpPosition[k], _kiPosition[k], _kdPosition[k], DIRECT, POS_PID_FILTER_GAINS[k]);
    _pidPosition[k]->setMode(AUTOMATIC);
    _pidSpeed[k] = new PID(&_innerTimer, &_speed(k), &_speedCtrlOut[k], &_speedD[k], _kpSpeed[k], _kiSpeed[k], _kdSpeed[k],DIRECT, VEL_PID_FILTER_GAINS[k]);
    _pidSpeed[k]->setMode(AUTOMATIC);
  }

  //! Change filters
  // _positionFilters[X].setAlpha(0.56);
  // _positionFilters[Y].setAlpha(0.56);
  // _positionFilters[PITCH].setAlpha(0.85);
  // _positionFilters[ROLL].setAlpha(0.85);
  // _positionFilters[YAW].setAlpha(0.85);

  _positionFilters[X].setAlpha(0.0);
  _positionFilters[Y].setAlpha(0.0);
  _positionFilters[PITCH].setAlpha(0.0);
  _positionFilters[ROLL].setAlpha(0.0);
  _positionFilters[YAW].setAlpha(0.0);

  _speedFilters[X].setAlpha(0.96);
  _speedFilters[Y].setAlpha(0.96);
  _speedFilters[PITCH].setAlpha(0.96);
  _speedFilters[ROLL].setAlpha(0.97);
  _speedFilters[YAW].setAlpha(0.97);

  _accFilters[X].setAlpha(0.96);
  _accFilters[Y].setAlpha(0.96);
  _accFilters[PITCH].setAlpha(0.96);
  _accFilters[ROLL].setAlpha(0.97);
  _accFilters[YAW].setAlpha(0.97);

  _innerCounterADC=0;

  _ros_ControlledAxis = -1; //! all of them
  _ros_controllerType=TORQUE_ONLY;
  _platform_controllerType=_ros_controllerType;
  _flagClearLastState=false;
  _flagControllerTypeChanged=false;
  _flagInWsConstrains = false;

  _ros_flagDefaultControl=true;
  
  for(int c = 0; c<NB_FI_CATEGORY; c++)
  {
    _flagInputReceived[c] = false; //! To be used specially for the telemanipulation state
  }
  
   wsConstrainsDefault(-1);
  
  for (uint j=0; j<NB_EFFORT_COMPONENTS; j++) // {NORMAL*, CONSTRAINS*, COMPENSATION, FEEDFORWARD}
  {
    if (j<=1){ _ros_effortComp[j]=1;}
    else{_ros_effortComp[j]=0;}
  }

  _tic=false;
  _ros_state = EMERGENCY;
  _platform_state=_ros_state;
  
    
  // Reset the flags that acknowledge when the state is entered for the first time 
  _enterStateOnceFlag[HOMING]=false;
  _enterStateOnceFlag[CENTERING]=false;
  _enterStateOnceFlag[TELEOPERATION]=false;
  _enterStateOnceFlag[STANDBY]=false;
  _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;

  _ros_controllerType= TORQUE_ONLY;
  /*******DESIGNATIONS OF PINS IN THE MICROCONTROLLER NUCLEO L476RG */

  _csPins[X] = PA_10;  //! CS1 -> Lateral NOT as PWMX/XN
  _csPins[Y] = PB_10;  //! CS2  -> Dorsi/Plantar Flexion NOT as PWMX/XN
  _csPins[PITCH] = PA_9; //! CS3 -> Flexion/Extension of the Leg NOT as PWM/XN
  _csPins[ROLL] = PB_6;  //! CS4 -> roll and yaw encoder 1 NOT as PWMX/XN
  _csPins[YAW] = PB_2;  //! CS5 -> roll and yaw encoder 2  NOT as PWMX/X
  
  _motorPins[X] = PC_7_ALT0; // D9 PWM8/2
  _motorPins[Y] = PB_3; // D3  PWM2/2
  _motorPins[PITCH] = PB_5; // D4 PWM3/2 
  _motorPins[ROLL] = PA_8; // D7 PWM1.0f/1
  _motorPins[YAW] = PB_4; // D5  PWM3/1

  _limitSwitchesPins[X] = PC_8; //  NOT as PWMX/XN
  _limitSwitchesPins[Y] = PC_5; // NOT as PWMX/XN
  _limitSwitchesPins[PITCH] = PC_6; // NOT as PWMX/XN
  
  _esconEnabledPins[X] = PC_3;
  _esconEnabledPins[PITCH] = PC_2;
  _esconEnabledPins[Y] = PH_1;
  _esconEnabledPins[ROLL] = PH_0;
  _esconEnabledPins[YAW] = PC_13;


  _motorCurrentsPins[X]=PC_0;
  _motorCurrentsPins[PITCH]=PC_1;
  _motorCurrentsPins[Y]=PB_0;
  _motorCurrentsPins[ROLL]=PA_4;  
  _motorCurrentsPins[YAW]=PA_0;

  // _motorCurrentsPins[X] = A5;
  // _motorCurrentsPins[Y] = A3;
  // _motorCurrentsPins[PITCH] = A4;
  // _motorCurrentsPins[ROLL] = A2;
  // _motorCurrentsPins[YAW] = A0;

  _enableMotors= new DigitalOut(PB_1);  

  _spi = new SPI(PA_7, PA_6, PA_5); // mosi, miso, sclk https://os.mbed.com/platforms/ST-Nucleo-L476RG/
  // _spi->format(8,0); // Default
  // _spi->frequency(1000000); // Default
  /************************************************************* */

  wait_us(10000); //! Wait a bit after the SPI starts

  for(int k = 0; k < NB_AXIS; k++)
  {
    _encoders[k] = new QEC_1X(_csPins[k]);
    _motors[k] = new PwmOut(_motorPins[k]);  {
    _esconEnabled[k]=new InterruptIn(_esconEnabledPins[k]);
    _motorCurrents[k]= new AnalogIn(_motorCurrentsPins[k]);

  }
    if (k<NB_SWITCHES)
      {
        _limitSwitches[k] = new InterruptIn(_limitSwitchesPins[k]);
        _limitSwitches[k]->mode(PullUp);
      }
    else
    {
      _limitSwitches[k]=NULL;
    }
      
   }
_timestamp=0; // We don't read the timer until the platform is initialized

_recoveringFromError=false;
_flagBiasADCOk=false;
_allEsconOk = 1;


_compensationEffort.setConstant(0.0f);

for (int sign_=0; sign_<NB_SIGN_COMP; sign_++)
{
  _dryFrictionEffortSign[sign_].setConstant(0.0f);
  _predictors[sign_].setConstant(0.0f);
  _betas[sign_].setConstant(0.0f);
  _mean[sign_].setConstant(0.0f);
  _stdInv[sign_].setConstant(0.0f);

}



// //% For friction compensation betas of a linear regression
_betas[NEG].col(X) << 0.1000f, 0.5090f, -0.1961f, -0.2133f, 0.6077f, -0.3332f, 0.3717f, 0.1320f, -0.2567f, -0.0420;
_betas[POS].col(X) << 1.6894f, -0.3858f, -0.0671f, -0.1247f, -0.4456f, 0.7966f, -0.4280f, -0.0831f, -0.1320f, 0.3302;

_mean[NEG].col(X) << -0.0139f, -0.0143f, 0.0623f, -0.0613f, 0.0552f, 0.0011f, 0.0014f, 0.0305f, 0.0251f, 0.0312 ;
_mean[POS].col(X) << -0.0147f, -0.0143f, 0.0623f, -0.0613f, 0.0552f, 0.0011f, 0.0014f, 0.0305f, 0.0251f, 0.0312 ;

_stdInv[NEG].col(X) << 33.6700f, 28.5714f, 6.1275f, 6.8306f, 5.9595f, 833.3333f, 666.6667f, 38.1679f, 37.3134f, 28.0112f;
_stdInv[POS].col(X) << 33.3333f, 28.5714f, 6.1275f, 6.8306f, 5.9595f, 833.3333f, 666.6667f, 38.1679f, 37.3134f, 28.0112f;

_bias[NEG].col(X) << -4.63060f;
_bias[POS].col(X) << 3.17075f;

_betas[NEG].col(Y) << 0.1197f, 1.6298f, -0.1719f, -0.0223f, 0.1983f, 0.0202f, -0.9100f, 0.0566f, -0.1002f, -0.1577f;
_betas[POS].col(Y) << 0.1095f, 0.0843f, 0.1773f, -0.1787f, -0.0708f, 0.0936f, 1.4363f, -0.0854f, 0.0247f, 0.1376f;

_mean[NEG].col(Y) << 0.0094f, 0.0613f, 0.0325f, -0.0148f, 0.0062f, 0.0022f, 0.0045f, 0.0307f, 0.0304f, 0.0533f;
_mean[POS].col(Y) << 0.0094f, 0.0613f, 0.0325f, -0.0148f, 0.0062f, 0.0022f, 0.0045f, 0.0307f, 0.0304f, 0.0533f;

_stdInv[NEG].col(Y) << 21.8818f, 35.7143f, 5.7870f, 5.7372f, 4.3178f, 416.6667f, 370.3704f, 32.4675f, 40.1606f, 23.4742f;
_stdInv[POS].col(Y) << 21.8818f, 25.3807f, 5.7971f, 5.7372f, 4.3178f, 416.6667f, 555.5556f, 32.6797f, 40.1606f, 23.4742f;

_bias[NEG].col(Y) << -8.18028f;
_bias[POS].col(Y) << 10.5657f;

_gravityCompJointsTorque.setConstant(0.0f);


 _massLinks.setConstant(0.0f);
for (int link = 0; link<NB_LINKS; link++ )
{
  _massLinks(link) = LINKS_MASS[link]; 
}

  _flagContact=false;
  _flagVibration=false;
  _feedForwardTorque.setConstant(0.0f);
}