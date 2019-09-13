#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

Platform *Platform::me = NULL;

Platform::Platform()
{
  me = this;

    _transmisions[X]=(1.0f/BELT_PULLEY_R)/cos(PI/3.0f); 
    _transmisions[Y]=1.0f/BELT_PULLEY_R; 
    _transmisions[PITCH]=PITCH_REDUCTION_R;
    _transmisions[ROLL]=ROLL_YAW_REDUCTION_R;
    _transmisions[YAW]=ROLL_YAW_REDUCTION_R;

    _maxCurrent[X]=MAX_CURRENT_X;
    _maxCurrent[Y]=MAX_CURRENT_Y;
    _maxCurrent[PITCH]=MAX_CURRENT_PITCH_ROLL_YAW;
    _maxCurrent[ROLL]=MAX_CURRENT_PITCH_ROLL_YAW;
    _maxCurrent[YAW]=MAX_CURRENT_PITCH_ROLL_YAW;

    _torqueConstants[X]=TORQUE_CONSTANT_X/1000; 
    _torqueConstants[Y]=TORQUE_CONSTANT_Y/1000;
    _torqueConstants[PITCH]=TORQUE_CONSTANT_PITCH_ROLL_YAW/1000;
    _torqueConstants[ROLL]=TORQUE_CONSTANT_PITCH_ROLL_YAW/1000;//[Nm/A]
    _torqueConstants[YAW]=TORQUE_CONSTANT_PITCH_ROLL_YAW/1000;
    
    for (int k=0; k<NB_AXIS; k++)
    {
      _maxEffort[k]=_torqueConstants[k]*_maxCurrent[k]*_transmisions[k];
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
    _position[k] = 0.0f;
    _positionOffsets[k] = 0.0f;
    _positionPrev[k] = 0.0f;
    
    positionCtrlClear(k);
    speedCtrlClear(k);

    _effort[k] = 0.0f;
    totalEffortDClear(k);
    _effortM[k] = 0.0f;
    _positionFilters[k] = new LP_Filter(0.6);
    _speedFilters[k] = new LP_Filter(0.95);
    _effortMFilters[k] = new LP_Filter(0.95);
    limitSwitchesClear();

    _ros_position[k]=0.0f;
    _ros_speed[k]=0.0f;
    
    _pidPosition[k] = new PID(&_innerTimer, &_position[k], &_positionCtrlOut[k], &_positionD[k], _kpPosition[k], _kiPosition[k], _kdPosition[k],DIRECT);
    _pidPosition[k]->setMode(AUTOMATIC);
    _pidSpeed[k] = new PID(&_innerTimer, &_speed[k], &_speedCtrlOut[k], &_speedD[k], _kpSpeed[k], _kiSpeed[k], _kdSpeed[k],DIRECT);
    _pidSpeed[k]->setMode(AUTOMATIC);
  }  
  _innerCounter=0;
  
  _ros_ControlledAxis=-1; //! all of them
  _ros_controllerType=TORQUE_ONLY;
  _flagClearLastState=false;
  _flagInWsConstrains=false;
  _ros_flagDefaultControl=true;
   wsConstrainsDefault(-1);
  
  for (int j=0; j<NB_WRENCH_COMPONENTS; j++) // {NORMAL*, CONSTRAINS*, COMPENSATION, FEEDFORWARD}
  {
    if (j<=1){ _ros_effortComp[j]=1;}
    else{_ros_effortComp[j]=0;}
  }

  _tic=false;
  _state = HOMING;
  _lastState=_state;
  _ros_newState=_state;
    
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
  _motorPins[ROLL] = PA_8; // D7 PWM1/1
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

  _enableMotors= new DigitalOut(PB_1);  

  _spi = new SPI(PA_7, PA_6, PA_5); // mosi, miso, sclk https://os.mbed.com/platforms/ST-Nucleo-L476RG/
  // _spi->format(8,0); // Default
  // _spi->frequency(1000000); // Default
  /************************************************************* */

  wait_ms(10); //! Wait a bit after the SPI starts

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
}
