#include "Platform.h"
#include "definitions.h"

const char *Platform_Names[]{"UNKNOWN", "RIGHT PLATFORM", "LEFT PLATFORM"};

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
    
  _effortD.setConstant(0.0f);
  _effortM.setConstant(0.0f);
  _positionD.setConstant(0.0f);
  _positionD_filtered.setConstant(0.0f);
  _position.setConstant(0.0f);
  _positionPrev.setConstant(0.0f);
  _positionOffsets.setConstant(0.0f);
  _virtualWall = Eigen::Map<const Eigen::MatrixXf>(C_WS_LIMITS, NB_AXIS, 1);
  _speed.setConstant(0.0f);
  _speedD.setConstant(0.0f);
  _speedPrev.setConstant(0.0f);
  _acceleration.setConstant(0.0f);

  _flagCalculateSinCos = false;
  _c_theta = 0.0f;
  _c_phi = 0.0f;
  _c_psi = 0.0f;
  _s_theta =0.0f;
  _s_phi = 0.0f;
  _s_psi = 0.0f;

  for(int k = 0; k < NB_AXIS; k++)
  {
    positionCtrlClear(k);
    _platform_kpPosition(k) = 0.0f;
    _platform_kiPosition(k) = 0.0f;
    _platform_kdPosition(k) = 0.0f;
    
    
    speedCtrlClear(k);
    _platform_kpSpeed(k) = 0.0f;
    _platform_kiSpeed(k) = 0.0f;
    _platform_kdSpeed(k) = 0.0f;
    totalEffortDClear(k);

    _posDesiredFilters[k].setAlpha(0.5f);
    _effortMFilters[k].setAlpha(0.8f);
    _adc_sum[k]=0.0f;

    limitSwitchesClear();

    _ros_position[k]=0.0f;
    _ros_speed[k]=0.0f;
    _ros_effort[k] = 0.0f;

    _pidPosition[k] = new PID(&_innerTimer, &_position(k), &_positionCtrlOut(k), &_positionD_filtered(k), _platform_kpPosition(k), _platform_kiPosition(k), _platform_kdPosition(k), DIRECT, POS_PID_FILTER_GAINS[k]);
    _pidPosition[k]->setMode(AUTOMATIC);
    _pidSpeed[k] = new PID(&_innerTimer, &_speed(k), &_speedCtrlOut(k), &_speedD(k), _platform_kpSpeed(k), _platform_kiSpeed(k), _platform_kdSpeed(k),DIRECT, VEL_PID_FILTER_GAINS[k]);
    _pidSpeed[k]->setMode(AUTOMATIC);

    _flagInWsConstrains[k] = false;

  }

  _speedFilters[Y].setAlpha(0.96);
  _speedFilters[X].setAlpha(0.96);
  _speedFilters[PITCH].setAlpha(0.96);
  _speedFilters[ROLL].setAlpha(0.97);
  _speedFilters[YAW].setAlpha(0.97);

  _accFilters[Y].setAlpha(0.96);
  _accFilters[X].setAlpha(0.96);
  _accFilters[PITCH].setAlpha(0.96);
  _accFilters[ROLL].setAlpha(0.97);
  _accFilters[YAW].setAlpha(0.97);

  _innerCounterADC=0;

  _ros_controlledAxis = -1; //! all of them
  _platform_controlledAxis = _ros_controlledAxis;
  _ros_controllerType=TORQUE_CTRL;
  _platform_controllerType=_ros_controllerType;
  _flagClearLastState=false;
  _flagControllerTypeChanged=false;
  _flagDefaultCtrlNew = false;
  _flagCtrlGainsNew = false;

  _ros_flagDefaultControl=true;
  _platform_flagDefaultControl = true;

  for(int c = 0; c<NB_FI_CATEGORY; c++)
  {
    _flagInputReceived[c] = false; //! To be used specially for the telemanipulation state
  }

  for (uint j=0; j<NB_EFFORT_COMPONENTS; j++) // {NORMAL*, CONSTRAINS*, COMPENSATION, FEEDFORWARD}
  {
    if (j<=1){ _ros_effortComp[j]=1;}
    else{_ros_effortComp[j]=0;}

    _platform_effortComp[j] = _ros_effortComp[j];
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

  /*******DESIGNATIONS OF PINS IN THE MICROCONTROLLER NUCLEO L476RG */

  _csPins[Y] = PB_10;  //! CS2  -> Dorsi/Plantar Flexion NOT as PWMX/XN
  _csPins[X] = PA_10;  //! CS1 -> Lateral NOT as PWMX/XN
  _csPins[PITCH] = PA_9; //! CS3 -> Flexion/Extension of the Leg NOT as PWM/XN
  _csPins[ROLL] = PB_6;  //! CS4 -> roll and yaw encoder 1 NOT as PWMX/XN
  _csPins[YAW] = PB_2;  //! CS5 -> roll and yaw encoder 2  NOT as PWMX/X
  
  _motorPins[Y] = PB_3; // D3  PWM2/2
  _motorPins[X] = PC_7_ALT0; // D9 PWM8/2
  _motorPins[PITCH] = PB_5; // D4 PWM3/2 
  _motorPins[ROLL] = PA_8; // D7 PWM1.0f/1
  _motorPins[YAW] = PB_4; // D5  PWM3/1

  _limitSwitchesPins[Y] = PC_5; // NOT as PWMX/XN
  _limitSwitchesPins[X] = PC_8; //  NOT as PWMX/XN
  _limitSwitchesPins[PITCH] = PC_6; // NOT as PWMX/XN
  
  _esconEnabledPins[X] = PC_3;
  _esconEnabledPins[Y] = PH_1;
  _esconEnabledPins[PITCH] = PC_2;
  _esconEnabledPins[ROLL] = PH_0;
  _esconEnabledPins[YAW] = PC_13;


  _motorCurrentsPins[X]=PC_0;
  _motorCurrentsPins[Y]=PB_0;
  _motorCurrentsPins[PITCH]=PC_1;
  _motorCurrentsPins[ROLL]=PA_4;  
  _motorCurrentsPins[YAW]=PA_0;

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
  }



  for (int lim_=L_MIN; lim_<NB_LIMS; lim_++)
  {
    _compTorqueLims[lim_].setConstant(0.0f);
    _compTorqueLims[lim_].col(COMP_GRAVITY) = Eigen::Map<const Eigen::MatrixXf>(GRAVITY_EFFORT_LIMS[lim_],NB_AXIS,1);
    _compTorqueLims[lim_].col(COMP_VISC_FRICTION) = Eigen::Map<const Eigen::MatrixXf>(VISC_EFFORT_LIMS[lim_],NB_AXIS,1);
    _compTorqueLims[lim_].col(COMP_INERTIA) = Eigen::Map<const Eigen::MatrixXf>(INERTIA_EFFORT_LIMS[lim_],NB_AXIS,1);
    _compTorqueLims[lim_].col(COMP_CORIOLIS) = Eigen::Map<const Eigen::MatrixXf>(CORIOLIS_EFFORT_LIMS[lim_],NB_AXIS,1);;

  }


  //! Model

  _jointsViscosityGains = Eigen::Map<const Eigen::MatrixXf>(VISCOUS_K, NB_AXIS, 1);
  _massLinks = Eigen::Map<const Eigen::MatrixXf>(LINKS_MASS, NB_LINKS, 1);
  for (int link = 0; link < NB_LINKS; link++) {
    _momentInertiaLinks[link] = Eigen::Map<const Eigen::Matrix<float,3,3,Eigen::RowMajor>>(LINKS_MOMENT_OF_INERTIAS[link]);

    _linkCOMGeomJacobian[link].setConstant(0.0f);
    _linkCOMGeometricJ_prev[link].setConstant(0.0f);
    _rotationMatrixCOM[link].setConstant(0.0f);
    _rotationMatrixCOM_prev[link].setConstant(0.0f);
    _devLinkCOMGeomJacobian[link].setConstant(0.0f);
    _devRotationMatrixCOM[link].setConstant(0.0f);
  }

    _flagSpeedSampledForCoriolis = false;

    _flagContact=false;
    _flagVibration=false;
    _feedForwardTorque.setConstant(0.0f);

}