#ifndef PLATFORM_H
#define PLATFORM_H

#include "ros.h"
#include "mbed.h"
#include "SPI.h"
#include "QEC_1X_SPI.h"
#include "LP_Filter.h"
#include "MA_Filter.h"
#include "Platform.h"
#include "definitions.h"
#include "FootInputMsg_v3.h"
#include "FootOutputMsg_v2.h"
#include "setControllerSrv.h"
#include "setStateSrv.h"
#include "definitions.h"

#include "PID_v1.h"
#include "/home/lsrob107772/.platformio/lib/Eigen_ID3522/Dense.h"
//#include "/home/jacob/.platformio/lib/Eigen_ID3522/Dense.h"


class Platform
{
  public:
    
    // ROS variables
    ros::NodeHandle _nh;
    //char _logMsg[256];
    bool _stop;

    //Power Electronics Variables
    PinName _esconEnabledPins[NB_AXIS];
    InterruptIn* _esconEnabled[NB_AXIS];
    volatile unsigned int _allEsconOk;
    bool _recoveringFromError;
    bool _flagBiasADCOk;
    float _adc_sum[NB_AXIS];
    DigitalOut* _enableMotors;

    //Public Time
    uint32_t _timestamp;
    uint32_t _timestep;
    uint32_t _analogReadStamp;
    uint32_t _posSamplingStamp;
    uint32_t _speedSamplingStamp;
    uint32_t _accSamplingStamp;
    uint32_t _vibGenStamp;
    Timer _innerTimer; //! micros()
    uint64_t _innerCounterADC;

  private:
    Mutex _platformMutex;

  public:
    enum WrenchAxis { FX, FY, FZ, TX, TY, TZ, NB_AXIS_WRENCH };

    // ROS variables  

      ros::Subscriber<custom_msgs::FootInputMsg_v3>*  _subFootInput;
      ros::Publisher *_pubFootOutput;
      custom_msgs::FootOutputMsg_v2 _msgFootOutput;
      ros::ServiceServer<custom_msgs::setStateSrvRequest,custom_msgs::setStateSrvResponse> *_servChangeState;
      ros::ServiceServer<custom_msgs::setControllerSrvRequest,custom_msgs::setControllerSrvResponse> *_servChangeCtrl;

      //CLIENT VARIABLES FROM (ROS)
        volatile float _ros_position[NB_AXIS];
        volatile float _ros_speed[NB_AXIS];
        volatile float _ros_effort[NB_AXIS];
        volatile float _ros_forceSensor[NB_AXIS_WRENCH];

        volatile bool _ros_flagDefaultControl;
        volatile int8_t _ros_controlledAxis;
        volatile Controller _ros_controllerType;
        volatile uint8_t _ros_effortComp[NB_EFFORT_COMPONENTS];
        volatile State _ros_state;
        


    // State variables
    State _platform_state;
    int8_t _platform_controlledAxis;
    uint8_t _platform_effortComp[NB_EFFORT_COMPONENTS];
    Controller _platform_controllerType;

    volatile bool _flagClearLastState;
    volatile bool _flagControllerTypeChanged;
     bool _flagDefaultCtrlNew;
    volatile bool _flagCtrlGainsNew;
    volatile bool _flagInputReceived[NB_FI_CATEGORY];
    volatile bool _flagEmergencyCalled;
    bool _enterStateOnceFlag[NB_MACHINE_STATES];

    bool _workspaceLimitReached[NB_AXIS];
    bool _platform_flagDefaultControl;

    Eigen::Matrix<float, NB_AXIS, 1> _positionOffsets; //! in m or radians
    Eigen::Matrix<float, NB_AXIS, 1> _positionD;
    Eigen::Matrix<float, NB_AXIS, 1> _virtualWall;
    Eigen::Matrix<float, NB_AXIS, 1> _positionD_filtered;
    Eigen::Matrix<float, NB_AXIS, 1> _positionCtrlOut;
    Eigen::Matrix<float, NB_AXIS, 1> _position;
    Eigen::Matrix<float, NB_AXIS, 1> _positionPrev;
    Eigen::Matrix<float,NB_AXIS,1> _speed;
    Eigen::Matrix<float,NB_AXIS,1> _speedPrev;
    Eigen::Matrix<float, NB_AXIS, 1> _acceleration;
    Eigen::Matrix<float, NB_AXIS, 1> _speedD;
    Eigen::Matrix<float, NB_AXIS, 1> _speedCtrlOut;
    Eigen::Matrix<float, NB_AXIS, 1> _effortD;
    Eigen::Matrix<float, NB_AXIS, 1> _effortM; // The last two elements are temporary variables
    Eigen::Matrix<float, NB_AXIS, NB_EFFORT_COMPONENTS> _effortD_ADD;
    LP_Filter _posDesiredFilters[NB_AXIS];
    LP_Filter _speedFilters[NB_AXIS];
    LP_Filter _accFilters[NB_AXIS];
    LP_Filter _effortMFilters[NB_AXIS];
    volatile uint _switchesState[NB_AXIS];
    bool _flagInWsConstrains[NB_AXIS];

    // Hardware variables
    PinName _csPins[NB_AXIS];
    QEC_1X* _encoders[NB_AXIS];
    PinName _motorPins[NB_AXIS];
    PwmOut* _motors[NB_AXIS];
    PinName _limitSwitchesPins[NB_AXIS];
    PinName _motorCurrentsPins[NB_AXIS];
    AnalogIn* _motorCurrents[NB_AXIS];

    InterruptIn* _limitSwitches[NB_AXIS]; 
    SPI* _spi;

    // PID variables
      //General Variables
    Matrix<float,NB_AXIS,1> _platform_kpPosition;
    Matrix<float,NB_AXIS,1> _platform_kiPosition;
    Matrix<float,NB_AXIS,1> _platform_kdPosition;
    Matrix<float,NB_AXIS,1> _platform_kpSpeed;
    Matrix<float,NB_AXIS,1> _platform_kiSpeed;
    Matrix<float,NB_AXIS,1> _platform_kdSpeed;

    volatile float _ros_kpPosition[NB_AXIS];
    volatile float _ros_kiPosition[NB_AXIS];
    volatile float _ros_kdPosition[NB_AXIS];
    volatile float _ros_kpSpeed[NB_AXIS];
    volatile float _ros_kiSpeed[NB_AXIS];
    volatile float _ros_kdSpeed[NB_AXIS];

    // PID 

    PID* _pidPosition[NB_AXIS];
    PID* _pidSpeed[NB_AXIS];

    // Other variables
    
    static Platform *me;
    

    uint32_t _toc;
    bool _tic; //flag for timer

//*********************************************LIST-OF-METHODS**********
  //!Platform_main.cpp
  public:
    Platform(); //! 1
    ~Platform();//! 2
    void init();//! 3
    void step();//! 4
  private:
      //! GPIO Interruptions
    static void switchCallbackX();      //! 5
    static void switchCallbackY();      //! 6
    static void switchCallbackPitch();  //! 7
    //! ROS verification
    bool flagPositionInControl();
    bool flagSpeedInControl();
    bool flagTorqueInControl();
    //! Platform_reset.cpp
  private:
      void resetEscons(); //! 1
      void softReset();   //!2

  //! Platform_ros.cpp
  public:
    void communicateToRos();                                              //! 1
  private:
  //ROS
    static void updateFootInput(const custom_msgs::FootInputMsg_v3 &msg); //! 2
    static void updateState(const custom_msgs::setStateSrv::Request 
    &req,custom_msgs::setStateSrv::Response &resp );                      //! 3
    static void updateController(const custom_msgs::setControllerSrv::Request 
    &req,custom_msgs::setControllerSrv::Response &resp );                 //! 4
    void pubFootOutput();                                                 //! 5
  private:
    void updatePlatformFromRos();
    void calculateMeasTorques();
  

  //!Platform_effort.cpp
  public:  
    void setEfforts();                                                     //! 1
  private:
    // Effort Computation for the ESCONS
      void setEffortAxis(float effort, int axis);   //! 2
      void setCurrentAxis(float torque,int axis);   //! 3

  //! Platform_utils.cpp
  public:  
    float map(float x, float in_min, float in_max, float out_min, float out_max); //! 1    
    float clip(float x, float out_min, float out_max);
    float smoothRise(float x, float a, float b);
    float smoothFall(float x, float a, float b);
    float smoothRiseFall(float x, float a, float b, float c, float d);
    Eigen::Matrix<float, Eigen::Dynamic,  Eigen::Dynamic> boundMat(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> minLimit,Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> maxLimit);
    Eigen::Matrix<float, NB_AXIS*NB_AXIS, NB_AXIS> kroneckerProductEye(Eigen::Matrix<float, NB_AXIS, 1> xVector);
        
        //!Platform_sensors.cpp
        public : void getMotion(); //! 1
  private:
      //! Robot State
      void getPosition();                     //! 2
      void getSpeed();                    //! 3
      void getAcceleration();
      //! Estimate Robot Effort (ADC)    
      void readActualEffort();            //! 4

  //! Platform_emergency.cpp
  public:
    static void emergencyCallback();  //! 1
    void releasePlatform();           //! 2

  //! Platform_security.cpp

  private:
    void workspaceCheck(int axis_);

  //! Platform_control.cpp
  private:
      // Position and Speed control
      void positionAllControl(EffortComp Component);                             //! 1
      void positionAxisControl(EffortComp Component, int axis);                //! 2
      void speedAxisControl(EffortComp Component, int axis);              //! 3
      void speedAllControl(EffortComp Component);                            //! 4
      void gotoPointAxis(int axis_, float point);                         //! 5
      void gotoPointAll(float pointX, float pointY, float pointPITCH,     
      float pointROLL, float pointYAW);                                   //! 6
      void gotoPointGainsDefault();                              //! 7
      void speedPIDGainsDefault();                                 //!
      void posCtrlLimitsSet();                                    //!
      void speedCtrlLimitsSet();                                 //!
      void posInterpolator(int axis);
      void loadDefaultPIDGains();
      void loadROSPIDGains();
      void setPIDGains();
      //! Platform_compensation.cpp

          private : void dynamicCompensation(const int *components_);
      void gravityCompensation();
      void dryFrictionCompensation();
      void viscFrictionCompensation();
      void inertiaCompensation();
      void coriolisCompensation();
      void forceSensorCompensation();
      void quadraticRegression();

      Eigen::Matrix<float, NB_STICTION_AXIS, 1>
          _dryFrictionEffortSign[NB_SIGN_COMP];
      Eigen::Matrix<float, NB_AXIS, NB_COMPENSATION_COMP> _compensationEffort;
      Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_AXIS>
          _predictors[NB_SIGN_COMP];

      Eigen::Matrix<float, NB_AXIS, NB_COMPENSATION_COMP - 1>
          _compTorqueLims[NB_LIMS];
      Eigen::Matrix<float, 6, NB_AXIS> _linkCOMGeomJacobian[NB_LINKS];
      Eigen::Matrix<float, 6, NB_AXIS> _linkCOMGeometricJ_prev[NB_LINKS];
      Eigen::Matrix<float, 3, 3> _rotationMatrixCOM[NB_LINKS];
      Eigen::Matrix<float, 3, 3> _rotationMatrixCOM_prev[NB_LINKS];

      Eigen::Matrix<float, 6, NB_AXIS> _devLinkCOMGeomJacobian[NB_LINKS];
      Eigen::Matrix<float, 3, 3> _devRotationMatrixCOM[NB_LINKS];
      volatile bool _flagSpeedSampledForCoriolis;

      // Platform_constrains.cpp
      void wsConstrains(int axis_); //! -1 := all of them                 //! 1
      void wsConstrainsGainsDefault(); //! 3

      //! Platform_clear.cpp
    private:
      //! Maintenance
      void limitSwitchesClear();
      void positionAllReset();
      void positionCtrlClear(int axis_); //! Put gains and set point to zero of the Position Control
      void speedCtrlClear(int axis_); //! Put gains and set point to zero of the Speed Control
      void totalEffortDClear(int axis_);
      void compEffortClear(int axis_, EffortComp comp_);
      void clearLastState();
      void resetControllers();

    //! Platform_model.cpp
  private:
    bool _flagCalculateSinCos;
    float _c_theta;
    float _c_phi;
    float _c_psi;
    float _s_theta;
    float _s_phi;
    float _s_psi;

    Eigen::Matrix<float,NB_LINKS,1> _massLinks;
    Eigen::Matrix<float, NB_CART_AXIS, NB_CART_AXIS> _momentInertiaLinks[NB_LINKS];

    Eigen::Matrix<float, NB_AXIS, 1> _jointsViscosityGains;

    Eigen::Vector3f positionFrame(frame_chain frame); //! Based on off-line DH forward kinematics
    Eigen::Matrix3f rotationMatrix(frame_chain frame);
    Eigen::Matrix<float,6,NB_AXIS> geometricJacobian(frame_chain frame);
    Eigen::Vector3f comLinkWRTBase(link_chain link);
    Eigen::Matrix3f comRotationMatrix(link_chain link);
    Eigen::Matrix<float, 6, NB_AXIS> comGeometricJacobian(link_chain link);
#if (CORILIS_DEV_STRATEGY==CORIOLIS_KRONECKER)
    Eigen::Matrix<float, 6, NB_AXIS*NB_AXIS> devQComGeomJacobian(link_chain link);
    Eigen::Matrix<float, NB_CART_AXIS, NB_CART_AXIS * NB_AXIS>
    devQComRotationMatrix(link_chain link);
#endif

    Eigen::Matrix4f dhTransform(float r,float d,float alpha,float beta);


    //! Platform_feedforward.cpp

    void feedForwardControl();
    Eigen::Matrix<float, NB_AXIS, NB_FF_COMP> _feedForwardTorque;
    void eventVibration(frame_chain frame);
    bool _flagVibration;
    bool _flagContact;
};




#endif //PLATFORM_H

/* TODO

 * 3. Include the PID gains in the parameter server
 * 5. Make multi-axis PID

*/