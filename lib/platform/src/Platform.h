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
#include "custom_msgs/FootInputMsg.h"
#include "custom_msgs/FootOutputMsg.h"
#include "custom_msgs/setControllerSrv.h"
#include "custom_msgs/setStateSrv.h"
#include "definitions.h"

#include "PID_v1.h"
#include <Core>
#include <Dense>


class Platform
{
  public:
    
    // ROS variables
    ros::NodeHandle _nh;
    //char _logMsg[256];
    bool _stop;

    //Power Electronics Variables
    PinName _esconEnabledPins[NB_PLATFORM_AXIS];
    InterruptIn* _esconEnabled[NB_PLATFORM_AXIS];
    volatile unsigned int _allEsconOk;
    bool _recoveringFromError;
    bool _flagBiasADCOk;
    float _adc_sum[NB_PLATFORM_AXIS];
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

  public:
    Mutex _platformMutex;

  public:

    // ROS variables  

      ros::Subscriber<custom_msgs::FootInputMsg>*  _subFootInput;
      ros::Publisher *_pubFootOutput;
      custom_msgs::FootOutputMsg _msgFootOutput;
      custom_msgs::FootInputMsg _msgFootInput;
      custom_msgs::setStateSrvRequest _reqSrvState;
      custom_msgs::setControllerSrvRequest _reqSrvController;
      ros::ServiceServer<custom_msgs::setStateSrvRequest,custom_msgs::setStateSrvResponse> *_servChangeState;
      ros::ServiceServer<custom_msgs::setControllerSrvRequest,custom_msgs::setControllerSrvResponse> *_servChangeCtrl;

      //CLIENT VARIABLES FROM (ROS)
        volatile float _ros_forceSensor[NB_AXIS_WRENCH];

        volatile bool _flagLoadParams;
        volatile bool _flagControllerRequest;
        volatile bool _flagStateRequest;

        // State variables
        State _platform_state;
        int8_t _platform_controlledAxis;
        uint8_t _platform_effortComp[NB_EFFORT_COMPONENTS];
        Controller _platform_controllerType;

        bool _flagControllerTypeChanged;
        bool _flagDefaultCtrlNew;
        volatile bool _flagEmergencyCalled;
        volatile bool _flagRosInputReceived;
        bool _enterStateOnceFlag[NB_MACHINE_STATES];

        bool _workspaceLimitReached[NB_PLATFORM_AXIS];
        bool _platform_flagDefaultControl;
        bool _flagRosConnected;

        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _positionOffsets; //! in m or radians
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _positionD_filtered;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _positionCtrlOut;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _position;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _positionD;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _softLimitsCtrlOut;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _softLimitsMin;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _softLimitsMax;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _softLimitsD;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _positionPrev;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _speed;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _speedPrev;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _acceleration;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _speedD;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _speedCtrlOut;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _forceSensorCtrlOut;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _forceSensorD;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _effortD;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _effortM;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _effortMNEG;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, NB_EFFORT_COMPONENTS> _effortD_ADD;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_filterAxisFS;
        bool _flagOutofSoftLimitsControl;
        LP_Filter _posDesiredFilters[NB_PLATFORM_AXIS];
        LP_Filter _speedFilters[NB_PLATFORM_AXIS];
        LP_Filter _accFilters[NB_PLATFORM_AXIS];
        LP_Filter _effortMFilters[NB_PLATFORM_AXIS];
        volatile uint8_t _switchesState[NB_PLATFORM_AXIS];
        bool _flagInWsConstrains[NB_PLATFORM_AXIS];


        // Hardware variables
        PinName _csPins[NB_PLATFORM_AXIS];
        QEC_1X *_encoders[NB_PLATFORM_AXIS];
        PinName _motorPins[NB_PLATFORM_AXIS];
        PwmOut *_motors[NB_PLATFORM_AXIS];
        PinName _limitSwitchesPins[NB_PLATFORM_AXIS];
        PinName _motorCurrentsPins[NB_PLATFORM_AXIS];
        AnalogIn *_motorCurrents[NB_PLATFORM_AXIS];

        InterruptIn *_limitSwitches[NB_PLATFORM_AXIS];
        SPI *_spi;

        // PID variables
        // General Variables
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kpPosition;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kiPosition;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kdPosition;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kpSpeed;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kiSpeed;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kdSpeed;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kpFS;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kiFS;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kdFS;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kpSoftLimits;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kiSoftLimits;
        Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _platform_kdSoftLimits;
        
        Eigen::Matrix<float, NB_AXIS, 1> _maxCtrlEfforts;

        float _rosParam_kpPosition[NB_PLATFORM_AXIS];
        float _rosParam_kiPosition[NB_PLATFORM_AXIS];
        float _rosParam_kdPosition[NB_PLATFORM_AXIS];
        float _rosParam_kpSpeed[NB_PLATFORM_AXIS];
        float _rosParam_kiSpeed[NB_PLATFORM_AXIS];
        float _rosParam_kdSpeed[NB_PLATFORM_AXIS];
        float _rosParam_kpFS[NB_PLATFORM_AXIS];
        float _rosParam_kiFS[NB_PLATFORM_AXIS];
        float _rosParam_kdFS[NB_PLATFORM_AXIS];
        float _rosParam_pointSoftLimitsMin[NB_PLATFORM_AXIS];
        float _rosParam_pointSoftLimitsMax[NB_PLATFORM_AXIS];
        float _rosParam_kpSoftLimits[NB_PLATFORM_AXIS];
        float _rosParam_kiSoftLimits[NB_PLATFORM_AXIS];
        float _rosParam_kdSoftLimits[NB_PLATFORM_AXIS];
        int _rosParam_compensation[NB_COMPENSATION_COMP];

        // PID

        PID *_pidPosition[NB_PLATFORM_AXIS];
        PID *_pidSpeed[NB_PLATFORM_AXIS];
        PID *_pidForceSensor[NB_PLATFORM_AXIS];
        PID *_pidSoftLimits[NB_PLATFORM_AXIS];

        // Other variables

        static Platform *me;

        uint32_t _toc;
        bool _tic; // flag for timer

        //*********************************************LIST-OF-METHODS**********
        //! Platform_main.cpp
      public:
        Platform();  //! 1
        ~Platform(); //! 2
        void init(); //! 3
        void step(); //! 4
      private:
        //! GPIO Interruptions
        static void switchCallbackX();     //! 5
        static void switchCallbackY();     //! 6
        static void switchCallbackPitch(); //! 7
        //! ROS verification
        bool flagPositionInControl();
        bool flagSpeedInControl();
        bool flagTorqueInControl();
        //! Platform_reset.cpp
      private:
        void resetEscons(); //! 1
        void softReset();   //! 2

        //! Platform_ros.cpp
      public:
        void communicateToRos(); //! 1
        void retrieveParams(Param_Category category_);
        bool waitUntilRosConnect();

      private:
        // ROS
        static void
        updateFootInput(const custom_msgs::FootInputMsg &msg); //! 2
        static void updateState(const custom_msgs::setStateSrv::Request &req,
                                custom_msgs::setStateSrv::Response &resp); //! 3
        static void
        updateController(const custom_msgs::setControllerSrv::Request &req,
                         custom_msgs::setControllerSrv::Response &resp); //! 4
        void pubFootOutput();                                            //! 5
        void updateControllerRequest();
        void updateStateInfo();
      private:
        void updateFootInputFromRos();
        void updatePlatformFromRos();
        void parseROSMessage();
        void calculateMeasTorques();

        //! Platform_effort.cpp
      public:
        void setEfforts(); //! 1
      private:
        // Effort Computation for the ESCONS
        void setEffortAxis(float effort, int axis);  //! 2
        void setCurrentAxis(float torque, int axis); //! 3

        //! Platform_utils.cpp
      public:
        float map(float x, float in_min, float in_max, float out_min,
                  float out_max); //! 1
        float clip(float x, float out_min, float out_max);
        float smoothRise(float x, float a, float b);
        float smoothFall(float x, float a, float b);
        float smoothRiseFall(float x, float a, float b, float c, float d);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        boundMat(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x,
                 Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> minLimit,
                 Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> maxLimit);
        Eigen::Matrix<float, NB_PLATFORM_AXIS * NB_PLATFORM_AXIS, NB_PLATFORM_AXIS>
        kroneckerProductEye(Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> xVector);
        
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
      void positionAxisControl(EffortComp Component, int axis_);                //! 2
      void speedAxisControl(EffortComp Component, int axis_);              //! 3
      void controlPositionWS(EffortComp Component, int axis);                        //! 5
      void gotoPointGainsDefault();                              //! 7
      void speedPIDGainsDefault();                                 //!
      void forceSensorPIDGainsDefault();                                 //!
      void posCtrlLimitsSet();                                    //!
      void speedCtrlLimitsSet();                                 //!
      void forceSensorCtrlLimitsSet();                                 //!
      void softLimitsCtrlLimitsSet();                                 //!
      void posInterpolator(int axis);
      void loadDefaultPIDGains();
      void loadParamPIDGains();
      void loadParamCompensation();
      void loadROSPIDGains();
      void setPIDGains();
      
    //! Platform_compensation.cpp

    private :

      void dynamicCompensation();
      void gravityCompensation();
      void dryFrictionCompensation();
      void viscFrictionCompensation();
      void inertiaCompensation();
      void coriolisCompensation();
      void forceSensorCompensation();
      void quadraticRegression();      
      bool _flagOutofCompensation;
      int _platform_compensation[NB_COMPENSATION_COMP];

      Eigen::Matrix<float, NB_STICTION_AXIS, 1>
          _dryFrictionEffortSign[NB_SIGN_COMP];
      Eigen::Matrix<float, NB_PLATFORM_AXIS, NB_COMPENSATION_COMP> _compensationEffort;
      Eigen::Matrix<float, 2 * NB_PLATFORM_AXIS, NB_STICTION_AXIS>
          _predictors[NB_SIGN_COMP];

      Eigen::Matrix<float, NB_PLATFORM_AXIS, NB_COMPENSATION_COMP - 1> _compTorqueLims[NB_LIMS];
      Eigen::Matrix<float, 6, NB_PLATFORM_AXIS> _linkCOMGeomJacobian[NB_LINKS];
      Eigen::Matrix<float, 6, NB_PLATFORM_AXIS> _linkCOMGeometricJ_prev[NB_LINKS];
      Eigen::Matrix<float, 3, 3> _rotationMatrixCOM[NB_LINKS];
      Eigen::Matrix<float, 3, 3> _rotationMatrixCOM_prev[NB_LINKS];

      Eigen::Matrix<float, 6, NB_PLATFORM_AXIS> _devLinkCOMGeomJacobian[NB_LINKS];
      Eigen::Matrix<float, 3, 3> _devRotationMatrixCOM[NB_LINKS];
      volatile bool _flagSpeedSampledForCoriolis;

      // Platform_constrains.cpp
      void wsConstrains(int axis_); //! -1 := all of them                 //! 1

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
      void resetControllers(Controller controllerType);

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

    Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _jointsViscosityGains;

    Eigen::Vector3f positionFrame(frame_chain frame); //! Based on off-line DH forward kinematics
    Eigen::Matrix3f rotationMatrix(frame_chain frame);
    Eigen::Matrix<float,6,NB_PLATFORM_AXIS> geometricJacobian(frame_chain frame);
    Eigen::Vector3f comLinkWRTBase(link_chain link);
    Eigen::Matrix3f comRotationMatrix(link_chain link);
    Eigen::Matrix<float, 6, NB_PLATFORM_AXIS> comGeometricJacobian(link_chain link);
#if (CORILIS_DEV_STRATEGY==CORIOLIS_KRONECKER)
    Eigen::Matrix<float, 6, NB_PLATFORM_AXIS*NB_PLATFORM_AXIS> devQComGeomJacobian(link_chain link);
    Eigen::Matrix<float, NB_CART_AXIS, NB_CART_AXIS * NB_PLATFORM_AXIS>
    devQComRotationMatrix(link_chain link);
#endif

    Eigen::Matrix4f dhTransform(float r,float d,float alpha,float beta);


    //! Platform_feedforward.cpp

    void feedForwardControl();
    Eigen::Matrix<float, NB_PLATFORM_AXIS, NB_FF_COMP> _feedForwardTorque;
    void eventVibration(frame_chain frame);
    bool _flagVibration;
    bool _flagContact;
};




#endif //PLATFORM_H

/* TODO

 * 3. Include the PID gains in the parameter server
 * 5. Include model of the leg

*/