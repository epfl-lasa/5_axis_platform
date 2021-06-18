#ifndef __platformHapticController_H__
#define __platformHapticController_H__

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/articulatedbodyinertia.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kdl.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <tf2_kdl/tf2_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <custom_msgs/FootInputMsg.h>
#include <custom_msgs/FootOutputMsg.h>
#include <custom_msgs/setControllerSrv.h>
#include <custom_msgs/setStateSrv.h>
#include "../../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>
#include <legRobot.h>
#include <vector>
#include <torque2TaskSpace_wdls.h>
//#include <smoothSignals.h>
#include <vibrator.h>
#include <Utils_math.h>
#include "LP_Filterd.h"
#include "HP_Filterd.h"
#include <custom_msgs/FootHapticDataMsg.h>

using namespace std;
using namespace Eigen;

class footHapticController {

public:
  enum FEET_ID{NO_FOOT_ID=0, RIGHT_FOOT_ID=1, LEFT_FOOT_ID=2};
  FEET_ID _feetID[NB_PLATFORMS];
private:
  
  unsigned int _nFoot;
  // float _minGainFoot[NB_PLATFORMS];
  // float _minGainLeg[NB_PLATFORMS];
  //LP_Filterd _minGainLegFilter[NB_PLATFORMS];
  LP_Filterd _lpFilterInPlatformHapticEfforts[NB_PLATFORMS][NB_PLATFORM_AXIS];
  HP_Filterd<double> _hpFilterInPlatformHapticEfforts[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _mixingCoefficients[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _vibDecayRateMinMax[NB_LIMS];
  double _vibFreqMinMax[NB_LIMS];
  bool _vibrationOn;
  vibrator<double>* _vibFBGenerator[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _vibFB[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _vibFreq[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _vibDecayRate[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _vibMagnitude[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _vibImpactVel[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _velGain[NB_PLATFORMS][NB_PLATFORM_AXIS];
  double _maxGainForAllJoints;
  double _minJND;
  double _maxVelocity[NB_PLATFORM_AXIS];
  
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _effortGainRaw[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _effortGain[NB_PLATFORMS];

  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  KDL::Frame _toolTipRobotTransform;
  
  Eigen::Matrix<double, NB_AXIS_WRENCH,1> _legToPlatformGravityWrench[NB_PLATFORMS];
  Eigen::Matrix<double, NB_AXIS_WRENCH,1> _platformToLegGravityWrench[NB_PLATFORMS];
  Eigen::Matrix<double, NB_AXIS_WRENCH,1> _platformToLegImpedanceWrench[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _legToPlatformGravityEfforts[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _maxPossibleGains[NB_PLATFORMS]; 

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platform_position[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platform_velocity[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platform_effort[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platform_humanEffort[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _weberRatios[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _outPlatformHapticEffortsMax[NB_PLATFORMS];
  
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _platformToLegGravityTorques[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _platformToLegImpedanceTorques[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _leg_position[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _leg_velocity[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _leg_effort[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _weberRatiosLeg[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _weberLegCoeff[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _userDefinedJND[NB_PLATFORMS];

  Eigen::Matrix<double, NB_AXIS_WRENCH, 1> _inPlatformHapticWrench[NB_PLATFORMS];
  KDL::JntArray _inPlatformHapticEffLPFFull[NB_PLATFORMS];
  KDL::JntArray _inPlatformHapticEffLPFProj[NB_PLATFORMS];
  KDL::JntArray _inPlatformHapticEffLPF[NB_PLATFORMS];
  KDL::JntArray _inPlatformHapticEffHPF[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _devInPlatformHapticEfforts[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _prevInPlatformHapticEfforts[NB_PLATFORMS];

  Eigen::Matrix<double, NB_LEG_AXIS, 1> _inPlatformToLegHapticEfforts[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _normalizedEffortCoeffsInLeg[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _jointLimitGaussianFilterCoeff[NB_PLATFORMS];
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _outPlatformToLegHapticEffortsMax[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _outPlatformHapticEfforts[NB_PLATFORMS];

  Eigen::Matrix<double, NB_AXIS_WRENCH, NB_AXIS_WRENCH> _rotationfSensor[NB_PLATFORMS];

  KDL::JntArray _platformJoints[NB_PLATFORMS];
  KDL::JntArray _platformJointLims[NB_LIMS][NB_PLATFORMS];
  
  KDL::JntArray _legJoints[NB_PLATFORMS];
  KDL::JntArray _legJointLims[NB_LIMS][NB_PLATFORMS];

  KDL::JntArray _legGravityTorques[NB_PLATFORMS];

  KDL::Vector _grav_vector;

  Eigen::JacobiSVD<MatrixXd> _svdFootJacobian[NB_PLATFORMS];
  KDL::ChainJntToJacSolver* _platformJacobianSolver[NB_PLATFORMS];
  KDL::Jacobian _platformFootRestJacobian[NB_PLATFORMS];
  KDL::Tree _platformTree[NB_PLATFORMS];
  std::vector<KDL::Segment> _platformSegments[NB_PLATFORMS];
  std::vector<KDL::Frame> _platformFrames[NB_PLATFORMS]; 
  KDL::Frame _platformFootRestFrame[NB_PLATFORMS];
  KDL::ChainDynParam* _platformChainDyn[NB_PLATFORMS];
  KDL::Chain _platformFootRestChain[NB_PLATFORMS];
  KDL::ChainFkSolverPos_recursive* _platformFKSolver[NB_PLATFORMS];
  KDL::Torque2TaskSpace_wdls* _platformFDSolver[NB_PLATFORMS];
  
  Eigen::JacobiSVD<MatrixXd> _svdlegJacobian[NB_PLATFORMS];
  KDL::ChainJntToJacSolver* _legJacobianSolver[NB_PLATFORMS];
  KDL::Jacobian _legFootBaseJacobian[NB_PLATFORMS];
  KDL::Tree _legTree[NB_PLATFORMS];
  std::vector<KDL::Segment> _legSegments[NB_PLATFORMS];
  std::vector<KDL::Frame> _legFrames[NB_PLATFORMS];
  KDL::Frame _legFootBaseFrame[NB_PLATFORMS];
  KDL::ChainDynParam* _legChainDyn[NB_PLATFORMS];
  KDL::Chain _legFootBaseChain[NB_PLATFORMS];
  KDL::ChainFkSolverPos_recursive* _legFKSolver[NB_PLATFORMS];
  KDL::Torque2TaskSpace_wdls* _legFDSolver[NB_PLATFORMS];

  // ros variables
  sensor_msgs::JointState _inMsgLegJointState[NB_PLATFORMS];
  // sensor_msgs::JointState _inMsgPlatformJointState[NB_PLATFORMS];
  

  custom_msgs::FootOutputMsg _inMsgPlatformOutput[NB_PLATFORMS];   
  custom_msgs::FootInputMsg _inMsgDesiredHapticEfforts[NB_PLATFORMS];
  custom_msgs::FootInputMsg _outMsgHapticEfforts[NB_PLATFORMS];
  custom_msgs::FootHapticDataMsg _outMsgHapticData[NB_PLATFORMS];

  // ros variables
  urdf::Model _platformModel[NB_PLATFORMS];
  urdf::Model _legModel[NB_PLATFORMS];
  ros::NodeHandle _n;
  ros::Rate _loopRate;

  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations

  // Publisher declaration
  ros::Publisher _pubHapticEfforts[NB_PLATFORMS];
  ros::Publisher _pubHapticData[NB_PLATFORMS];

  // ros::Subscriber _subPlatformJointState[NB_PLATFORMS];
  ros::Subscriber _subPlatformOutput[NB_PLATFORMS];
  ros::Subscriber _subLegJointState[NB_PLATFORMS];
  
  ros::Subscriber _subDesiredHapticEfforts[NB_PLATFORMS];
  //! boolean variables

  bool _flagLegJointStateRead[NB_PLATFORMS];
  bool _flagPlatformJointStateRead[NB_PLATFORMS];
  bool _flagDesiredHapticRead[NB_PLATFORMS];



  bool _stop;
  std::mutex _mutex;
  static footHapticController *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  footHapticController(const ros::NodeHandle &n_1, const float &frequency, FEET_ID* feet_id);
  ~footHapticController();

  bool init();
  void run();

private:
  //! ROS METHODS

  
  void loadModels(int whichFoot);
  void processLegJoints(int whichFoot);
  void readLegJointState(const sensor_msgs::JointState::ConstPtr &msg, int whichFoot);
  void updateLegTreeFKState(int whichFoot);
  
  void processPlatformJoints(int whichFoot);
  // void readPlatformJointState(const sensor_msgs::JointState::ConstPtr &msg, int whichFoot);
  void readPlatformOutput(const custom_msgs::FootOutputMsg::ConstPtr &msg, int whichFoot);
  void updatePlatformTreeFKState(int whichFoot);

  void computeLegGravityTorques(int whichFoot);
  void processDesiredHapticEfforts(int whichFoot); 
  void readDesiredHapticEfforts(const custom_msgs::FootInputMsg::ConstPtr &msg, int whichFoot);
  
  
  void publishHapticEfforts(int whichFoot); 
  void publishHapticData(int whichFoot); 
  void doHapticControl();

  void readToolTipRobotBase(int whichFoot);
  
  //! OTHER METHODS
  static void stopNode(int sig);
};

#endif // __platformHapticController_H__