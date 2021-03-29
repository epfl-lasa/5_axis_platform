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

using namespace std;
using namespace Eigen;

class footHapticController {

public:
  enum FEET_ID{NO_FOOT_ID=0, RIGHT_FOOT_ID=1, LEFT_FOOT_ID=2};
  std::vector<FEET_ID> _feetID;
  #define NB_FEET_MAX 2
private:
  unsigned int _nFoot;

  std::vector<Eigen::Matrix<double, NB_PLATFORM_AXIS, 1>> _platform_position;
  std::vector<Eigen::Matrix<double, NB_PLATFORM_AXIS, 1>> _platform_velocity;
  std::vector<Eigen::Matrix<double, NB_PLATFORM_AXIS, 1>> _platform_effort;
  
  std::vector<Eigen::Matrix<double, NB_LEG_AXIS, 1>> _leg_position;
  std::vector<Eigen::Matrix<double, NB_LEG_AXIS, 1>> _leg_velocity;
  std::vector<Eigen::Matrix<double, NB_LEG_AXIS, 1>> _leg_effort;

  std::vector<Eigen::Matrix<double, NB_AXIS_WRENCH, 1>> _estimatedGuidanceWrench;
  std::vector<Eigen::Matrix<double, NB_PLATFORM_AXIS, 1>> _desiredGuidanceEfforts;

  std::vector<KDL::JntArray> _platformJoints;
  std::vector<KDL::JntArray> _platformJointLims[NB_LIMS];
  std::vector<KDL::JntArray> _legJoints;
  std::vector<KDL::JntArray> _legJointLims[NB_LIMS];

  KDL::Vector _grav_vector;

  std::vector<Eigen::JacobiSVD<MatrixXd>> _svdFootJacobian;
  std::vector<KDL::ChainJntToJacSolver*> _platformJacobianSolver;
  std::vector<KDL::Jacobian> _platformFootBaseJacobian;
  std::vector<KDL::Tree> _platformTree;
  std::vector<std::vector<KDL::Segment>> _platformSegments;
  std::vector<std::vector<KDL::Frame>> _platformFrames; //!
  std::vector<KDL::ChainDynParam*> _platformChainDyn;
  std::vector<KDL::Chain> _platformFootRestChain;
  std::vector<KDL::ChainFkSolverPos_recursive*> _platformFKSolver;
  
  std::vector<Eigen::JacobiSVD<MatrixXd>> _svdlegJacobian;
  std::vector<KDL::ChainJntToJacSolver*> _legJacobianSolver;
  std::vector<KDL::Jacobian> _legFootBaseJacobian;
  std::vector<KDL::Tree> _legTree;
  std::vector<std::vector<KDL::Segment>> _legSegments;
  std::vector<std::vector<KDL::Frame>> _legFrames; //!
  std::vector<KDL::ChainDynParam*> _legChainDyn;
  std::vector<KDL::Chain> _legFootBaseChain;
  std::vector<KDL::ChainFkSolverPos_recursive*> _legFKSolver;

  // ros variables
  std::vector<sensor_msgs::JointState> _inMsgLegJoints;
  std::vector<sensor_msgs::JointState> _inMsgFootJointState;
  std::vector<custom_msgs::FootInputMsg> _inMsgDesiredGuidanceEfforts;
  std::vector<custom_msgs::FootOutputMsg> _outMsgHapticEfforts;

  // ros variables
  std::vector<urdf::Model> _platformModel;
  std::vector<urdf::Model> _legModel;
  ros::NodeHandle _n;
  ros::Rate _loopRate;

  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations

  // Publisher declaration
  std::vector<ros::Publisher> _pubHapticEfforts;

  std::vector<ros::Subscriber> _subPlatformJointState; // sensor_msgs/JointState
  std::vector<ros::Subscriber> _subLegJointState; //Reads the wrench needed for gravity compensation of the leg (in the foot base frame) to torques for the joints of the platform.
  
  std::vector<ros::Subscriber> _subDesiredGuidance;  // geometry_msgs/WrenchStamped.h
  //! boolean variables

  std::vector<volatile bool> _flagLegJointStateRead;
  std::vector<volatile bool> _flagPlatformJointStateRead;
  

  bool _stop;
  std::mutex _mutex;
  static footHapticController *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  footHapticController(ros::NodeHandle &n_1, double frequency, std::vector<FEET_ID> feet_id);
  ~footHapticController();

  bool init();
  void run();

private:
  //! ROS METHODS

  
  void loadModels(int whichFoot);
  void processLegJoints(int whichFoot);
  void readLegJoints(const sensor_msgs::JointState::ConstPtr &msg, int whichFoot);
  void updateLegTreeFKState(int whichFoot);
  
  void processPlatformJoints(int whichFoot);
  void readPlatformJoints(const sensor_msgs::JointState::ConstPtr &msg, int whichFoot);
  void updatePlatformTreeFKState(int whichFoot);

  void publishHapticEfforts(int whichFoot); 
  void publishForceFootRestWorld(int whichFoot);
  void computeFootManipulability(int whichFoot); 
  void computeLegManipulability(int whichFoot); 

  
  //! OTHER METHODS
  static void stopNode(int sig);
};

#endif // __platformHapticController_H__