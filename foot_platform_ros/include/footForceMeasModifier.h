#ifndef __footForceMeasModifier_H__
#define __footForceMeasModifier_H__

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


#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <custom_msgs/FootInputMsg_v3.h>
#include <custom_msgs/FootOutputMsg_v2.h>
#include <custom_msgs/setControllerSrv.h>
#include <custom_msgs/setStateSrv_v2.h>
#include "../../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_pid.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_security.h"
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

using namespace std;

class footForceMeasModifier {

public:
  enum Platform_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2 };

private:
  int8_t _ros_platform_id;

  Platform_Name _platform_id;

  Eigen::Matrix<double, NB_AXIS, 1> _platform_position;
  Eigen::Matrix<double, NB_AXIS, 1> _platform_velocity;
  Eigen::Matrix<double, NB_AXIS, 1> _platform_effort;

  Eigen::Matrix<double, NB_AXIS, 1> _legTorquesGravityComp;
  Eigen::Matrix<double, NB_CART_AXIS, 1> _legCogWrtPlatfomBase;
  Eigen::Matrix<double, NB_AXIS_WRENCH, 1> _forceBias;
  Eigen::Matrix<double, NB_AXIS_WRENCH, 1> _legWrenchGravityComp;


  // KDL variables

  KDL::JntArray _platformJoints;
  KDL::JntArray _platformJointsInit;
  KDL::JntArray _platformJointLims[NB_LIMS];
  KDL::JntArray _gravityTorques;
  KDL::ChainJntToJacSolver* _myJacobianSolver;
  KDL::Jacobian _myFootBaseJacobian;
  KDL::Tree _myTree;
  KDL::Vector _grav_vector;
  std::vector<KDL::Segment> _mySegments;
  std::vector<KDL::Frame> _myFrames; //!
  KDL::ChainDynParam* _myChainDyn;
  KDL::Chain _myFootRestChain;
  KDL::Chain _myVirtualAnkleChain;


  KDL::ChainFkSolverPos_recursive* _myFKSolver;

  // ros variables

  geometry_msgs::WrenchStamped _msgForceBias; //! intented for the node FootVariableSynchronizer
  custom_msgs::FootInputMsg_v3 _msgLegGravCompFI; //! intented for the node FootVariableSynchronizer

  // ros variables
  urdf::Model _myModel;

  ros::NodeHandle _n;
  ros::Rate _loopRate;

  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations

  // Publisher declaration
  ros::Publisher _pubForceBias;
  ros::Publisher _pubLegCompFootInput;
  ros::Subscriber _subPlatformOutput; // FootOutputMsg_v2
  ros::Subscriber _subLegGravityComp;
  ros::Subscriber _subLegCoG;

  //! boolean variables
  bool _flagPlatformConnected;
  bool _flagLegGravityCompWrenchRead;
  bool _flagFootOutputRead;
  bool _stop;

  std::mutex _mutex;
  static footForceMeasModifier *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  footForceMeasModifier(ros::NodeHandle &n_1, double frequency,
                     footForceMeasModifier::Platform_Name platform_id_,
                     urdf::Model model_);

  ~footForceMeasModifier();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void readLegGravityComp(const geometry_msgs::WrenchStampedConstPtr &msg);
  void readLegCoG(const geometry_msgs::PointStampedConstPtr &msg);
  void readPlatformOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr &msg);
  void updateTreeFKState();
  void computeGravityTorque(); //! effort in each leg joint
  void computePedalBias();
  void computeWrenchFromPedalMeasBias();
  void computeLegGravityCompTorque();
  void publishForceBias(); // to be read by the foot variables synchronizer
  void publishLegCompFootInput(); // to be read by the foot variables synchronizer

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __footForceMeasModifier_H__