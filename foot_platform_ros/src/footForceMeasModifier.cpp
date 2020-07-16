#include "footForceMeasModifier.h"

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
  AXES};
#undef ListofAxes

char const *Platform_Names[]{"none", "right", "left"};

footForceMeasModifier *footForceMeasModifier::me = NULL;

footForceMeasModifier::footForceMeasModifier ( ros::NodeHandle &n_1, double frequency,
    footForceMeasModifier::Platform_Name platform_id, urdf::Model model_)
    : _n(n_1), _platform_id(platform_id), _loopRate(frequency),
      _dt(1.0f / frequency), _myModel(model_), _grav_vector(0.0, 0.0, (double)GRAVITY){
   me = this;
  _stop = false;
  _ros_platform_id = _platform_id;
  _forceBias.setZero();
  _platform_position.setZero();
  _platform_velocity.setZero();
  _platform_effort.setZero();
  _legCogWrtPlatfomBase.setZero();
  _legWrenchGravityComp.setZero();
  _legTorquesGravityComp.setZero();
  _platformJoints.resize(NB_AXIS);
  _gravityTorques.resize(NB_AXIS);
  _platformJointsInit.resize(NB_AXIS);
  _platformJointLims[L_MIN].resize(NB_AXIS);
  _platformJointLims[L_MAX].resize(NB_AXIS);
  _myFootBaseJacobian.resize(NB_AXIS);
  _flagPlatformConnected=false;
  _flagLegGravityCompWrenchRead = false;
  _flagFootOutputRead = false;

  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("Failed to construct kdl tree");
    _stop=true;
  }

  _myTree.getChain("platform_base_link", "foot_rest", _myFootRestChain);

  _myTree.getChain("platform_base_link", "virtual_ankle", _myVirtualAnkleChain);

  _myChainDyn = new KDL::ChainDynParam(_myFootRestChain, _grav_vector);

  _myFKSolver = new KDL::ChainFkSolverPos_recursive(_myFootRestChain);

  _myJacobianSolver = new KDL::ChainJntToJacSolver(_myFootRestChain);

  _mySegments = _myFootRestChain.segments;
  

  for (int joint_=0; joint_<NB_AXIS; joint_++ )
   {
     _platformJointLims[L_MIN].data(joint_) = _myModel.getJoint(Axis_names[joint_])->limits->lower;
     _platformJointLims[L_MAX].data(joint_) = _myModel.getJoint(Axis_names[joint_])->limits->upper;
   }

}

footForceMeasModifier::~footForceMeasModifier() { me->_n.shutdown(); }

bool footForceMeasModifier::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  _pubForceBias = _n.advertise<geometry_msgs::WrenchStamped>("force_modified", 1);
  _pubLegCompFootInput = _n.advertise<custom_msgs::FootInputMsg_v3>("leg_comp_platform_effort", 1);

  if (_platform_id == LEFT) {

    _subLegCoG = _n.subscribe<geometry_msgs::PointStamped>("/left/leg_joint_publisher/leg_cog", 1, boost::bind(&footForceMeasModifier::readLegCoG, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityComp = _n.subscribe<geometry_msgs::WrenchStamped>("left/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footForceMeasModifier::readLegGravityComp, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(
        PLATFORM_PUBLISHER_NAME_LEFT, 1,
        boost::bind(&footForceMeasModifier::readPlatformOutput, this, _1),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    

  }
  if (_platform_id == RIGHT) {

    _subLegCoG = _n.subscribe<geometry_msgs::PointStamped>("/right/leg_joint_publisher/leg_cog", 1, boost::bind(&footForceMeasModifier::readLegCoG, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityComp = _n.subscribe<geometry_msgs::WrenchStamped>("/right/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footForceMeasModifier::readLegGravityComp, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(
        PLATFORM_PUBLISHER_NAME_RIGHT, 1,
        boost::bind(&footForceMeasModifier::readPlatformOutput, this, _1),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  }

  // Subscriber definitions
  signal(SIGINT, footForceMeasModifier::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("The platform joint state publisher "
             "is about to start ");
    return true;
  } 
  else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void footForceMeasModifier::stopNode(int sig) { me->_stop = true; }

void footForceMeasModifier::run() {
  while (!_stop) {
    if (_flagPlatformConnected) {
      if ((_platform_id != (Platform_Name)_ros_platform_id) &&
          (_platform_id != UNKNOWN)) 
      {
        ROS_ERROR("This node  is acting on the "
                  "wrong platform");
        ros::spinOnce();
        break;
      } 
      else {
        if (_flagFootOutputRead)
        {
          updateTreeFKState();
          computeGravityTorque();
          computeWrenchFromPedalMeasBias();
          _flagFootOutputRead = false;
        }
        if (_flagLegGravityCompWrenchRead)
        {
          computeLegGravityCompTorque();
          publishForceBias();
          publishLegCompFootInput();
        _flagLegGravityCompWrenchRead = false;
        }

      }
    }
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("Platform state variables stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}


void footForceMeasModifier::readPlatformOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr &msg) {  
  _ros_platform_id = msg->platform_id;
  for (int k = 0; k < NB_AXIS; k++) {
    _platform_position(k) = msg->platform_position[rosAxis[k]] * conversion_factor[rosAxis[k]];
    _platform_velocity(k) = msg->platform_speed[rosAxis[k]] * conversion_factor[rosAxis[k]];
    _platform_effort(k) = msg->platform_effortD[rosAxis[k]];
  }
  me->_platformJoints.data = _platform_position;
  if (!_flagPlatformConnected) {
    _flagPlatformConnected = true;
  }
  _flagFootOutputRead = true;
}

void footForceMeasModifier::computeGravityTorque() {
  _myChainDyn->JntToGravity(_platformJoints, _gravityTorques);
  // cout<<_gravityTorques.data.transpose()<<endl;
}

void footForceMeasModifier::updateTreeFKState() {
  
  KDL::Frame frame_;
   
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
    _myFKSolver->JntToCart(_platformJoints, frame_, i + 1);
    _myFrames.push_back(frame_);
  }
  _myJacobianSolver->JntToJac(_platformJoints,_myFootBaseJacobian);
}

void footForceMeasModifier::computeWrenchFromPedalMeasBias()
{
  Eigen::Vector3d cogPedal_wrt_FS, weightPedal_wrt_FS;
  unsigned int segmentPedal = _myFootRestChain.getNrOfSegments() - 2; //"fSensor"
  //cout<<_mySegments[segmentPedal].getName().c_str()<<endl;  
  
  tf::vectorKDLToEigen (_myFrames[segmentPedal+1].M.Inverse() * _grav_vector * _mySegments[segmentPedal].getInertia().getMass() ,weightPedal_wrt_FS);
  tf::vectorKDLToEigen (_mySegments[segmentPedal].getInertia().getCOG(),cogPedal_wrt_FS);

  _forceBias.segment(0,3) = weightPedal_wrt_FS;
  _forceBias.segment(3,3) = cogPedal_wrt_FS.cross(weightPedal_wrt_FS);
}

void footForceMeasModifier::readLegCoG(const geometry_msgs::PointStampedConstPtr &msg) {
  tf::pointMsgToEigen(msg->point, me->_legCogWrtPlatfomBase);
}

void footForceMeasModifier::readLegGravityComp(const geometry_msgs::WrenchStampedConstPtr &msg) {
  tf::wrenchMsgToEigen(msg->wrench, me->_legWrenchGravityComp);
  _flagLegGravityCompWrenchRead = true;
}


void footForceMeasModifier::computeLegGravityCompTorque() {
  _legTorquesGravityComp = - _myFootBaseJacobian.data.transpose() * _legWrenchGravityComp;
}


void footForceMeasModifier::publishForceBias() {
  
    _msgForceBias.header.stamp = ros::Time::now();
    _msgForceBias.header.frame_id = _platform_id ==RIGHT ? "right/fSensor" : "left/fSensor";
    tf::wrenchEigenToMsg(_forceBias,_msgForceBias.wrench);
    _pubForceBias.publish(_msgForceBias);
}

void footForceMeasModifier::publishLegCompFootInput()
{
  for (unsigned int i = 0 ; i<NB_AXIS; i++)
  {
    _msgLegGravCompFI.ros_effort[i] = _legTorquesGravityComp(i);
  }
  _pubLegCompFootInput.publish(_msgLegGravCompFI);
}