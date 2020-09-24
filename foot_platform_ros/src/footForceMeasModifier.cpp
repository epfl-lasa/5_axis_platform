#include "footForceMeasModifier.h"

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{
  PLATFORM_AXES};
#undef ListofPlatformAxes

char const *Platform_Names[]{"none", "right", "left"};

footForceMeasModifier *footForceMeasModifier::me = NULL;

footForceMeasModifier::footForceMeasModifier ( ros::NodeHandle &n_1, double frequency,
    footForceMeasModifier::Platform_Name platform_id, urdf::Model model_)
    : _n(n_1), _platform_id(platform_id), _loopRate(frequency),
      _dt(1.0f / frequency), _myModel(model_), _grav_vector(0.0, 0.0, (double)GRAVITY){
   me = this;
  _stop = false;
  _ros_platform_id = 0;
  _ros_platform_machineState = 0;
  _forcePedalBias.setZero();
  _forcePedalBiasInit.setZero();
  _platform_position.setZero();
  _platform_velocity.setZero();
  _platform_effort.setZero();
  _legCogWrtPlatfomBase.setZero();
  _legWrenchGravityComp.setZero();
  _legTorquesGravityComp.setZero();
  _legTorquesGravityComp_prev.setZero();
  _platformJoints.resize(NB_PLATFORM_AXIS);
  _gravityTorques.resize(NB_PLATFORM_AXIS);
  _platformJointsInit.resize(NB_PLATFORM_AXIS);
  _platformJointLims[L_MIN].resize(NB_PLATFORM_AXIS);
  _platformJointLims[L_MAX].resize(NB_PLATFORM_AXIS);
  _myFootBaseJacobian.resize(NB_PLATFORM_AXIS);
  _flagPlatformConnected=false;
  _flagLegGravityCompWrenchRead = false;
  _flagFootOutputRead = false;
  _forceMeasurements.setZero();
  
  _forceFiltered.setZero(); _forceFiltered_prev.setZero();
  
  _forceInFootRest.setZero();
  _forceModified.setZero();
  _rotationfSensor.setIdentity();
  _force_filt_alpha=0.5;
  _flagForceConnected = false;
  _force_filt_alphas.setConstant(_force_filt_alpha);
  
  _calibrationCount=0;
  _flagForceCalibrated=false;
  _undesiredForceBias.setZero();
  _forceSensorCoG.setZero();
  
  // _flagTFConnected = false;

  // _fSensorBaseTransform.M.Identity();
  // _fSensorBaseTransform.p.Zero();


  // _footRestBaseTransform.M.Identity(); 
  // _footRestBaseTransform.p.Zero(); 



  //  _tfListener = new tf2_ros::TransformListener(_tfBuffer);

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
  

  for (int joint_=0; joint_<NB_PLATFORM_AXIS; joint_++ )
   {
     _platformJointLims[L_MIN].data(joint_) = _myModel.getJoint(Platform_Axis_Names[joint_])->limits->lower;
     _platformJointLims[L_MAX].data(joint_) = _myModel.getJoint(Platform_Axis_Names[joint_])->limits->upper;
   }

}

footForceMeasModifier::~footForceMeasModifier() { me->_n.shutdown(); }

bool footForceMeasModifier::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  _pubForceBias = _n.advertise<geometry_msgs::WrenchStamped>("force_modified", 1);
  _pubForceSensorCoG = _n.advertise<geometry_msgs::PointStamped>("/" + std::string(Platform_Names[_platform_id]) +"/force_sensor_cog" , 1);
  _pubLegCompFootInput = _n.advertise<custom_msgs::FootInputMsg_v5>("leg_comp_platform_effort", 1);
  _subForceSensor = _n.subscribe<geometry_msgs::WrenchStamped>(
					    	"/"+std::string(Platform_Names[_platform_id])+"/rokubimini0/force/", 1,boost::bind(&footForceMeasModifier::readForceSensor, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _pubForceFootRestWorld = _n.advertise<geometry_msgs::WrenchStamped>("force_foot_rest_world", 1);


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


  if (_platform_id==LEFT)
	{
		cout<<"The rotation matrix for the left force sensor has been considered!"<<endl;
		Eigen::Matrix3d rotTemp;
		rotTemp<< cos(-M_PI), -sin(-M_PI), 0,
            sin(-M_PI),  cos(-M_PI), 0,
                0,           0, 1;
		_rotationfSensor.block(0,0,3,3) = rotTemp;
		_rotationfSensor.block(3,3,3,3) = rotTemp;
	}
	  

	if (!_n.getParam("force_alpha", _force_filt_alpha))
		{ ROS_ERROR("No force filter gain found"); }
	
  _force_filt_alphas.setConstant(_force_filt_alpha);

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
        }
        if (_flagLegGravityCompWrenchRead)
        {
          computeLegGravityCompTorque();
          publishLegCompFootInput();
        _flagLegGravityCompWrenchRead = false;
        }
        if (_flagForceConnected)
				{ 
          filterForce();
          if (_ros_platform_machineState!=EMERGENCY && _ros_platform_machineState!=CENTERING)
          {
            if (!_flagForceCalibrated)
            {
					    calibrateForce();
            }
					  else
					  {
					    modifyForce();
              publishForceModified();
              publishForceFootRestWorld();
					  }
          }
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
  _ros_platform_machineState =msg->platform_machineState;
  _ros_platform_id = msg->platform_id;
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
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
  _myFrames.clear(); 
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
    _myFKSolver->JntToCart(_platformJoints, frame_, i + 1);
    _myFrames.push_back(frame_);
  }
  _myJacobianSolver->JntToJac(_platformJoints,_myFootBaseJacobian,_myFootRestChain.getNrOfSegments() - 1);
}

void footForceMeasModifier::computeWrenchFromPedalMeasBias()
{
  Eigen::Vector3d cogPedal_wrt_FS, weightPedal_wrt_FS;
  unsigned int segmentPedal = _myFootRestChain.getNrOfSegments() - 2; //"fSensor"
  //cout<<_mySegments[segmentPedal+1].getName().c_str()<<endl;  
  
  tf::vectorKDLToEigen (_myFrames[segmentPedal+1].M.Inverse() * _grav_vector * _mySegments[segmentPedal].getInertia().getMass() ,weightPedal_wrt_FS);
  tf::vectorKDLToEigen (_mySegments[segmentPedal].getInertia().getCOG(),cogPedal_wrt_FS);

  _forcePedalBias.segment(0,3) = weightPedal_wrt_FS;
  _forcePedalBias.segment(3,3) = cogPedal_wrt_FS.cross(weightPedal_wrt_FS);
}

void footForceMeasModifier::readLegCoG(const geometry_msgs::PointStampedConstPtr &msg) {
  tf::pointMsgToEigen(msg->point, me->_legCogWrtPlatfomBase);
}

void footForceMeasModifier::readLegGravityComp(const geometry_msgs::WrenchStampedConstPtr &msg) {
  tf::wrenchMsgToEigen(msg->wrench, me->_legWrenchGravityComp);
  _flagLegGravityCompWrenchRead = true;
}


void footForceMeasModifier::computeLegGravityCompTorque() {
  //cout<<_myFootBaseJacobian.data<<endl;
  _legTorquesGravityComp =  (1-ALPHA_LEG_COMP)*(_myFootBaseJacobian.data.transpose() * _legWrenchGravityComp) + ALPHA_LEG_COMP*_legTorquesGravityComp_prev;
  _legTorquesGravityComp_prev = _legTorquesGravityComp;
  
}


void footForceMeasModifier::publishForceModified() {
  
    _msgForceModified.header.stamp = ros::Time::now();
    _msgForceModified.header.frame_id = _platform_id ==RIGHT ? "right/fSensor" : "left/fSensor";
    tf::wrenchEigenToMsg(_forceModified,_msgForceModified.wrench);
    _pubForceBias.publish(_msgForceModified);
}

void footForceMeasModifier::publishLegCompFootInput()
{
  for (unsigned int i = 0 ; i<NB_PLATFORM_AXIS; i++)
  {
    _msgLegGravCompFI.ros_effort[rosAxis[i]] = _legTorquesGravityComp(i) ;
  }
  _pubLegCompFootInput.publish(_msgLegGravCompFI);
}

void footForceMeasModifier::readForceSensor(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  
	_forceMeasurements(0) = msg->wrench.force.x;
	_forceMeasurements(1) = msg->wrench.force.y;
	_forceMeasurements(2) = msg->wrench.force.z;
	_forceMeasurements(3) = msg->wrench.torque.x;
	_forceMeasurements(4) = msg->wrench.torque.y;
	_forceMeasurements(5) = msg->wrench.torque.z;
  _forceMeasurements = _rotationfSensor * _forceMeasurements;
    if (!_flagForceConnected)
	 {
		ROS_INFO("Force Sensor Connected");
    _flagForceConnected = true;
	 }
}

void footForceMeasModifier::filterForce()
{
  _forceFiltered = _forceFiltered_prev.cwiseProduct(_force_filt_alphas) + (_forceMeasurements.array().cwiseProduct(1.0f - _force_filt_alphas.array())).matrix();
  _forceFiltered_prev = _forceFiltered;
}

void footForceMeasModifier::calibrateForce()
{
	if (!_flagForceCalibrated)
	{
		_undesiredForceBias+=_forceMeasurements;
		_calibrationCount++;
	}
	if (_calibrationCount==NB_CALIBRATION_COUNT)
	{
          _undesiredForceBias = (_undesiredForceBias.array()/NB_CALIBRATION_COUNT).matrix();
		  ROS_INFO("Sensor Calibrated!");
		  cout<<"Undesired force bias: "<<_undesiredForceBias.transpose()<<endl;
      _flagForceCalibrated = true;
      _forcePedalBiasInit = _forcePedalBias;
      _calibrationCount = 0;
	};
}


// KDL::Frame footForceMeasModifier::readTF(std::string frame_origin_, std::string frame_destination_)
// {
//   static int count = 0;
  
//   std::string original_frame;
//   std::string destination_frame;
//   geometry_msgs::TransformStamped poseTransform_;
//   KDL::Frame kdlPoseTransform_;

//   if (_platform_id==LEFT)
//   {
//     destination_frame = "left/"+frame_destination_;
//     original_frame = "left/"+frame_origin_;
//   }
//   else 
//   {
//     destination_frame = "right/"+frame_destination_;
//     original_frame = "right/"+frame_origin_;
//   }


//   try {
//     poseTransform_ = _tfBuffer.lookupTransform(
//         original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
//     kdlPoseTransform_ = tf2::transformToKDL(poseTransform_);
//     if (! _flagTFConnected)
//     {    _flagTFConnected = true;    }
    
//   } catch (tf2::TransformException ex) {
//     if (count>2)
//     { ROS_ERROR("%s", ex.what());
//       count = 0;
//     }
//     else
//     {
//       count++;
//     }
//     ros::Duration(1.0).sleep();
//   }
//   return kdlPoseTransform_;
// }

void footForceMeasModifier::publishForceFootRestWorld(){

  Eigen::Matrix<double, NB_AXIS_WRENCH,NB_AXIS_WRENCH> wrenchRotation;
  wrenchRotation.setIdentity();
  Eigen::Vector3d forceRotated_, momentRotated_;

  unsigned int framefSensor = _myFootRestChain.getNrOfSegments() - 1; //"fSensor"
  //cout<<_mySegments[framefSensor-1].getName().c_str()<<endl;  
  
  Eigen::Quaterniond wrenchQuaternion;
  wrenchQuaternion.Identity();
  tf::quaternionKDLToEigen(_myFrames[framefSensor].M, wrenchQuaternion);

  wrenchRotation.block(0,0,3,3) = wrenchQuaternion.matrix(); 
  wrenchRotation.block(3,3,3,3) = wrenchQuaternion.matrix();

  Eigen::Matrix<double,NB_AXIS_WRENCH,1> wrenchRotated = wrenchRotation*(_forceModified - _forcePedalBias);
  Eigen::Vector3d distanceFSToFootRest; 
   
  tf::vectorKDLToEigen(_myFrames[framefSensor+1].p - _myFrames[framefSensor].p,distanceFSToFootRest);
  
  forceRotated_ = wrenchRotated.segment(0,3);
  momentRotated_ =  wrenchRotated.segment(3,3);

  _forceInFootRest.segment(0,3) = forceRotated_;

  _forceInFootRest.segment(3,3) = momentRotated_ +  distanceFSToFootRest.cross(forceRotated_);

  _msgForceFootRestWorld.header.stamp = ros::Time::now();
  _msgForceFootRestWorld.header.frame_id = _platform_id ==RIGHT ? "right/foot_rest" : "left/foot_rest";
  tf::wrenchEigenToMsg(_forceInFootRest,_msgForceFootRestWorld.wrench);
  _pubForceFootRestWorld.publish(_msgForceFootRestWorld);

}


void footForceMeasModifier::publishForceSensorStaticCoG(){
	if (_flagForceCalibrated)
	{	
		Eigen::Matrix<double,3,1> force_, moment_;
		force_.setZero(); moment_.setZero();
		force_= _forceModified.segment(0,3);
		moment_= _forceModified.segment(3,3);
		
		if (force_.norm()>FLT_EPSILON)
		{
			_forceSensorCoG = force_.normalized().cross(moment_/force_.norm());
		}	
		
		std::string frame_name;
		frame_name = _platform_id==RIGHT ? "/right/fSensor" : "/left/fSensor"; 
		_msgForceSensorCoG.header.stamp = ros::Time::now();
		_msgForceSensorCoG.header.frame_id = frame_name; 
		_msgForceSensorCoG.point.x = _forceSensorCoG(0);
		_msgForceSensorCoG.point.y = _forceSensorCoG(1);
		_msgForceSensorCoG.point.z = _forceSensorCoG(2);
		_pubForceSensorCoG.publish(_msgForceSensorCoG);
	}
}

void footForceMeasModifier::modifyForce() {
    _forceModified = _forceFiltered - (_undesiredForceBias-_forcePedalBiasInit);
    //cout<<_forcePedalBiasInit<<endl;
}
