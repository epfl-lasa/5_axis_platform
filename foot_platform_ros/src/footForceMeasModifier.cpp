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
  _torquesModified.setZero();
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
    ROS_ERROR("[%s force sensor]: Failed to construct kdl tree",Platform_Names[_platform_id]);
    _stop=true;
  }

  _myTree.getChain(std::string(Platform_Names[_platform_id]) + "_platform_base_link", std::string(Platform_Names[_platform_id]) + "_platform_foot_rest", _myFootRestChain);


  _myTree.getChain(std::string(Platform_Names[_platform_id]) + "_platform_base_link", std::string(Platform_Names[_platform_id]) + "_platform_virtual_ankle", _myVirtualAnkleChain);

  _myChainDyn = new KDL::ChainDynParam(_myFootRestChain, _grav_vector);

  _myFKSolver = new KDL::ChainFkSolverPos_recursive(_myFootRestChain);

  _myJacobianSolver = new KDL::ChainJntToJacSolver(_myFootRestChain);

  _mySegments = _myFootRestChain.segments;
  

  for (int joint_=0; joint_<NB_PLATFORM_AXIS; joint_++ )
   {
     _platformJointLims[L_MIN].data(joint_) = _myModel.getJoint(std::string(Platform_Names[_platform_id]) + "_" + std::string(Platform_Axis_Names[joint_]))->limits->lower;
     _platformJointLims[L_MAX].data(joint_) = _myModel.getJoint(std::string(Platform_Names[_platform_id]) + "_" + std::string(Platform_Axis_Names[joint_]))->limits->upper;
   }

}

footForceMeasModifier::~footForceMeasModifier() { me->_n.shutdown(); }

bool footForceMeasModifier::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  _pubForceModified = _n.advertise<geometry_msgs::WrenchStamped>("force_modified", 1);
  _pubPedalBias = _n.advertise<geometry_msgs::WrenchStamped>("pedal_bias_force", 1);
  _pubTorquesModified = _n.advertise<custom_msgs::FootOutputMsg>("torques_modified", 1);
  _pubForceSensorCoG = _n.advertise<geometry_msgs::PointStamped>("/" + std::string(Platform_Names[_platform_id]) +"/force_sensor_cog" , 1);
  _pubLegCompFootInput = _n.advertise<custom_msgs::FootInputMsg>("leg_comp_platform_effort", 1);
  _subForceSensor = _n.subscribe<geometry_msgs::WrenchStamped>(
					    	"/ft_"+std::string(Platform_Names[_platform_id])+"/rokubimini/ft_"+std::string(Platform_Names[_platform_id])+"/ft_sensor_readings/wrench/", 1,boost::bind(&footForceMeasModifier::readForceSensor, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _pubForceFootRestWorld = _n.advertise<geometry_msgs::WrenchStamped>("force_foot_rest_world", 1);
  _pubManipEllipsoidRot = _n.advertise<visualization_msgs::Marker>("foot_manipulability_rot", 0);
  _pubManipEllipsoidLin = _n.advertise<visualization_msgs::Marker>("foot_manipulability_lin", 0);

  if (_platform_id == LEFT) {

    _subLegCoG = _n.subscribe<geometry_msgs::PointStamped>("/left_leg/leg_joint_publisher/leg_cog", 1, boost::bind(&footForceMeasModifier::readLegCoG, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityComp = _n.subscribe<geometry_msgs::WrenchStamped>("/left_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footForceMeasModifier::readLegGravityComp, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg>(
        PLATFORM_PUBLISHER_NAME_LEFT, 1,
        boost::bind(&footForceMeasModifier::readPlatformOutput, this, _1),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    

  }
  if (_platform_id == RIGHT) {

    _subLegCoG = _n.subscribe<geometry_msgs::PointStamped>("/right_leg/leg_joint_publisher/leg_cog", 1, boost::bind(&footForceMeasModifier::readLegCoG, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityComp = _n.subscribe<geometry_msgs::WrenchStamped>("/right_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footForceMeasModifier::readLegGravityComp, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg>(
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
		{ 
      ROS_ERROR("[%s force sensor]: No force filter gain found",Platform_Names[_platform_id]);
    }
	
  _force_filt_alphas.setConstant(_force_filt_alpha);

  // Subscriber definitions
  signal(SIGINT, footForceMeasModifier::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[%s force sensor]: The platform joint state publisher is about to start ",Platform_Names[_platform_id]);
    return true;
  } 
  else {
    ROS_ERROR("[%s force sensor]: The ros node has a problem.",Platform_Names[_platform_id]);
    return false;
  }
}

void footForceMeasModifier::stopNode(int sig) { me->_stop = true; }

void footForceMeasModifier::run() {
  while (!_stop) {
    if (_flagPlatformConnected) {
      if (_flagFootOutputRead){
        updateTreeFKState();
        computeGravityTorque();
        computeWrenchFromPedalMeasBias();
      if (_flagLegGravityCompWrenchRead){
        computeLegGravityCompTorque();
        publishLegCompFootInput();
      }
      if (_flagForceConnected){ 
        filterForce();
        if (_ros_platform_machineState!=EMERGENCY && _ros_platform_machineState!=CENTERING){

          if (!_flagForceCalibrated){
            calibrateForce();
          }else {
            modifyForce();
            publishForceModified();
            publishForceFootRestWorld();
            publishTorquesModified();
            publishPedalBias();
					  }
        
        } else{
            ROS_INFO_ONCE("[%s force sensor]: Please put the platform in state TELEOPERATION.",Platform_Names[_platform_id]);
          }
				}
        }
    }else {

      ROS_INFO_ONCE("[%s force sensor]: The platform is not connected yet",Platform_Names[_platform_id]);
    }  
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[%s force sensor]: The force sensor modifier stopped",Platform_Names[_platform_id]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}


void footForceMeasModifier::readPlatformOutput(const custom_msgs::FootOutputMsg::ConstPtr &msg) {  
  _ros_platform_machineState =msg->platform_machineState;
  _ros_platform_id = msg->platform_id;
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _platform_position(k) = msg->platform_position[rosAxis[k]] * conversion_factor[rosAxis[k]];
    _platform_velocity(k) = msg->platform_speed[rosAxis[k]] * conversion_factor[rosAxis[k]];
    _platform_effort(k) = msg->platform_effortD[rosAxis[k]];
  }
  me->_platformJoints.data = _platform_position;
  if (!_flagPlatformConnected) {
    ROS_INFO("[%s force sensor]: Platform Connected",Platform_Names[_platform_id]);
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
    // cout<<_myFrames[i].p.data[0]<<" "<<_myFrames[i].p.data[1]<<" "<<_myFrames[i].p.data[2]<<endl;
  }
  _footBaseFrame = _myFrames[_myFootRestChain.getNrOfSegments()-1];
  _myJacobianSolver->JntToJac(_platformJoints,_myFootBaseJacobian,_myFootRestChain.getNrOfSegments() - 1);
  computeFootManipulability();
   publishManipulabilityEllipsoidRot();
   publishManipulabilityEllipsoidLin();
}

void footForceMeasModifier::computeWrenchFromPedalMeasBias()
{
  Eigen::Vector3d cogPedal_wrt_FS, weightPedal_wrt_FS;
  cogPedal_wrt_FS.setZero();
  weightPedal_wrt_FS.setZero();
  unsigned int framefSensor = _myFootRestChain.getNrOfSegments() - 2; //"frame fSensor"
  //cout<<_mySegments[framefSensor+1].getName().c_str()<<endl;  
  
  tf::vectorKDLToEigen (_myFrames[framefSensor].M.Inverse() * _grav_vector * _mySegments[framefSensor].getInertia().getMass() ,weightPedal_wrt_FS);
  tf::vectorKDLToEigen (_mySegments[framefSensor].getInertia().getCOG(),cogPedal_wrt_FS);

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
  _legTorquesGravityComp_prev = _legTorquesGravityComp;
  _legTorquesGravityComp =  (1-ALPHA_LEG_COMP)*(_myFootBaseJacobian.data.transpose() * _legWrenchGravityComp) + ALPHA_LEG_COMP*_legTorquesGravityComp_prev;
  
}


void footForceMeasModifier::publishForceModified() {
  
    _msgForceModified.header.stamp = ros::Time::now();
    _msgForceModified.header.frame_id = _platform_id ==RIGHT ? "/right_platform_fSensor" : "/left_platform_fSensor";
    tf::wrenchEigenToMsg(_forceModified,_msgForceModified.wrench);
    _pubForceModified.publish(_msgForceModified);
}

void footForceMeasModifier::publishPedalBias(){
  _msgPedalBias.header.stamp = ros::Time::now();
  _msgPedalBias.header.frame_id = _platform_id ==RIGHT ? "/right_platform_fSensor" : "/left_platform_fSensor";
  tf::wrenchEigenToMsg(_forcePedalBias,_msgPedalBias.wrench);
  _pubPedalBias.publish(_msgPedalBias);
}

void footForceMeasModifier::publishLegCompFootInput()
{
  // cout<<"boo"<<endl;
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
		ROS_INFO("[%s force sensor]: Force Sensor Connected",Platform_Names[_platform_id]);
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
		  ROS_INFO("[%s force sensor]: Sensor Calibrated!",Platform_Names[_platform_id]);
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
//     destination_frame = "/right_"+frame_destination_;
//     original_frame = "/right_"+frame_origin_;
//   }


//   try {
//     poseTransform_ = _tfBuffer.lookupTransform(
//         original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
//     kdlPoseTransform_ = tf2::transformToKDL(poseTransform_);
//     if (! _flagTFConnected)
//     {    _flagTFConnected = true;    }
    
//   } catch (tf2::TransformException ex) {
//     if (count>2)
//     { ROS_ERROR("[%s force sensor]: %s", ex.what(),Platform_Names[_platform_id]);
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

  if (_flagForceCalibrated)
	{	

  Eigen::Matrix<double, NB_AXIS_WRENCH,NB_AXIS_WRENCH> wrenchRotation;
  wrenchRotation.setIdentity();
  Eigen::Vector3d forceRotated_, momentRotated_;
  forceRotated_.setZero();
  momentRotated_.setZero();

  unsigned int framefSensor = _myFootRestChain.getNrOfSegments() - 2; //"fSensor"
  // cout<<_mySegments[framefSensor].getName().c_str()<<endl;  
  
  Eigen::Quaterniond wrenchQuaternion;
  wrenchQuaternion.setIdentity();
  tf::quaternionKDLToEigen(_myFrames[framefSensor].M, wrenchQuaternion);

  wrenchRotation.block(0,0,3,3) = wrenchQuaternion.normalized().toRotationMatrix(); 
  wrenchRotation.block(3,3,3,3) = wrenchQuaternion.normalized().toRotationMatrix();

  Eigen::Matrix<double,NB_AXIS_WRENCH,1> wrenchRotated;
  wrenchRotated.setZero();
  wrenchRotated = wrenchRotation*(_forceModified - _forcePedalBias);
  Eigen::Vector3d distanceFSToFootRest; 
  distanceFSToFootRest.setZero();
    
  tf::vectorKDLToEigen(_myFrames[framefSensor+1].p - _myFrames[framefSensor].p,distanceFSToFootRest);
  
  forceRotated_ = wrenchRotated.segment(0,3);
  momentRotated_ =  wrenchRotated.segment(3,3);

  _forceInFootRest.segment(0,3) = forceRotated_;

  _forceInFootRest.segment(3,3) = momentRotated_ +  distanceFSToFootRest.cross(forceRotated_);

  _msgForceFootRestWorld.header.stamp = ros::Time::now();
  _msgForceFootRestWorld.header.frame_id = _platform_id ==RIGHT ? "/right_platform_foot_rest" : "/left_platform_foot_rest";
  tf::wrenchEigenToMsg(_forceInFootRest,_msgForceFootRestWorld.wrench);
  _pubForceFootRestWorld.publish(_msgForceFootRestWorld);
  }

}

void footForceMeasModifier::publishTorquesModified(){
  
  if (_flagForceCalibrated)
  {
    _msgTorquesModified.platform_stamp=ros::Time::now();
    _msgTorquesModified.platform_effortM.fill(0.0);
    _torquesModified = _myFootBaseJacobian.data.transpose() * _forceInFootRest;
    // _torquesModified(p_pitch)*=-1;
    // _torquesModified(p_pitch)*=-1;
    for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
    {
      _msgTorquesModified.platform_effortM[i]=_torquesModified(i);
    }
    _pubTorquesModified.publish(_msgTorquesModified);
  }

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
		frame_name = _platform_id==RIGHT ? "/right_platform_fSensor" : "/left_platform_fSensor"; 
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


void footForceMeasModifier::computeFootManipulability()
{
  Eigen::Matrix<double, NB_AXIS_WRENCH, NB_PLATFORM_AXIS> jacobian_ = _myFootBaseJacobian.data;
  _mySVD.compute(jacobian_, ComputeThinU | ComputeThinV);
  Eigen::VectorXd mySingValues = _mySVD.singularValues();
  //cout<<"Singular Values :"<<endl<<_mySVD.singularValues()<<endl;
  // cout << "Foot Platform Condition Number: "<<mySingValues.minCoeff()/mySingValues.maxCoeff() << endl;
  // cout << "Foot Platform Yoshikawa Manipulability: "<<mySingValues.prod()<< endl;
}


void footForceMeasModifier::publishManipulabilityEllipsoidRot() {
  // _mutex.lock();
  VectorXd svdValues = _mySVD.singularValues();
  Matrix<double, 3,3> leftSVDVectors = _mySVD.matrixU().block(3,2,3,3);
  Quaternion<double> svdSingQuaternion(leftSVDVectors);
  std::string frame_name;
  std::string ns_name;
  frame_name = _platform_id == RIGHT ? "/right_platform_base_link" : "/left_platform_base_link";
  ns_name = _platform_id == RIGHT ? "/right" : "/left";
  _msgManipEllipsoidRot.header.frame_id = frame_name;
  _msgManipEllipsoidRot.header.stamp = ros::Time::now();
  _msgManipEllipsoidRot.ns = ns_name;
  _msgManipEllipsoidRot.id = 0;
  _msgManipEllipsoidRot.type = visualization_msgs::Marker::SPHERE;
  _msgManipEllipsoidRot.action = visualization_msgs::Marker::ADD;
  _msgManipEllipsoidRot.pose.position.x = _footBaseFrame.p.x();
  _msgManipEllipsoidRot.pose.position.y = _footBaseFrame.p.y();
  _msgManipEllipsoidRot.pose.position.z = _footBaseFrame.p.z();
  _msgManipEllipsoidRot.pose.orientation.x = svdSingQuaternion.x();
  _msgManipEllipsoidRot.pose.orientation.y = svdSingQuaternion.y();
  _msgManipEllipsoidRot.pose.orientation.z = svdSingQuaternion.z();
  _msgManipEllipsoidRot.pose.orientation.w = svdSingQuaternion.w();
  _msgManipEllipsoidRot.scale.x = svdValues(2) * 1;
  _msgManipEllipsoidRot.scale.y = svdValues(3) * 1;
  _msgManipEllipsoidRot.scale.z = svdValues(4) * 1;
  _msgManipEllipsoidRot.color.a = 0.5; // Don't forget to set the alpha!
  _msgManipEllipsoidRot.color.r = 0;
  _msgManipEllipsoidRot.color.g = 1;
  _msgManipEllipsoidRot.color.b = 0;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  _pubManipEllipsoidRot.publish(_msgManipEllipsoidRot);
  // _mutex.unlock();
}

void footForceMeasModifier::publishManipulabilityEllipsoidLin() {
   // _mutex.lock();
  VectorXd svdValues = _mySVD.singularValues();
  Matrix<double, 3,3> leftSVDVectors = _mySVD.matrixU().block(0,0,3,3);
  Quaternion<double> svdSingQuaternion(leftSVDVectors);
  std::string frame_name;
  std::string ns_name;
  frame_name = _platform_id == RIGHT ? "/right_platform_base_link" : "/left_platform_base_link";
  ns_name = _platform_id == RIGHT ? "/right" : "/left";
  _msgManipEllipsoidLin.header.frame_id = frame_name;
  _msgManipEllipsoidLin.header.stamp = ros::Time::now();
  _msgManipEllipsoidLin.ns = ns_name;
  _msgManipEllipsoidLin.id = 0;
  _msgManipEllipsoidLin.type = visualization_msgs::Marker::SPHERE;
  _msgManipEllipsoidLin.action = visualization_msgs::Marker::ADD;
  _msgManipEllipsoidLin.pose.position.x = _footBaseFrame.p.x();
  _msgManipEllipsoidLin.pose.position.y = _footBaseFrame.p.y();
  _msgManipEllipsoidLin.pose.position.z = _footBaseFrame.p.z();
  _msgManipEllipsoidLin.pose.orientation.x = svdSingQuaternion.x();
  _msgManipEllipsoidLin.pose.orientation.y = svdSingQuaternion.y();
  _msgManipEllipsoidLin.pose.orientation.z = svdSingQuaternion.z();
  _msgManipEllipsoidLin.pose.orientation.w = svdSingQuaternion.w();
  _msgManipEllipsoidLin.scale.x = svdValues(0);
  _msgManipEllipsoidLin.scale.y = svdValues(1);
  _msgManipEllipsoidLin.scale.z = svdValues(2);
  _msgManipEllipsoidLin.color.a = 0.5; // Don't forget to set the alpha!
  _msgManipEllipsoidLin.color.r = 0;
  _msgManipEllipsoidLin.color.g = 1;
  _msgManipEllipsoidLin.color.b = 1;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  _pubManipEllipsoidLin.publish(_msgManipEllipsoidLin);
  // _mutex.unlock();
}