#include <footHapticController.h>

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

const float AccFilter = 0.99f;
const int Axis_Ros[] = {1,0,2,3,4};

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{
  PLATFORM_AXES};
#undef ListofPlatformAxes

#define ListofLegAxes(enumeration, names) names,
char const *Leg_Axis_Names[]{LEG_AXES};
#undef ListofLegAxes

char const *Feet_Names[]{"none","right", "left"};

footHapticController *footHapticController::me = NULL;

footHapticController::footHapticController ( ros::NodeHandle &n_1, double frequency,
    std::vector<FEET_ID> feetID)
    : _n(n_1), _loopRate(frequency),
      _dt(1.0f / frequency), _grav_vector(0.0, 0.0, (double)GRAVITY), _feetID(feetID){
   me = this;
  _stop = false;

  _nFoot = _feetID.size(); 

  _platform_position.resize(_nFoot);
  _platform_velocity.resize(_nFoot);
  _platform_effort.resize(_nFoot); 
  _leg_position.resize(_nFoot);
  _leg_velocity.resize(_nFoot);
  _leg_effort.resize(_nFoot);
  _estimatedGuidanceWrench.resize(_nFoot);
  _desiredGuidanceEfforts.resize(_nFoot);

  _platformJoints.resize(_nFoot);
  _platformJointLims[L_MIN].resize(_nFoot);
  _platformJointLims[L_MAX].resize(_nFoot);
  

  _legJoints.resize(_nFoot);
  _legJointLims[L_MIN].resize(_nFoot);
  _legJointLims[L_MAX].resize(_nFoot);

  _platformFootBaseJacobian.resize(_nFoot);
  _legFootBaseJacobian.resize(_nFoot);
  
  _flagLegJointStateRead.resize(_nFoot);
  _flagPlatformJointStateRead.resize(_nFoot);

  _svdFootJacobian.resize(_nFoot);
  _platformJacobianSolver.resize(_nFoot);
  _platformFootBaseJacobian.resize(_nFoot);
  _platformTree.resize(_nFoot);
  _platformSegments.resize(_nFoot);
  _platformFrames.resize(_nFoot); 
  _platformChainDyn.resize(_nFoot);
  _platformFootRestChain.resize(_nFoot);
  _platformFKSolver.resize(_nFoot);
  
  _svdlegJacobian.resize(_nFoot);
  _legJacobianSolver.resize(_nFoot);
  _legFootBaseJacobian.resize(_nFoot);
  _legTree.resize(_nFoot);
  _legSegments.resize(_nFoot);
  _legFrames.resize(_nFoot); 
  _legChainDyn.resize(_nFoot);
  _legFootBaseChain.resize(_nFoot);
  _legFKSolver.resize(_nFoot);

  _inMsgLegJoints.resize(_nFoot);
  _inMsgFootJointState.resize(_nFoot);
  _inMsgDesiredGuidanceEfforts.resize(_nFoot);
  _outMsgHapticEfforts.resize(_nFoot);

  // ros variables
  _platformModel.resize(_nFoot);
  _legModel.resize(_nFoot);


  
  for (size_t i = 0; i < _nFoot; i++)
  {
    _platform_position[i].setZero();
    _platform_velocity[i].setZero();
    _platform_effort[i].setZero();

    _leg_position[i].setZero();
    _leg_velocity[i].setZero();
    _leg_effort[i].setZero();
    _estimatedGuidanceWrench[i].setZero();
    _desiredGuidanceEfforts[i].setZero();

    
    _platformJoints[i].resize(NB_PLATFORM_AXIS);
    _platformJoints[i].data.setZero();
    _platformJointLims[i][L_MIN].resize(NB_PLATFORM_AXIS);
    _platformJointLims[i][L_MAX].resize(NB_PLATFORM_AXIS);
    

    _legJoints[i].resize(NB_PLATFORM_AXIS);
    _legJoints[i].data.setZero();
    _legJointLims[i][L_MIN].resize(NB_PLATFORM_AXIS);
    _legJointLims[i][L_MAX].resize(NB_PLATFORM_AXIS);

    _platformFootBaseJacobian[i].resize(NB_PLATFORM_AXIS);
    _legFootBaseJacobian[i].resize(NB_LEG_AXIS);
    
    _flagLegJointStateRead[i] = false;
    _flagPlatformJointStateRead[i] = false;
  
  
    urdf::Model platformModelLoad;
    if (!platformModelLoad.initParam("/"+std::string(Feet_Names[_feetID[i]])+"_platform/robot_description")) 
    {
      ROS_ERROR("Failed to parse platform urdf file");
      _stop=true;
    }
    _platformModel.push_back(platformModelLoad);
    urdf::Model legModelLoad;
    if (!legModelLoad.initParam("/"+std::string(Feet_Names[_feetID[i]])+"_leg/robot_description")) {
      ROS_ERROR("[footHapticController: ] Failed to parse  one of the leg urdf file, please check");
      _stop=true;
    }
    _legModel.push_back(legModelLoad);
    ROS_INFO("[footHapticController: ] Successfully parsed %s leg+platform urdf files", std::string(Feet_Names[_feetID[i]]));
    
    KDL::Tree platformTreeLoad;
     if (!kdl_parser::treeFromUrdfModel(platformModelLoad, platformTreeLoad)) {
    ROS_ERROR("[%s force sensor]: Failed to construct kdl tree",Feet_Names[_feetID[i]]);
    _stop=true;
  }
  _platformTree.push_back(platformTreeLoad);
  _platformTree[i].getChain(std::string(Feet_Names[_feetID[i]]) + "_platform_base_link", std::string(Feet_Names[_feetID[i]]) + "_platform_foot_rest", _platformFootRestChain[i]);
  _legTree[i].getChain(std::string(Feet_Names[_feetID[i]]) + "_leg_hip_base_link", std::string(Feet_Names[_feetID[i]]) + "_leg_foot_base", _legFootBaseChain[i]);

  _platformChainDyn[i] = new KDL::ChainDynParam(_platformFootRestChain[i], _grav_vector);
  _legChainDyn[i] = new KDL::ChainDynParam(_legFootBaseChain[i], _grav_vector);
  
  
  _platformFKSolver[i] = new KDL::ChainFkSolverPos_recursive(_platformFootRestChain[i]);
  _legFKSolver[i] = new KDL::ChainFkSolverPos_recursive(_legFootBaseChain[i]);

  _platformJacobianSolver[i] = new KDL::ChainJntToJacSolver(_platformFootRestChain[i]);
  _legJacobianSolver[i] = new KDL::ChainJntToJacSolver(_legFootBaseChain[i]);

  _platformSegments[i] = _platformFootRestChain[i].segments;
  _legSegments[i] = _legFootBaseChain[i].segments;
  

  for (int joint_=0; joint_<NB_PLATFORM_AXIS; joint_++ )
   {
     _platformJointLims[i][L_MIN].data(joint_) = _platformModel[i].getJoint(std::string(Feet_Names[_feetID[i]]) + "_" + std::string(Platform_Axis_Names[joint_]))->limits->lower;
     _platformJointLims[i][L_MAX].data(joint_) = _platformModel[i].getJoint(std::string(Feet_Names[_feetID[i]]) + "_" + std::string(Platform_Axis_Names[joint_]))->limits->upper;
   }

  for (int joint_=0; joint_<NB_LEG_AXIS; joint_++ )
  {
    _legJointLims[i][L_MIN].data(joint_) = _legModel[i].getJoint(std::string(Feet_Names[_feetID[i]]) + "_leg_" + std::string(Leg_Axis_Names[joint_]))->limits->lower;
    _legJointLims[i][L_MAX].data(joint_) = _legModel[i].getJoint(std::string(Feet_Names[_feetID[i]]) + "_leg_" + std::string(Leg_Axis_Names[joint_]))->limits->upper;
  }
  
  }
}

footHapticController::~footHapticController() { me->_n.shutdown(); }

bool footHapticController::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  _pubForceModified = _n.advertise<geometry_msgs::WrenchStamped>("force_modified", 1);
  _pubPedalBias = _n.advertise<geometry_msgs::WrenchStamped>("pedal_bias_force", 1);
  _pubTorquesModified = _n.advertise<custom_msgs::FootOutputMsg>("torques_modified", 1);
  _pubForceSensorCoG = _n.advertise<geometry_msgs::PointStamped>("/" + std::string(Feet_Names[_platform_id]) +"/force_sensor_cog" , 1);
  _pubLegCompFootInput = _n.advertise<custom_msgs::FootInputMsg>("leg_comp_platform_effort", 1);
  _pubInertiaCoriolisFootInput = _n.advertise<custom_msgs::FootInputMsg>("foot_comp_inertia_coriolis", 1);
  _subForceSensor = _n.subscribe<geometry_msgs::WrenchStamped>(
					    	"/ft_"+std::string(Feet_Names[_platform_id])+"/rokubimini/ft_"+std::string(Feet_Names[_platform_id])+"/ft_sensor_readings/wrench/", 1,boost::bind(&footHapticController::readForceSensor, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _pubForceFootRestWorld = _n.advertise<geometry_msgs::WrenchStamped>("force_foot_rest_world", 1);
  _pubManipEllipsoidRot = _n.advertise<visualization_msgs::Marker>("foot_manipulability_rot", 0);
  _pubManipEllipsoidLin = _n.advertise<visualization_msgs::Marker>("foot_manipulability_lin", 0);

  if (_platform_id == LEFT) {

    _subLegCoG = _n.subscribe<geometry_msgs::PointStamped>("/left_leg/leg_joint_publisher/leg_cog", 1, boost::bind(&footHapticController::readLegCoG, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityComp = _n.subscribe<geometry_msgs::WrenchStamped>("/left_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footHapticController::readLegGravityComp, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg>(
        PLATFORM_PUBLISHER_NAME_LEFT, 1,
        boost::bind(&footHapticController::readPlatformOutput, this, _1),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    

  }
  if (_platform_id == RIGHT) {

    _subLegCoG = _n.subscribe<geometry_msgs::PointStamped>("/right_leg/leg_joint_publisher/leg_cog", 1, boost::bind(&footHapticController::readLegCoG, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityComp = _n.subscribe<geometry_msgs::WrenchStamped>("/right_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footHapticController::readLegGravityComp, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg>(
        PLATFORM_PUBLISHER_NAME_RIGHT, 1,
        boost::bind(&footHapticController::readPlatformOutput, this, _1),
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
      ROS_ERROR("[%s force sensor]: No force filter gain found",Feet_Names[_platform_id]);
    }
	
  _force_filt_alphas.setConstant(_force_filt_alpha);

  // Subscriber definitions
  signal(SIGINT, footHapticController::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[%s force sensor]: The platform joint state publisher is about to start ",Feet_Names[_platform_id]);
    return true;
  } 
  else {
    ROS_ERROR("[%s force sensor]: The ros node has a problem.",Feet_Names[_platform_id]);
    return false;
  }
}

void footHapticController::stopNode(int sig) { me->_stop = true; me->publishForceModified(); me->publishForceFootRestWorld();me->publishTorquesModified();me->publishPedalBias(); me->publishLegCompFootInput(); me->publishInertiaCoriolisFootInput();}

void footHapticController::run() {
  while (!_stop) {

    if(_subLegCoG.getNumPublishers()>0)
    {
      if(_flagLegCoGRead)
      {
        processLegCoG();
        _flagLegCoGRead=false;
      }
    }

    if (_subPlatformOutput.getNumPublishers()>0) {
      if (_flagPlatformOutputRead){
        processPlatformOutput();
        _flagPlatformOutputRead=false;
      }
        updateTreeFKState();
        computeGravityTorque();
        computeInertiaTorque();
        computeCoriolisTorque();
        publishInertiaCoriolisFootInput();
        computeWrenchFromPedalMeasBias();

      if(_subLegGravityComp.getNumPublishers()>0)  
      {
        if (_flagLegGravityCompWrenchRead){
          processLegGravityComp();
          _flagLegGravityCompWrenchRead=false;
        }
          computeLegGravityCompTorque();
          publishLegCompFootInput();
      }

      if (_subForceSensor.getNumPublishers()>0){ 
        if(_flagForceSensorRead)
        {
          processForceSensor();
          _flagForceSensorRead=false;
        }

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
            ROS_INFO_ONCE("[%s force sensor]: Please put the platform in state TELEOPERATION.",Feet_Names[_platform_id]);
          }
				}else
        {
          _forceMeasurements.setZero();
        }
    }else {

      ROS_INFO_ONCE("[%s force sensor]: The platform is not connected yet",Feet_Names[_platform_id]);
    }  
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[%s force sensor]: The force sensor modifier stopped",Feet_Names[_platform_id]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void footHapticController::processPlatformOutput()
{
  _ros_platform_machineState =_msgFootOutputRead.platform_machineState;
  _ros_platform_id = _msgFootOutputRead.platform_id;
  _platform_velocityPrev=_platform_velocity;
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _platform_position(k) = _msgFootOutputRead.platform_position[rosAxis[k]] * conversion_factor[rosAxis[k]];
    _platform_velocity(k) = _msgFootOutputRead.platform_speed[rosAxis[k]] * conversion_factor[rosAxis[k]];
    _platform_effort(k) = _msgFootOutputRead.platform_effortD[rosAxis[k]];
  }
  _platformJoints.data = _platform_position;
  _platformVelocity.data = _platform_velocity;
  _platformAcc.data = AccFilter * _platform_acceleration + (1.0f - AccFilter) * ((_platform_velocity - _platform_velocityPrev)/_dt);
}
void footHapticController::readPlatformOutput(const custom_msgs::FootOutputMsg::ConstPtr &msg) {  
 
  if (!me->_flagPlatformOutputRead)
  {
    ROS_INFO_ONCE("[%s force sensor]: Platform Connected",Feet_Names[_platform_id]);
    me->_flagPlatformOutputRead = true;
    me->_msgFootOutputRead = *msg;
  }
}

void footHapticController::computeGravityTorque() {
  _myChainDyn->JntToGravity(_platformJoints, _gravityTorques);
  // cout<<_gravityTorques.data.transpose()<<endl;
}

void footHapticController::updateTreeFKState() {
  
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

void footHapticController::computeWrenchFromPedalMeasBias()
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
void footHapticController::processLegCoG()
{
  tf::pointMsgToEigen(_msgLegCoGRead.point, _legCogWrtPlatfomBase);
}
void footHapticController::readLegCoG(const geometry_msgs::PointStampedConstPtr &msg) {
  if(!me->_flagLegCoGRead)
  {
    me->_msgLegCoGRead = *msg;
    me->_flagLegCoGRead=true;
  }
}

void footHapticController::processLegGravityComp()
{
  tf::wrenchMsgToEigen(_msgLegGravityCompRead.wrench, _legWrenchGravityComp);
}

void footHapticController::readLegGravityComp(const geometry_msgs::WrenchStampedConstPtr &msg) {
  if(!me->_flagLegGravityCompWrenchRead)
  {
    me->_msgLegGravityCompRead = *msg;
    me->_flagLegGravityCompWrenchRead = true;
  }
}


void footHapticController::computeLegGravityCompTorque() {
  //cout<<_myFootBaseJacobian.data<<endl;
  _legTorquesGravityComp_prev = _legTorquesGravityComp;
  _legTorquesGravityComp =  (1-ALPHA_LEG_COMP)*(_myFootBaseJacobian.data.transpose() * _legWrenchGravityComp) + ALPHA_LEG_COMP*_legTorquesGravityComp_prev;
  
}


void footHapticController::publishForceModified() {
  
  _msgForceModified.header.stamp = ros::Time::now();
  _msgForceModified.header.frame_id = _platform_id ==RIGHT ? "/right_platform_fSensor" : "/left_platform_fSensor";
  if (!_stop)
  {

    tf::wrenchEigenToMsg(_forceModified,_msgForceModified.wrench);
  }else
  {
    tf::wrenchEigenToMsg(Eigen::Matrix<double,NB_AXIS_WRENCH,1>::Zero(),_msgForceModified.wrench);
  }

    _pubForceModified.publish(_msgForceModified);
}

void footHapticController::publishPedalBias(){
  _msgPedalBias.header.stamp = ros::Time::now();
  _msgPedalBias.header.frame_id = _platform_id ==RIGHT ? "/right_platform_fSensor" : "/left_platform_fSensor";
  if(!_stop)
  {
    tf::wrenchEigenToMsg(_forcePedalBias,_msgPedalBias.wrench);
  }else{
    tf::wrenchEigenToMsg(Eigen::Matrix<double,NB_AXIS_WRENCH,1>::Zero(),_msgPedalBias.wrench);
  }
    _pubPedalBias.publish(_msgPedalBias);

}


void footHapticController::publishLegCompFootInput()
{
  // cout<<"boo"<<endl;
  if(!_stop)
  {
    for (unsigned int i = 0 ; i<NB_PLATFORM_AXIS; i++)
    {
      _msgLegGravCompFI.ros_effort[rosAxis[i]] = _legTorquesGravityComp(i) ;
    }
  }else
  {
    _msgLegGravCompFI.ros_effort.fill(0.0f);
  }
  _pubLegCompFootInput.publish(_msgLegGravCompFI);
}


void footHapticController::publishInertiaCoriolisFootInput()
{
  if(!_stop)
  {
    for (unsigned int i = 0 ; i<NB_PLATFORM_AXIS; i++)
    {
      _msgInertiaCorilisFI.ros_effort[rosAxis[i]] = _inertiaTorques.data[i] + _coriolisTorques.data[i];
    }
  }else
  {
    _msgInertiaCorilisFI.ros_effort.fill(0.0f);
  }
  _pubInertiaCoriolisFootInput.publish(_msgInertiaCorilisFI);
}


void footHapticController::processForceSensor()
{ 
  _forceMeasurements(0) = _msgForceSensorRead.wrench.force.x;
	_forceMeasurements(1) = _msgForceSensorRead.wrench.force.y;
	_forceMeasurements(2) = _msgForceSensorRead.wrench.force.z;
	_forceMeasurements(3) = _msgForceSensorRead.wrench.torque.x;
	_forceMeasurements(4) = _msgForceSensorRead.wrench.torque.y;
	_forceMeasurements(5) = _msgForceSensorRead.wrench.torque.z;
  _forceMeasurements = _rotationfSensor * _forceMeasurements;
}
void footHapticController::readForceSensor(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  if (!me->_flagForceSensorRead)
	 {
		ROS_INFO_ONCE("[%s force sensor]: Force Sensor Connected",Feet_Names[_platform_id]);
    me->_msgForceSensorRead = *msg;
    me->_flagForceSensorRead = true;
	 }
}

void footHapticController::filterForce()
{
  _forceFiltered = _forceFiltered_prev.cwiseProduct(_force_filt_alphas) + (_forceMeasurements.array().cwiseProduct(1.0f - _force_filt_alphas.array())).matrix();
  _forceFiltered_prev = _forceFiltered;
}

void footHapticController::calibrateForce()
{
	if (!_flagForceCalibrated)
	{
		_undesiredForceBias+=_forceMeasurements;
		_calibrationCount++;
	}
	if (_calibrationCount==NB_CALIBRATION_COUNT)
	{
          _undesiredForceBias = (_undesiredForceBias.array()/NB_CALIBRATION_COUNT).matrix();
		  ROS_INFO("[%s force sensor]: Sensor Calibrated!",Feet_Names[_platform_id]);
		  cout<<"Undesired force bias: "<<_undesiredForceBias.transpose()<<endl;
      _flagForceCalibrated = true;
      _forcePedalBiasInit = _forcePedalBias;
      _calibrationCount = 0;
	};
}



void footHapticController::computeInertiaTorque(){
  
  _myChainDyn->JntToMass(_platformJoints, _myJointSpaceInertiaMatrix);
  Multiply(_myJointSpaceInertiaMatrix,_platformAcc,_inertiaTorques);
}

void footHapticController::computeCoriolisTorque(){
  _myChainDyn->JntToCoriolis(_platformJoints,_platformVelocity,_coriolisTorques);
}
// KDL::Frame footHapticController::readTF(std::string frame_origin_, std::string frame_destination_)
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
//     { ROS_ERROR("[%s force sensor]: %s", ex.what(),Feet_Names[_platform_id]);
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

void footHapticController::publishForceFootRestWorld(){
  if (_flagForceCalibrated)
  {	
    _msgForceFootRestWorld.header.stamp = ros::Time::now();
    _msgForceFootRestWorld.header.frame_id = _platform_id ==RIGHT ? "/right_platform_foot_rest" : "/left_platform_foot_rest";

  if(!_stop)
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

      tf::wrenchEigenToMsg(_forceInFootRest,_msgForceFootRestWorld.wrench);
    }else
    {
      tf::wrenchEigenToMsg(Eigen::Matrix<double,NB_AXIS_WRENCH,1>::Zero(),_msgForceFootRestWorld.wrench);
    }
    _pubForceFootRestWorld.publish(_msgForceFootRestWorld);
  }
}

void footHapticController::publishTorquesModified(){
  
  if (_flagForceCalibrated)
  {
    _msgTorquesModified.platform_stamp=ros::Time::now();
    _msgTorquesModified.platform_effortM.fill(0.0);
    if(!_stop)
    {
      _torquesModified = _myFootBaseJacobian.data.transpose() * _forceInFootRest;
      //std::cout<<_myFootBaseJacobian.data<<std::endl;
      // _torquesModified(p_pitch)*=-1;
      // _torquesModified(p_pitch)*=-1;


      for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
      {
        _msgTorquesModified.platform_effortM[Axis_Ros[i]]=_torquesModified(i);
      }
    }else
    {
      _msgTorquesModified.platform_effortM.fill(0.0f);
    }

    _pubTorquesModified.publish(_msgTorquesModified);
  }

}

void footHapticController::publishForceSensorStaticCoG(){
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

void footHapticController::modifyForce() {
    _forceModified = _forceFiltered - (_undesiredForceBias-_forcePedalBiasInit);
    //cout<<_forcePedalBiasInit<<endl;
}


void footHapticController::computeFootManipulability()
{
  Eigen::Matrix<double, NB_AXIS_WRENCH, NB_PLATFORM_AXIS> jacobian_ = _myFootBaseJacobian.data;
  _mySVD.compute(jacobian_, ComputeThinU | ComputeThinV);
  Eigen::VectorXd mySingValues = _mySVD.singularValues();
  //cout<<"Singular Values :"<<endl<<_mySVD.singularValues()<<endl;
  // cout << "Foot Platform Condition Number: "<<mySingValues.minCoeff()/mySingValues.maxCoeff() << endl;
  // cout << "Foot Platform Yoshikawa Manipulability: "<<mySingValues.prod()<< endl;
}


void footHapticController::publishManipulabilityEllipsoidRot() {
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

void footHapticController::publishManipulabilityEllipsoidLin() {
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

void footHapticController::loadModels(int whichFoot)
{
  if (whichFoot==BOTH_FEET)
  {
    for (size_t i = 0; i < _nFoot; i++)
    {
      loadModels(i);
    }    
  }else{
  
  }
}
