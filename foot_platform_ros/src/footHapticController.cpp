#include <footHapticController.h>

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

const float AccFilter = 0.99f;
const int AxisRos[] = {1,0,2,3,4};

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{
  PLATFORM_AXES};
#undef ListofPlatformAxes

#define ListofLegAxes(enumeration, names) names,
char const *Leg_Axis_Names[]{LEG_AXES};
#undef ListofLegAxes

char const *Feet_Names[]{"none","right", "left"};



footHapticController *footHapticController::me = NULL;

footHapticController::footHapticController (const ros::NodeHandle &n_1, const float &frequency,
    FEET_ID* feet_id ) 
    : _n(n_1), _loopRate(frequency),
      _dt(1.0f / frequency), _grav_vector(0.0, 0.0, (double)GRAVITY)
      {
   me = this;
  _stop = false;
  _nFoot = 0;
  
  for (size_t i = 0; i < NB_PLATFORMS; i++)
  {
    _feetID[i] = *(feet_id + i);
    _nFoot = _nFoot + (_feetID[i] > NO_FOOT_ID) ? 1 : 0;
    
    _platform_position[i].setZero();
    _platform_velocity[i].setZero();
    _platform_effort[i].setZero();

    _leg_position[i].setZero();
    _leg_velocity[i].setZero();
    _leg_effort[i].setZero();
    _inPlatformHapticWrench[i].setZero();
    _outPlatformHapticWrenchMax[i].setZero();
    _outPlatformHapticWrenchMin[i].setZero();
    

    
    
    urdf::Model platformModelLoad;
    if (!platformModelLoad.initParam("/"+std::string(Feet_Names[_feetID[i]])+"_platform/robot_description")) 
    {
      ROS_ERROR("[footHapticController: ] Failed to parse platform urdf file");
      _stop=true;
    }
    _platformModel[i]=platformModelLoad;
    urdf::Model legModelLoad;
    if (!legModelLoad.initParam("/"+std::string(Feet_Names[_feetID[i]])+"_leg/robot_description")) {
      ROS_ERROR("[footHapticController: ] Failed to parse  one of the leg urdf file, please check");
      _stop=true;
    }
    _legModel[i]=legModelLoad;
    ROS_INFO("[footHapticController: ] Successfully parsed %s leg and platform urdf files", Feet_Names[_feetID[i]]);
    
    KDL::Tree platformTreeLoad;
     if (!kdl_parser::treeFromUrdfModel(platformModelLoad, platformTreeLoad)) {
      ROS_ERROR("[footHapticController: ] Failed to construct %s kdl platform tree",Feet_Names[_feetID[i]]);
      _stop=true;
    }
    _platformTree[i]=platformTreeLoad;
    KDL::Tree legTreeLoad;
      if (!kdl_parser::treeFromUrdfModel(legModelLoad, legTreeLoad)) {
        ROS_ERROR("[footHapticController: ] Failed to construct %s kdl leg tree",Feet_Names[_feetID[i]]);
        _stop=true;
      }
    _legTree[i]=legTreeLoad;  
    ROS_INFO("[footHapticController: ] Successfully constructed %s leg and platform kdl trees", Feet_Names[_feetID[i]]);
    
  
  
    platformTreeLoad.getChain(std::string(Feet_Names[_feetID[i]]) + "_platform_base_link", std::string(Feet_Names[_feetID[i]]) + "_platform_foot_rest", _platformFootRestChain[i]);
    legTreeLoad.getChain(std::string(Feet_Names[_feetID[i]]) + "_leg_hip_base_link", std::string(Feet_Names[_feetID[i]]) + "_leg_foot_base", _legFootBaseChain[i]);

    _platformChainDyn[i] = new KDL::ChainDynParam(_platformFootRestChain[i], _grav_vector);
    _legChainDyn[i] =  new KDL::ChainDynParam(_legFootBaseChain[i], _grav_vector);
    
    
    _platformFKSolver[i] =  new KDL::ChainFkSolverPos_recursive(_platformFootRestChain[i]);
    _legFKSolver[i] =  new KDL::ChainFkSolverPos_recursive(_legFootBaseChain[i]);

    _platformJacobianSolver[i] =  new KDL::ChainJntToJacSolver(_platformFootRestChain[i]);
    _legJacobianSolver[i] =  new KDL::ChainJntToJacSolver(_legFootBaseChain[i]);
    _platformFDSolver[i] =  new KDL::Torque2TaskSpace_wdls(_platformFootRestChain[i]);
    
    _platformSegments[i] = _platformFootRestChain[i].segments;
    _legSegments[i] = _legFootBaseChain[i].segments;
    
    _platformFrames[i].resize(_platformSegments[i].size());
    _legFrames[i].resize(_legSegments[i].size());

    if (_platformFootRestChain[i].getNrOfJoints()!=NB_PLATFORM_AXIS)
    {
      ROS_ERROR("[footHapticController: ] Failed to validated %s platform's number of joints",Feet_Names[_feetID[i]]);
      _stop=true;
    }

    if (NB_LEG_AXIS!=NB_LEG_AXIS)
    {
      ROS_ERROR("[footHapticController: ] Failed to validated %s leg's number of joints",Feet_Names[_feetID[i]]);
      _stop=true;
    }

    _inPlatformHapticEfforts[i].resize(NB_PLATFORM_AXIS);
    _inPlatformHapticEfforts[i].data.setZero();
    
    
    
    _outPlatformHapticEffortsMax[i].resize(NB_PLATFORM_AXIS);
    _outPlatformHapticEffortsMax[i].data.setZero();
    _outPlatformHapticEffortsMin[i].resize(NB_PLATFORM_AXIS);
    _outPlatformHapticEffortsMin[i].data.setZero();

    _inPlatformToLegHapticEfforts[i].setZero();
    _outPlatformToLegHapticEffortsMax[i].setZero();
    _outPlatformToLegHapticEffortsMin[i].setZero();
    _outPlatformHapticEfforts[i].setZero();



    _platformJoints[i].resize(NB_PLATFORM_AXIS);
    _platformJoints[i].data.setZero();
    _platformSegments[i].resize(NB_PLATFORM_AXIS);


    _platformJointLims[L_MIN][i].resize(NB_PLATFORM_AXIS);
    _platformJointLims[L_MIN][i].data.setZero();
    _platformJointLims[L_MAX][i].resize(NB_PLATFORM_AXIS);
    _platformJointLims[L_MAX][i].data.setZero();
    
    
    
    _legJoints[i].resize(NB_LEG_AXIS);
    _legJoints[i].data.setZero();
    
    _legJointLims[L_MIN][i].resize(NB_LEG_AXIS);
    _legJointLims[L_MIN][i].data.setZero();
    _legJointLims[L_MAX][i].resize(NB_LEG_AXIS);
    _legJointLims[L_MAX][i].data.setZero();
    
    
    _legGravityTorques[i].resize(NB_LEG_AXIS);
    _legGravityTorques[i].data.setZero();


    _platformFootRestFrame[i] = KDL::Frame::Identity();
    
    _platformFootRestJacobian[i].resize(NB_PLATFORM_AXIS);
    _platformFootRestJacobian[i].data.setZero();

    _legFootBaseJacobian[i].resize(NB_LEG_AXIS);
    _legFootBaseJacobian[i].data.setZero();    

  _rotationfSensor[i].setIdentity();
  if (_feetID[i]==LEFT_FOOT_ID)
	{
		ROS_INFO("[footHapticController: ] The rotation matrix for the left force sensor has been considered!");
		Eigen::Matrix3d rotTemp;
		rotTemp<< cos(-M_PI), -sin(-M_PI), 0,
            sin(-M_PI),  cos(-M_PI), 0,
                0,           0, 1;
		_rotationfSensor[i].block(0,0,3,3) = rotTemp;
		_rotationfSensor[i].block(3,3,3,3) = rotTemp;
	}
	  
    _platformFootRestJacobian[i].resize(NB_PLATFORM_AXIS);
    _legFootBaseJacobian[i].resize(NB_LEG_AXIS);
    
    _flagLegJointStateRead[i] = false;
    _flagPlatformJointStateRead[i] = false;
  
  

  for (int joint_=0; joint_<NB_PLATFORM_AXIS; joint_++ )
   {
     _platformJointLims[L_MIN][i].data(joint_) = platformModelLoad.getJoint(std::string(Feet_Names[_feetID[i]]) + "_" + std::string(Platform_Axis_Names[joint_]))->limits->lower;
     _platformJointLims[L_MAX][i].data(joint_) = platformModelLoad.getJoint(std::string(Feet_Names[_feetID[i]]) + "_" + std::string(Platform_Axis_Names[joint_]))->limits->upper;
   }

  for (int joint_=0; joint_<NB_LEG_AXIS; joint_++ )
  {
    _legJointLims[L_MIN][i].data(joint_) = legModelLoad.getJoint(std::string(Feet_Names[_feetID[i]]) + "_leg_" + std::string(Leg_Axis_Names[joint_]))->limits->lower;
    _legJointLims[L_MAX][i].data(joint_) = legModelLoad.getJoint(std::string(Feet_Names[_feetID[i]]) + "_leg_" + std::string(Leg_Axis_Names[joint_]))->limits->upper;
  }
  

  }


}

footHapticController::~footHapticController() { me->_n.shutdown(); }

bool footHapticController::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  float maxEffortPlatform[] = {19.0,19.0,5.0,5.0,5.0};
  

  for (size_t i = 0; i < _nFoot; i++)
  {
    for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
    {
      _outPlatformHapticEffortsMax[i].data(j) = maxEffortPlatform[j];
    }
    for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
    {
      _outPlatformHapticEffortsMin[i].data(j) = -maxEffortPlatform[j];
    }
    
    _pubHapticEfforts[i] = _n.advertise<custom_msgs::FootInputMsg>("/"+std::string(Feet_Names[_feetID[i]])+"/foot_haptic_efforts", 1);
    _subPlatformJointState[i] = _n.subscribe<sensor_msgs::JointState>(
					    	"/"+std::string(Feet_Names[_feetID[i]])+"_platform"+"/platform_joint_publisher/joint_states", 1,boost::bind(&footHapticController::readPlatformJointState, this, _1, (int)i ),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegJointState[i] = _n.subscribe<sensor_msgs::JointState>(
					    	"/"+std::string(Feet_Names[_feetID[i]])+"_leg"+"/leg_joint_publisher/joint_states", 1,boost::bind(&footHapticController::readLegJointState, this, _1, (int)i ),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subDesiredHapticEfforts[i] = _n.subscribe<custom_msgs::FootInputMsg>(
					    	"/"+std::string(Feet_Names[_feetID[i]])+"/surgical_task/foot_input", 1,boost::bind(&footHapticController::readDesiredHapticEfforts, this, _1, (int)i ),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  }

  // Subscriber definitions
  signal(SIGINT, footHapticController::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[footHapticController: ] The footHapticControllerisher is about to start ");
    return true;
  } 
  else {
    ROS_ERROR("[footHapticController: ] The ros node has a problem.");
    return false;
  }
}

void footHapticController::stopNode(int sig) { me->_stop = true; }

void footHapticController::run() {
  while (!_stop) {
   for (size_t i = 0; i < _nFoot; i++)
    {
      if (_subPlatformJointState[i].getNumPublishers()>0 && _subLegJointState[i].getNumPublishers()>0) 
      
      {
        ROS_INFO_ONCE("[footHapticController: ] The %s platform and leg are connected!",Feet_Names[_feetID[i]]);
        if (_flagPlatformJointStateRead[i]) 
        
        {
          processPlatformJoints(i);
          updatePlatformTreeFKState(i);
          _flagPlatformJointStateRead[i] = false;
        }

        if (_flagLegJointStateRead[i])       
        {
          processLegJoints(i);
          updateLegTreeFKState(i);
          computeLegGravityTorques(i);
          _flagLegJointStateRead[i] = false;
        }
      } else 
      {
        
        ROS_INFO_ONCE("[footHapticController: ] The %s platform and leg  are not connected yet", Feet_Names[_feetID[i]]);
      }
      if (_subDesiredHapticEfforts[i].getNumPublishers()>0)

      {
        ROS_INFO_ONCE("[footHapticController: ] The %s desired Haptic efforts are now available!", Feet_Names[_feetID[i]]);
        if (_flagDesiredHapticRead[i])
        {
          processDesiredHapticEfforts(i);
          
          _flagDesiredHapticRead[i]=false;
        }

        doHapticControl();
      }else

      {
        ROS_INFO_ONCE("[footHapticController: ] The %s desired Haptic efforts are not yet available", Feet_Names[_feetID[i]]);
      }
    }  

    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[footHapticController: ] The foot haptic controller stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void footHapticController::processPlatformJoints(int whichFoot)
{
  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
  {
    _platform_position[whichFoot](i) =  _inMsgPlatformJointState[whichFoot].position[i];
    _platform_velocity[whichFoot](i) =  _inMsgPlatformJointState[whichFoot].velocity[i];
    _platform_effort[whichFoot](i) =  _inMsgPlatformJointState[whichFoot].effort[i];
  }
  _platformJoints[whichFoot].data = _platform_position[whichFoot];
}

void footHapticController::readPlatformJointState(const sensor_msgs::JointState::ConstPtr &msg, int whichFoot)
{
  me->_inMsgPlatformJointState[whichFoot] = *msg;
  me->_flagPlatformJointStateRead[whichFoot]=true;
}

void footHapticController::processLegJoints(int whichFoot)
{
  for (size_t i = 0; i < NB_LEG_AXIS; i++)
  {
    _leg_position[whichFoot](i) =  _inMsgLegJointState[whichFoot].position[i];
    _leg_velocity[whichFoot](i) =  _inMsgLegJointState[whichFoot].velocity[i];
    _leg_effort[whichFoot](i) =  _inMsgLegJointState[whichFoot].effort[i];
  }
  _legJoints[whichFoot].data = _leg_position[whichFoot];
}

void footHapticController::readLegJointState(const sensor_msgs::JointState::ConstPtr &msg, int whichFoot)
{
  me->_inMsgLegJointState[whichFoot] = *msg;
  me->_flagLegJointStateRead[whichFoot]=true;
}



void footHapticController::processDesiredHapticEfforts(int whichFoot)
{
  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
  {
    _inPlatformHapticEfforts[whichFoot].data(i) = _inMsgDesiredHapticEfforts[whichFoot].ros_effort[AxisRos[i]];
  }
}

void footHapticController::readDesiredHapticEfforts(const custom_msgs::FootInputMsg::ConstPtr &msg, int whichFoot)
{

  me->_inMsgDesiredHapticEfforts[whichFoot] = *msg;
  me->_flagDesiredHapticRead[whichFoot]=true;
}



void footHapticController::updatePlatformTreeFKState(int whichFoot) {
  
  KDL::Frame frame_;
  _platformFrames[whichFoot].clear(); 
  for (unsigned int i = 0; i < _platformSegments[whichFoot].size(); i++) {
    _platformFKSolver[whichFoot]->JntToCart(_platformJoints[whichFoot], frame_, i + 1);
    _platformFrames[whichFoot].push_back(frame_);
    // cout<<_myFrames[i].p.data[0]<<" "<<_myFrames[i].p.data[1]<<" "<<_myFrames[i].p.data[2]<<endl;
  }
  
  _platformFootRestFrame[whichFoot] = _platformFrames[whichFoot][_platformFootRestChain[whichFoot].getNrOfSegments()-1];
  
  _platformJacobianSolver[whichFoot]->JntToJac(_platformJoints[whichFoot],_platformFootRestJacobian[whichFoot],_platformFootRestChain[whichFoot].getNrOfSegments() - 1);
  
}



void footHapticController::updateLegTreeFKState(int whichFoot) {
  
  KDL::Frame frame_;
  _legFrames[whichFoot].clear(); 
  for (unsigned int i = 0; i < _legSegments[whichFoot].size(); i++) {
    _legFKSolver[whichFoot]->JntToCart(_legJoints[whichFoot], frame_, i + 1);
    _legFrames[whichFoot].push_back(frame_);
  }
  
  _legFootBaseFrame[whichFoot] = _legFrames[whichFoot][_legFootBaseChain[whichFoot].getNrOfSegments()-1];
  
  _legJacobianSolver[whichFoot]->JntToJac(_legJoints[whichFoot],_legFootBaseJacobian[whichFoot],_legFootBaseChain[whichFoot].getNrOfSegments() - 1);
  
}


void footHapticController::doHapticControl()
{
  float maxGain_j = 10.0f; 
  for (size_t i = 0; i < _nFoot; i++)  {

    _platformFDSolver[i]->JntToCart(_platformJoints[i], _outPlatformHapticEffortsMax[i],
                               _outPlatformHapticWrenchMax[i]);
                               
    _platformFDSolver[i]->JntToCart(_platformJoints[i], _outPlatformHapticEffortsMin[i],
                               _outPlatformHapticWrenchMin[i]);
    
    _outPlatformToLegHapticEffortsMax[i] = _legFootBaseJacobian[i].data.transpose() * _outPlatformHapticWrenchMax[i];  
    _outPlatformToLegHapticEffortsMin[i] = _legFootBaseJacobian[i].data.transpose() * _outPlatformHapticWrenchMin[i];  

    // //Convert the desired haptic torques in a desired haptic wrench in the platform-foot-rest  frame (collocated with leg-foot-base) frame
    // cout<<"_inPlatformHapticEfforts "<<     _inPlatformHapticEfforts[i].data.transpose()<<endl;  
     _platformFDSolver[i]->JntToCart(_platformJoints[i], _inPlatformHapticEfforts[i],
                                  _inPlatformHapticWrench[i]);
    // cout<<"_inPlatformHapticWrench "<<     _inPlatformHapticWrench[i].transpose()<<endl;  
    // //Convert the wrench felt in the leg-foot-base frame to efforts in the joints of the leg
     _inPlatformToLegHapticEfforts[i] = _legFootBaseJacobian[i].data.transpose() * _inPlatformHapticWrench[i]; 

    // //Finding the possible gains for amplification of the efforts  

    Eigen::Array<float, NB_LEG_AXIS,1> gainsFirstStep_i;
    // gainFirstStep is an array of the possible gains that each joint can have 
    // lambdaFirstStep is the array of effort-deltas to avoid singular solutions in the gains
    // nFirstStep is the array of signs of the desired efforts in the axis
    gainsFirstStep_i.setConstant(1.0f);
    float lambdaFirstStep_j = 0.0f;
    float nFirstStep_j = 1.0;
    float minGain_j = maxGain_j;
    float legTorque_j = 0.0f;
    float deltaToEffort_j = 0.0f;
    float remainingEffort_j = 0.0f;
    float effortMax = 0.0f;
    for (size_t j = 0; j < NB_LEG_AXIS; j++)
    {
      if (fabs(_inPlatformToLegHapticEfforts[i](j)) > FLT_EPSILON)
      {
        legTorque_j = Utils_math<float>::bound(_legGravityTorques[i].data(j),_outPlatformToLegHapticEffortsMin[i](j), _outPlatformToLegHapticEffortsMax[i](j));
        nFirstStep_j = _inPlatformToLegHapticEfforts[i](j)<0.0 ? -1.0 : 1.0;
        effortMax = nFirstStep_j < 0.0 ? _outPlatformToLegHapticEffortsMin[i](j) : _outPlatformToLegHapticEffortsMax[i](j);
        lambdaFirstStep_j = fabs((maxGain_j * 0.01f) * ( effortMax - _inPlatformToLegHapticEfforts[i](j)));
        deltaToEffort_j = fabs(_inPlatformToLegHapticEfforts[i](j) + nFirstStep_j * lambdaFirstStep_j); 
        remainingEffort_j = ( fabs( effortMax - legTorque_j) -  deltaToEffort_j );
        if (remainingEffort_j>0.0)
        {
          gainsFirstStep_i(j) = ( remainingEffort_j /  deltaToEffort_j ) + 1.0f;
        }else{

          gainsFirstStep_i(j)=1.0f;
        }
        minGain_j = gainsFirstStep_i(j) < minGain_j ? gainsFirstStep_i(j) : minGain_j;

        // cout<<"inTorqueToLeg "<<_inPlatformToLegHapticEfforts[i](j)<<endl;
        // cout<<"legTorque "<<legTorque_j<<endl;
        // cout<<"nFirstStep_j "<<nFirstStep_j<<endl;
        // cout<<"lambdaFirstStep_j "<<lambdaFirstStep_j<<endl;
       // cout<<"deltaToEffort_j "<<deltaToEffort_j<<endl;
        // cout<<"gainsFirstStep_j "<<gainsFirstStep_j<<endl;
        // cout<<"minGain_j "<<minGain_j<<endl;
      } 
    }
    std::cout<<minGain_j<<std::endl;
    minGain_j = Utils_math<float>::bound(minGain_j,1.0f,maxGain_j);
    std::cout<<gainsFirstStep_i.transpose()<<endl;
  }
  
  
}

void footHapticController::computeLegGravityTorques(int whichFoot){
  _legChainDyn[whichFoot]->JntToGravity(_legJoints[whichFoot],_legGravityTorques[whichFoot]);
}