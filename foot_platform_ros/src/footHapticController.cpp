#include <footHapticController.h>

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

const float MaxGain = 10.0f;
const int AxisRos[] = {1,0,2,3,4};


const float effortLims[] = {19.0,19.0,10.0,10.0,10.0};
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
  _maxGainForAllJoints = MaxGain;



  
  _minJND = 0.3;
  for (size_t i = 0; i < NB_PLATFORMS; i++)
    {
      _feetID[i] = NO_FOOT_ID;
      _feetID[i] = *(feet_id + i);
      _nFoot = _nFoot + ((_feetID[i] > NO_FOOT_ID) ? 1 : 0);
    }
   
  for (size_t i = 0; i < _nFoot; i++)
  {
    


    for (size_t j = 0; j < NB_PLATFORM_AXIS; i++)
    {     
      _vibFreq[i][j] = 0.0;
      _vibFB[i][j] = 0.0; 
      _vibFBGenerator[i][j] = new smoothSignals<double>(smoothSignals<double>::SINUSOID,&_vibFB[i][j],_vibFreq[i][j]);
    
    }

    _legToPlatformGravityWrench[i].setZero();
    _platformToLegGravityWrench[i].setZero();
    _legToPlatformGravityEfforts[i].setZero();
    _maxPossibleGains[i].setConstant(MaxGain);

    _platform_position[i].setZero();
    _platform_velocity[i].setZero();
    _platform_effort[i].setZero();

    _leg_position[i].setZero();
    _leg_velocity[i].setZero();
    _leg_effort[i].setZero();
    _inPlatformHapticWrench[i].setZero();
    _effortGain[i].setZero();
    
    
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
    _legFDSolver[i] =  new KDL::Torque2TaskSpace_wdls(_legFootBaseChain[i]);
    
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
    
    
    
    _outPlatformHapticEffortsMax[i];
    _outPlatformHapticEffortsMax[i].setZero();

    _inPlatformToLegHapticEfforts[i].setZero();
    _normalizedEffortCoeffsInLeg[i].setZero();
    _jointLimitGaussianFilterCoeff[i].setZero();
    _weberRatios[i].setZero();
    _weberRatiosLeg[i].setZero();
    _weberLegCoeff[i].setZero();
    _userDefinedJND[i].setConstant(_minJND);
    _outPlatformToLegHapticEffortsMax[i].setZero();
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
    _platformToLegGravityTorques[i].setZero();


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
  std::vector<double> platformEffortLims;

  if(!_n.getParam("/hapticControl/platformEffortLims", platformEffortLims))
  {
    ROS_INFO("[footHapticController: ] platformEffortLims was not indicated, default Y 19.0 X 19.0 PITCH 10.0 ROLL 10.0 YAW 10.0 ");
    for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
    {
      platformEffortLims.push_back(effortLims[j]);
    }
  }


  if(!_n.getParam("/hapticControl/maxGain", _maxGainForAllJoints))
  {
    ROS_INFO("[footHapticController: ] maxGain was not indicated, default to 10.0");
    _maxGainForAllJoints = MaxGain;
  }else
  {
    ROS_INFO("[footHapticController: ] maxGain will be %f", _maxGainForAllJoints );
  }

  if(!_n.getParam("/hapticControl/minJNDGain", _minJND))
  {
    ROS_INFO("[footHapticController: ] minJNDGain was not indicated, default to 0.3");
    _minJND = 0.3;
  }

  for (size_t i = 0; i < _nFoot; i++)
  {
    _userDefinedJND[i].setConstant(_minJND);
    for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
    {
      _outPlatformHapticEffortsMax[i](j) = platformEffortLims[j];
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

void footHapticController::stopNode(int sig) { me->_stop = true; for (size_t i = 0; i < me->_nFoot; i++) { me->publishHapticEfforts(int (i));}
}

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
        publishHapticEfforts(i);
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
  float minMaxGain = _maxGainForAllJoints;
  float maxVelocity = 1.0f;
  KDL::JntArray legToPlatformGravJntEfforts(NB_PLATFORM_AXIS);
  for (size_t i = 0; i < _nFoot; i++)
  {
    for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
    {
      _vibFreq[i][j] = Utils_math<double>::map(_platform_velocity[i](j),0.0,maxVelocity,0.0,100.0);
      _vibFBGenerator[i][j]->changeParams(smoothSignals<double>::SINUSOID,_vibFreq[i][j]);
      _vibFBGenerator[i][j]->run(ros::Time::now());
    }
    
    
    _legToPlatformGravityWrench[i].setZero();
    _platformToLegGravityWrench[i].setZero();
    _legToPlatformGravityEfforts[i].setZero();
    _maxPossibleGains[i].setConstant(_maxGainForAllJoints);
    legToPlatformGravJntEfforts.data.setZero();
    _weberRatios[i].setZero();
    _weberRatiosLeg[i].setZero();
    _weberLegCoeff[i].setZero();

    // Calculate maximum 
    _legFDSolver[i]->JntToCart(_legJoints[i],_legGravityTorques[i],_legToPlatformGravityWrench[i]);
    _legToPlatformGravityEfforts[i] = _platformFootRestJacobian[i].data.transpose() * _legToPlatformGravityWrench[i];
     legToPlatformGravJntEfforts.data = _legToPlatformGravityEfforts[i];
    _platformFDSolver[i]->JntToCart(_platformJoints[i], legToPlatformGravJntEfforts,
                                  _platformToLegGravityWrench[i]);
    
    _platformToLegGravityTorques[i] = _legFootBaseJacobian[i].data.transpose() * _platformToLegGravityWrench[i];

    _maxPossibleGains[i] = ( _inPlatformHapticEfforts[i].data.array().cwiseAbs() > __FLT_EPSILON__ ).select (
                          ((_inPlatformHapticEfforts[i].data.array() < 0.0f).select(
                              -_outPlatformHapticEffortsMax[i],_outPlatformHapticEffortsMax[i]) -
                               _legToPlatformGravityEfforts[i]).cwiseAbs().cwiseQuotient(_inPlatformHapticEfforts[i].data.cwiseAbs()), _maxGainForAllJoints); 
    
    //_maxPossibleGains[i] = _maxPossibleGains[i].cwiseMin(MaxGain);
    Eigen::MatrixXd::Index minGainIndex;
    minMaxGain = Utils_math<float>::bound(_maxPossibleGains[i].minCoeff(&minGainIndex),1.0f,_maxGainForAllJoints);

    
    cout<<"_legToPlatformGravityTorques "<<_legToPlatformGravityEfforts[i].transpose()<<endl;

    cout<<"_legGravityWrench "<<_legToPlatformGravityWrench[i].transpose()<<endl;
 
     cout<<"_legGravityWrenchBack "<<_platformToLegGravityWrench[i].transpose()<<endl;
 
     
     
     cout<<"_legGravityTorques "<<_legGravityTorques[i].data.transpose()<<endl;
 
     cout<<"_legGravityTorquesBack "<<_platformToLegGravityTorques[i].transpose()<<endl;

    cout<<"_maxPossibleGains "<<_maxPossibleGains[i].transpose()<<endl;
    
    _platformFDSolver[i]->JntToCart(_platformJoints[i], _inPlatformHapticEfforts[i],
                                  _inPlatformHapticWrench[i]);
    
    _inPlatformToLegHapticEfforts[i] = _legFootBaseJacobian[i].data.transpose() * _inPlatformHapticWrench[i]; 
    cout<<"projected efforts in leg "<< _inPlatformToLegHapticEfforts[i].transpose() << endl;

    if ((_inPlatformToLegHapticEfforts[i].cwiseAbs()).norm()>FLT_EPSILON)
    {
      _normalizedEffortCoeffsInLeg[i] = _inPlatformToLegHapticEfforts[i].cwiseAbs().normalized();
    }else
    {
      _normalizedEffortCoeffsInLeg[i].setZero();
    }

    cout<<"normalized efforts in leg "<< _normalizedEffortCoeffsInLeg[i].transpose() << endl;


    _jointLimitGaussianFilterCoeff[i] = ( 
                                          ((0.0f-_legJointLims[L_MIN][i].data.array()).abs() > FLT_EPSILON).select(
                                           (1.0f+sin(M_PI*(_legJoints[i].data.array()-_legJointLims[L_MIN][i].data.array())/
                                          (0.0f-_legJointLims[L_MIN][i].data.array())
                                          -M_PI/2.0f))/2.0f , 0.0f )  *
                                          ((_legJointLims[L_MAX][i].data.array()-0.0f).abs() > FLT_EPSILON).select( 
                                            (1.0f - ( (1.0f+sin(M_PI*(_legJoints[i].data.array()-0.0f)/
                                          (_legJointLims[L_MAX][i].data.array()-0.0f)
                                          -M_PI/2.0f))/2.0f) ) , 0.0f )) .matrix();

    cout<<"_jointLimitGaussianFilterCoeff "<< _jointLimitGaussianFilterCoeff[i].transpose() << endl;
    // cout<<"_legJoints "<< _legJoints[i].data.transpose()<<endl;
    // cout<<"_legJointsMin "<< _legJointLims[L_MIN][i].data.transpose()<<endl;
    // cout<<"_legJointsMax "<< _legJointLims[L_MAX][i].data.transpose()<<endl;
    cout<<"_minMaxPossibleGain "<< minMaxGain << endl;

    _weberRatiosLeg[i] =  ( _legGravityTorques[i].data.array().abs() > FLT_EPSILON ).select
                                     (  _inPlatformToLegHapticEfforts[i].array().abs() /
                                     _legGravityTorques[i].data.array().abs() , 0.0f) ;
    cout<<"weber ratios leg "<< _weberRatiosLeg[i].transpose() <<endl;

    _weberLegCoeff[i] = (_weberRatiosLeg[i].array()  < _userDefinedJND[i].array()).select(
                                      (_weberRatiosLeg[i].array() > FLT_EPSILON ).select(  
                                      (_userDefinedJND[i].array()) / _weberRatiosLeg[i].array(), 1.0f
                                      ), 1.0f );
    cout<<"weber coefficients "<< _weberLegCoeff[i].transpose() <<endl;

    if(_normalizedEffortCoeffsInLeg[i].cwiseAbs().sum()>__FLT_EPSILON__)
     {
      _effortGain[i].setConstant(_normalizedEffortCoeffsInLeg[i].dot(
                        (_jointLimitGaussianFilterCoeff[i].array() * (_weberLegCoeff[i].array()- 1.0f) + 1.0f).matrix())/_normalizedEffortCoeffsInLeg[i].sum() );

      // _effortGain[i].setConstant(_normalizedEffortCoeffsInLeg[i].dot(
      //                   _jointLimitGaussianFilterCoeff[i])/_normalizedEffortCoeffsInLeg[i].sum() 
      //                  );

    cout<<"effortGains "<< _effortGain[i].transpose() << endl;   
    
    _effortGain[i] = _effortGain[i].cwiseMin(minMaxGain).cwiseMax(1.0f);
                
    cout<<"effortGainsWeber "<< _effortGain[i].transpose() << endl;   

     }else
     {
       _effortGain[i].setConstant(1.0f);
     }
    

    bool vibrationOn=true; 
    float frequency = 0.0f;
    if (vibrationOn)
    {
      for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
      {
        _outPlatformHapticEfforts[i](j) = _inPlatformHapticEfforts[i].data(j) + (_effortGain[i](j) - 1.0f) * (Utils_math<double>::map(_vibFB[i][j],-1.0f,1.0f,0.0f,1.0f));                                                        
      }

    }else
    {
      _outPlatformHapticEfforts[i] = _effortGain[i].cwiseProduct(_inPlatformHapticEfforts[i].data);                                                        
    }
      
      _outPlatformHapticEfforts[i] = _outPlatformHapticEfforts[i].cwiseMin(_outPlatformHapticEffortsMax[i]).cwiseMax(-_outPlatformHapticEffortsMax[i]);
  }
}




void footHapticController::publishHapticEfforts(int whichFoot)
{
  _outMsgHapticEfforts[whichFoot].ros_filterAxisForce.fill(1.0f);
  if (!_stop)
  {
    for (size_t j = 0; j < NB_PLATFORM_AXIS; j++)
    {
      _outMsgHapticEfforts[whichFoot].ros_effort[AxisRos[j]] = _outPlatformHapticEfforts[whichFoot](j);
    }
  }else
  {
    _outMsgHapticEfforts[whichFoot].ros_effort.fill(0.0f);
  }

  _pubHapticEfforts[whichFoot].publish(_outMsgHapticEfforts[whichFoot]);
}

void footHapticController::computeLegGravityTorques(int whichFoot){
  _legChainDyn[whichFoot]->JntToGravity(_legJoints[whichFoot],_legGravityTorques[whichFoot]);
}