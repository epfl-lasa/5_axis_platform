#include "mixedPlatformStatePublisher.h"


#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{
  PLATFORM_AXES};
#undef ListofPlatformAxes

#define ListofAuxPlatformActions(enumeration, names) names,
char const *Aux_Platform_Actions_Names[]{
  AUX_PLATFORMS_ACTIONS};
#undef ListofAuxPlatformActions

#define ListofMainPlatformActions(enumeration, names) names,
char const *Main_Platform_Actions_Names[]{
  MAIN_PLATFORMS_ACTIONS};
#undef ListofMainPlatformActions

char const *Platform_Names[]{"none", "right", "left"}; 
char const *Platform_Names_2[]{"None", "Right", "Left"}; 

mixedPlatformStatePublisher *mixedPlatformStatePublisher::me = NULL;

mixedPlatformStatePublisher::mixedPlatformStatePublisher(
    ros::NodeHandle &n_1, double frequency)
    : _n(n_1), _loopRate(frequency),
      _dt(1.0f / frequency) {
    me = this;
    _stop = false;
    _currentToolID=RIGHT_TOOL_ID;
    _currentToolIT=RIGHT_TOOL_IT;
    _prevEnabledToolIT = RIGHT_TOOL_IT;

    for (size_t i = 0; i < NB_PLATFORMS; i++)
    {
      _platformEffort[i].setZero();
      _platformPosition[i].setZero();
      _platformVelocity[i].setZero();
    }
    for (size_t i = 0; i < NB_TOOLS; i++)
    {
      _platformPositionOffset[i].setZero();
    }
    
    _flagWarningCenterMainPlatform=false;
    _flagPrevToolSaved=true;  
    _flagOffsetCalculated = false;
    _mixedPlatformState = BOTH_TOOLS_DISABLED;
    _nextMixedPlatformState = BOTH_TOOLS_DISABLED;
    
    for (size_t p = 0; p < NB_PLATFORMS; p++)
    {
      std::vector<double> deadZoneValuesPlatformMin;
      deadZoneValuesPlatformMin.resize(NB_PLATFORM_AXIS);
      if (!_n.getParam("/"+std::string(Platform_Names[p+1])+"_tool/"+"deadZoneValuesPlatformMin", deadZoneValuesPlatformMin))
      { 
          ROS_ERROR(" [mixed platform]: No %s deadZoneValuesPlatformMin  param", Platform_Names[p+1]); 
      }
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _deadZoneValues[p](i,L_MIN)=deadZoneValuesPlatformMin[i];
        }
      
      std::vector<double> deadZoneValuesPlatformMax;
      deadZoneValuesPlatformMax.resize(NB_PLATFORM_AXIS);
      if (!_n.getParam("/"+std::string(Platform_Names[p+1])+"_tool/"+"deadZoneValuesPlatformMax", deadZoneValuesPlatformMax))
      { 
          ROS_ERROR(" [mixed platform]: No %s deadZoneValuesPlatformMax  param", Platform_Names[p+1]); 
      }
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _deadZoneValues[p](i,L_MAX)=deadZoneValuesPlatformMax[i];
        }
    }
    
    

    std::string leftToolType, rightToolType;
    if (!_n.getParam("/left_tool/toolType", leftToolType))
    { 
      ROS_ERROR(" [mixed platform]: No /left_tool/toolType  param"); 
    }
    if (!_n.getParam("/right_tool/toolType", rightToolType))
    { 
      ROS_ERROR(" [mixed platform]: No /right_tool/toolType  param"); 
    }  
    
    _toolTypes[LEFT_TOOL_IT] =  leftToolType.compare("forceps")==0 ? FORCEPS : CAMERA;
    _toolTypes[RIGHT_TOOL_IT] =  rightToolType.compare("camera")==0 ? CAMERA : FORCEPS;
    
    std::string leftToolControl, rightToolControl;
    if (!_n.getParam("/left_tool/toolControl", leftToolControl))
    { 
      ROS_ERROR(" [mixed platform]: No /left_tool/toolControl  param"); 
    }
    if (!_n.getParam("/right_tool/toolControl", rightToolControl))
    { 
      ROS_ERROR(" [mixed platform]: No /right_tool/toolControl  param"); 
    }  
    
    _toolControls[LEFT_TOOL_IT] =  leftToolControl.compare("position")==0 ? TOOL_POSITION_CTRL : TOOL_SPEED_CTRL;
    _toolControls[RIGHT_TOOL_IT] =  rightToolControl.compare("speed")==0 ? TOOL_SPEED_CTRL : TOOL_POSITION_CTRL;
    

    std::string mainPlatform_;
    if (!_n.getParam("/mixed_platform/mainPlatform", mainPlatform_))
    { 
      ROS_ERROR(" [mixed platform]: No mainPlatform  param"); 
    } 
    if (mainPlatform_.compare("left")==0)
    {
      _mainPlatformID=LEFT_PLATFORM_ID;
      _mainPlatformIT=LEFT_PLATFORM_IT;
      _auxPlatformID=RIGHT_PLATFORM_ID;
      _auxPlatformIT=RIGHT_PLATFORM_IT;
    } else {
      _mainPlatformID=RIGHT_PLATFORM_ID;
      _mainPlatformIT=RIGHT_PLATFORM_IT;
      _auxPlatformID=LEFT_PLATFORM_ID;
      _auxPlatformIT=LEFT_PLATFORM_IT;
    }

    for (size_t i = 0; i < NB_AUX_ACTIONS_CHECK; i++)
    {
        _requestAuxAction[i]=false;
    }
    

    std::vector<double> auxPlatformWorkspaceLimsMin_, auxPlatformWorkspaceLimsMax_;
    auxPlatformWorkspaceLimsMin_.assign(NB_PLATFORM_AXIS,0.0);
    auxPlatformWorkspaceLimsMax_.assign(NB_PLATFORM_AXIS,0.0);

    if (!_n.getParam("/mixed_platform/auxPlatformWorkspaceLimsMin"+std::string(Platform_Names_2[_auxPlatformID]), auxPlatformWorkspaceLimsMin_))
    { 
      ROS_ERROR(" [mixed platform]: No auxPlatformWorkspaceLimsMin  param"); 
    }else {
      ROS_INFO(" [mixed platform]: auxPlatformWorkspaceLimsMin  param loaded"); 
    }
    if (!_n.getParam("/mixed_platform/auxPlatformWorkspaceLimsMax"+std::string(Platform_Names_2[_auxPlatformID]), auxPlatformWorkspaceLimsMax_))
    { 
      ROS_ERROR(" [mixed platform]: No auxPlatformWorkspaceLimsMax  param"); 
    }else {
      ROS_INFO(" [mixed platform]: auxPlatformWorkspaceLimsMax  param loaded"); 
    }  

    for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
    {
      _auxPlatformWSLims(i,L_MIN)=auxPlatformWorkspaceLimsMin_[i];
      _auxPlatformWSLims(i,L_MAX)=auxPlatformWorkspaceLimsMax_[i];
    }
    

    std::vector<std::string> mappingMain_, mappingAux_;

    mappingMain_.assign(NB_PLATFORM_AXIS,"");
    mappingAux_.assign(NB_PLATFORM_AXIS,"");
    
    if (!_n.getParam("/mixed_platform/mappingMain", mappingMain_))
    { 
      ROS_ERROR(" [mixed platform]: No mappingMain  param"); 
    }else {
      ROS_INFO(" [mixed platform]: mappingMain  param loaded"); 
    } 
    if (!_n.getParam("/mixed_platform/mappingAux", mappingAux_))
    { 
      ROS_ERROR(" [mixed platform]: No mappingAux  param"); 
    }else {
      ROS_INFO(" [mixed platform]: mappingAux  param loaded"); 
    }   
    
      for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
      {
        for (size_t j = 0; j < NB_MAIN_ACTIONS; j++)
        {
          
          if (mappingMain_[i].compare(std::string(Main_Platform_Actions_Names[j]))==0)
          {
            _mainPlatformAxisToAction[i]=(Main_Platform_Actions) j;
            break;
          }else
          {
            _mainPlatformAxisToAction[i]=MAIN_NOTHING;
          }
        } 
      } 

      for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
      {
        for (size_t j = 0; j < NB_AUX_ACTIONS; j++)
        {
          if (mappingAux_[i].compare(std::string(Aux_Platform_Actions_Names[j]))==0)
          {
            _auxPlatformAxisToAction[i]=(Aux_Platform_Actions) j;
            break;
          } else
          {
            _auxPlatformAxisToAction[i]=AUX_NOTHING;
          }
        } 
      }   


      ROS_INFO(" [mixed platform]: the mapping for the main platform is %s %s %s %s %s",mappingMain_[0].c_str(),mappingMain_[1].c_str(),mappingMain_[2].c_str(),mappingMain_[3].c_str(),mappingMain_[4].c_str() ); 
      
      ROS_INFO(" [mixed platform]: the mapping for the aux platform is %s %s %s %s %s",mappingAux_[0].c_str(),mappingAux_[1].c_str(),mappingAux_[2].c_str(),mappingAux_[3].c_str(),mappingAux_[4].c_str() ); 
    
      //cout<<_mainPlatformAxisToAction[0]<<" "<<_mainPlatformAxisToAction[1]<<" "<<_mainPlatformAxisToAction[2]<<" "<<_mainPlatformAxisToAction[3]<<" "<<_mainPlatformAxisToAction[4]<<endl;
      
      //cout<<_auxPlatformAxisToAction[0]<<" "<<_auxPlatformAxisToAction[1]<<" "<<_auxPlatformAxisToAction[2]<<" "<<_auxPlatformAxisToAction[3]<<" "<<_auxPlatformAxisToAction[4]<<endl;

  
}

mixedPlatformStatePublisher::~mixedPlatformStatePublisher() { me->_n.shutdown(); }

bool mixedPlatformStatePublisher::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  _pubMixedPlatformJointStates = _n.advertise<custom_msgs::TwoFeetOneToolMsg>("/mixed_platform/platform_state", 1);

  for (size_t i = 0; i < NB_PLATFORMS; i++)
  {
     _subPlatformJointStatesOutput[i] = _n.subscribe<sensor_msgs::JointState>(
        "/"+std::string(Platform_Names[i+1])+"_platform/platform_joint_publisher/joint_states", 1,
        boost::bind(&mixedPlatformStatePublisher::readPlatformsJointState, this, _1, i),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  }

  
  _mixedPlatformPosition[p_y] = &(_platformPosition[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_Y]));
  _mixedPlatformPosition[p_x] = &(_platformPosition[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_X]));
  _mixedPlatformPosition[p_pitch] = &(_platformPosition[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_Z]));
  _mixedPlatformPosition[p_yaw] = &(_platformPosition[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_YAW]));
  _mixedPlatformPosition[p_roll] = &(_platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[GRASP]));
  
  
  _mixedPlatformVelocity[p_y] = &(_platformVelocity[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_Y]));
  _mixedPlatformVelocity[p_x] = &(_platformVelocity[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_X]));
  _mixedPlatformVelocity[p_pitch] = &(_platformVelocity[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_Z]));
  _mixedPlatformVelocity[p_yaw] = &(_platformVelocity[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_YAW]));
  _mixedPlatformVelocity[p_roll] = &(_platformVelocity[_auxPlatformIT](_auxPlatformAxisToAction[GRASP]));
  
  _mixedPlatformEffort[p_y] = &(_platformEffort[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_Y]));
  _mixedPlatformEffort[p_x] = &(_platformEffort[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_X]));
  _mixedPlatformEffort[p_pitch] = &(_platformEffort[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_Z]));
  _mixedPlatformEffort[p_yaw] = &(_platformEffort[_mainPlatformIT](_mainPlatformAxisToAction[MOVE_YAW]));
  _mixedPlatformEffort[p_roll] = &(_platformEffort[_auxPlatformIT](_auxPlatformAxisToAction[GRASP]));
  // Subscriber definitions
  signal(SIGINT, mixedPlatformStatePublisher::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[mixed platform]: The platform joint state publisher is about to start ");
    return true;
  } 
  else {
    ROS_ERROR("[mixed platform]: The ros node has a problem.");
    return false;
  }
}

void mixedPlatformStatePublisher::stopNode(int sig) { me->_stop = true; }

void mixedPlatformStatePublisher::run() {
  static uint8_t counterRosWarnMsg = 0;
  while (!_stop) {
    if (_subPlatformJointStatesOutput[RIGHT_PLATFORM_IT].getNumPublishers()!=0
        && _subPlatformJointStatesOutput[LEFT_PLATFORM_IT].getNumPublishers()!=0) {
          ROS_INFO_ONCE("[mixed platform]: platforms connected");
          checkAuxPlatformsInput();
          mixPlatformsActions(); 
          publishMixedPlatformJointStates();
      }
      else
      {
        if (counterRosWarnMsg>500)
        {
          ROS_WARN("[mixed platform]: No platform state publishers detected, please check this");
          counterRosWarnMsg=0;
        }
        counterRosWarnMsg++;
      }
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[mixed platform]: Platform state variables stopped");
  
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();

}

void mixedPlatformStatePublisher::checkAuxPlatformsInput(){


  if ( _platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[CLUTCH]) > 
    0.5*_auxPlatformWSLims(_auxPlatformAxisToAction[CLUTCH],L_MAX) )
  {
    if (!_requestAuxAction[CLUTCH])
    {
      ROS_INFO("[mixed platform]: Clutch on");
      _requestAuxAction[CLUTCH]=true;
    }
  }

  if (_platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[CLUTCH]) < 
    0.3*_auxPlatformWSLims(_auxPlatformAxisToAction[CLUTCH],L_MAX))
  {
    if (_requestAuxAction[CLUTCH])
    {
      ROS_INFO("[mixed platform]: Clutch off");
      _requestAuxAction[CLUTCH]=false;
    }
  }  

  if ( _platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[SWITCH]) > 
      0.5*_auxPlatformWSLims(_auxPlatformAxisToAction[SWITCH],L_MAX))
    {
      if (!_requestAuxAction[SWITCH])
      {
        ROS_INFO("[mixed platform]: Switching tools requested");
        _requestAuxAction[SWITCH]=true;
      }
    }

    if ( _platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[SWITCH]) < 
        0.3*_auxPlatformWSLims(_auxPlatformAxisToAction[SWITCH],L_MAX))
    {
      if (_requestAuxAction[SWITCH])
      {
        ROS_INFO("[mixed platform]: Switching tools enabled");
        _requestAuxAction[SWITCH]=false;
      }
    }

 
  
}


void mixedPlatformStatePublisher::mixPlatformsActions() {
 
  switch (_mixedPlatformState)

  {
    case BOTH_TOOLS_DISABLED:
    {
      _currentToolID=NO_TOOL_ID; 
      if(_requestAuxAction[CLUTCH]) 
      {    
        if(!_flagPrevToolSaved)
        {
          _prevEnabledToolIT = _currentToolIT;
          _flagPrevToolSaved=true; 
        }
          _nextMixedPlatformState= TOOL_ENABLED;
      } else if (_nextMixedPlatformState==TOOL_ENABLED)
      {
          _currentToolIT=_prevEnabledToolIT;
          _flagPrevToolSaved=false;
          _flagOffsetCalculated=false;
          _flagWarningCenterMainPlatform=false;
          _mixedPlatformState = _nextMixedPlatformState;
      } 
      break; 
    }

    case TOOL_ENABLED:
    { 
      if (_toolControls[_currentToolIT]==TOOL_POSITION_CTRL)
      {
        if (!_flagOffsetCalculated)
        {
          for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
          {
            _platformPositionOffset[_currentToolIT](i) = *_mixedPlatformPosition[i];
          }
          _flagOffsetCalculated=true;
        }
      }else {
        // if ( ((_deadZoneValues[_currentToolIT].col(L_MAX) - _platformPosition[_mainPlatformIT]).array() > FLT_EPSILON ).all()
        //   && ((-_deadZoneValues[_currentToolIT].col(L_MIN) + _platformPosition[_mainPlatformIT]).array() > FLT_EPSILON ).all() )
        if ( ((_deadZoneValues[_currentToolIT].col(L_MAX) - _platformPosition[_mainPlatformIT]).segment(0,p_roll+1).array() > FLT_EPSILON ).all()
          && ((-_deadZoneValues[_currentToolIT].col(L_MIN) + _platformPosition[_mainPlatformIT]).segment(0,p_roll+1).array() > FLT_EPSILON ).all() )
        { 
          if(!_flagOffsetCalculated)
          {
            
            ROS_INFO("[mixed platform]: main platform centered!");
            _flagOffsetCalculated=true;
          }
        }else
        {
          if(!_flagWarningCenterMainPlatform)
          {
            ROS_WARN("[mixed platform]: please center your main platform");
            _flagWarningCenterMainPlatform=true;
          }
        }
      }
      _currentToolID = NO_TOOL_ID;
      if (_flagOffsetCalculated)
      {
        _currentToolID=(Tool_ID) ((uint8_t)_currentToolIT+1);
      }
      //! Transition
      if(_requestAuxAction[CLUTCH]) 
      {    
        _nextMixedPlatformState= BOTH_TOOLS_DISABLED;
      } 
      else if (_nextMixedPlatformState==BOTH_TOOLS_DISABLED)
      {
        _mixedPlatformState = _nextMixedPlatformState;
      }

      if(_requestAuxAction[SWITCH]) 
      {    
        _nextMixedPlatformState= TOOL_SWITCH;
      }
      else if( _nextMixedPlatformState==TOOL_SWITCH)
      {
       _mixedPlatformState = _nextMixedPlatformState;
      }
      break;
    }
     case TOOL_SWITCH:
    { 
        _currentToolID = NO_TOOL_ID;
        if (_nextMixedPlatformState==TOOL_ENABLED)
        {
          _flagWarningCenterMainPlatform=false;
          _mixedPlatformState = _nextMixedPlatformState;
        }else
        {
          if(!_flagPrevToolSaved)
          {
          _prevEnabledToolIT = _currentToolIT;
          _flagPrevToolSaved=true; 
          }
          _currentToolIT=_prevEnabledToolIT == RIGHT_TOOL_IT ? LEFT_TOOL_IT : RIGHT_TOOL_IT;
          _flagOffsetCalculated=false;
          _flagPrevToolSaved=false;
          _nextMixedPlatformState = TOOL_ENABLED;
        }
        break;
    }
  }
}

void mixedPlatformStatePublisher::publishMixedPlatformJointStates() {
  //! Keep send the same valuest that the platform is broadcasting
  _msgMixedPlatformState.currentControlMode = _toolControls[_currentToolIT];
  _msgMixedPlatformState.mixedPlatformStateMachine=_mixedPlatformState;
  _msgMixedPlatformState.currentTool=_currentToolID;
  _msgMixedPlatformState.mixedPlatformOffset.fill(0.0);
  
  _msgMixedPlatformState.mixedPlatformJointState.header.stamp = ros::Time::now();

  _msgMixedPlatformState.mixedPlatformJointState.name.resize(NB_PLATFORM_AXIS);
  _msgMixedPlatformState.mixedPlatformJointState.position.resize(NB_PLATFORM_AXIS);
  _msgMixedPlatformState.mixedPlatformJointState.velocity.resize(NB_PLATFORM_AXIS);
  _msgMixedPlatformState.mixedPlatformJointState.effort.resize(NB_PLATFORM_AXIS);


  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _msgMixedPlatformState.mixedPlatformJointState.name[k] =  "mixed_" +  std::string(Platform_Axis_Names[k]);
      _msgMixedPlatformState.mixedPlatformJointState.position[k] = *(_mixedPlatformPosition[k]);
      _msgMixedPlatformState.mixedPlatformJointState.velocity[k] = *(_mixedPlatformVelocity[k]);
      _msgMixedPlatformState.mixedPlatformJointState.effort[k] = *(_mixedPlatformEffort[k]);
      if(_toolControls[_currentToolIT]==TOOL_POSITION_CTRL)
      {
        _msgMixedPlatformState.mixedPlatformOffset[k] = -_platformPositionOffset[_currentToolIT](k);
      }
   }

  _pubMixedPlatformJointStates.publish(_msgMixedPlatformState);
}

void mixedPlatformStatePublisher::readPlatformsJointState(const sensor_msgs::JointState::ConstPtr &msg, unsigned int n_) {  
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _platformPosition[n_](k)=msg->position[k];
    _platformVelocity[n_](k)=msg->velocity[k];
    _platformEffort[n_](k)=msg->effort[k];
  }
  if (!_flagPlatformConnected[n_]) {
    _flagPlatformConnected[n_] = true;
  }
}

