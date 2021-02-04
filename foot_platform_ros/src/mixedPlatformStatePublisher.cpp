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
    _prevEnabledToolID=RIGHT_TOOL_ID;
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
    
    _mixedPlatformState = BOTH_TOOLS_DISABLED;
    
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
        if (counterRosWarnMsg>5)
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

  if (_mixedPlatformState==STANDBY)
  { 
    if ( _platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[CLUTCH]) > 0.5*_auxPlatformWSLims(_auxPlatformAxisToAction[CLUTCH],L_MAX) )
    {
      _prevEnabledToolID = _currentToolID;
      _prevEnabledToolIT = _currentToolIT;
      _mixedPlatformState= BOTH_TOOLS_DISABLED;
    } else if (_platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[CLUTCH]) < 0.5*_auxPlatformWSLims(_auxPlatformAxisToAction[CLUTCH],L_MIN))
    {
      switch (_prevEnabledToolID)
      {
      case RIGHT_TOOL_ID:
      {
        _mixedPlatformState=RIGHT_TOOL_ENABLED;
      
          break;
      }
      case LEFT_TOOL_ID:
      {
        _mixedPlatformState=LEFT_TOOL_ENABLED;
        break;
      }
      default:
        _mixedPlatformState=RIGHT_TOOL_ENABLED;
        break;
      }
    }
  }
  if ( _platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[SWITCH]) < 0.5*_auxPlatformWSLims(_auxPlatformAxisToAction[SWITCH],L_MIN) )
  {
    if (_mixedPlatformState!=WAITING_TOOL_TO_CHANGE)
    {
      _mixedPlatformState=WAITING_TOOL_TO_CHANGE;
    }
  } else if (_platformPosition[_auxPlatformIT](_auxPlatformAxisToAction[SWITCH]) > 0.3*_auxPlatformWSLims(_auxPlatformAxisToAction[SWITCH],L_MIN))
  {
    if (_mixedPlatformState==WAITING_TOOL_TO_CHANGE)
    {
      ROS_INFO("[mixed Platform]: About to change tool ");
      switch (_currentToolID)
      {
        case RIGHT_TOOL_ID:
        { 
          ROS_INFO("[mixed Platform]: Next tool is the left one ");
          _mixedPlatformState=LEFT_TOOL_ENABLED;
          break;
        
        }
        case LEFT_TOOL_ID:
        {
          ROS_INFO("[mixed Platform]: Next tool is the right one ");
          _mixedPlatformState=RIGHT_TOOL_ENABLED;
          break;
        }
        default:
          _mixedPlatformState=RIGHT_TOOL_ENABLED;
          break;
        }
    }
  }


}


void mixedPlatformStatePublisher::mixPlatformsActions() {
 
   
   switch (_mixedPlatformState)
    {
      case BOTH_TOOLS_DISABLED:
      {
        _currentToolID=NO_TOOL_ID;
        _mixedPlatformState=STANDBY;
        break;
      }
      case STANDBY: case WAITING_TOOL_TO_CHANGE:
      {
        //Do nothing
        break;
      }
      case RIGHT_TOOL_ENABLED:
      {
        if (_toolTypes[RIGHT_TOOL_IT]==FORCEPS)
        {
          _currentToolID=RIGHT_TOOL_ID;
          _currentToolIT=RIGHT_TOOL_IT;
          _mixedPlatformState=CALCULATE_OFFSET_MIXED_PLATFORM;
        }else{
          if (_platformPosition[_mainPlatformIT].norm()<0.01)
          {
            _currentToolID=RIGHT_TOOL_ID;
            _currentToolIT=RIGHT_TOOL_IT;
            _mixedPlatformState=STANDBY;
          } else {
            ROS_WARN_ONCE("[mixed platform]: please center your main foot");
          }
        }
        break;
      }
      case LEFT_TOOL_ENABLED:
      {
        if (_toolTypes[LEFT_TOOL_IT]==FORCEPS)
        {
          _currentToolID=LEFT_TOOL_ID;
          _currentToolIT=LEFT_TOOL_IT;
          _mixedPlatformState=CALCULATE_OFFSET_MIXED_PLATFORM;
        }else{
          if (_platformPosition[_mainPlatformIT].norm()<0.01)
          {
            _currentToolID=LEFT_TOOL_ID;
            _currentToolIT=LEFT_TOOL_IT;
            _mixedPlatformState=STANDBY;
          } else {
            ROS_WARN("[mixed platform]: please center your main foot");
          }
        }
        break;
      }
      case CALCULATE_OFFSET_MIXED_PLATFORM:
      
      {
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _platformPositionOffset[_currentToolIT](i) = *_mixedPlatformPosition[i];
        } 
        _mixedPlatformState=STANDBY;
        break;
      }   
    }

      

}

void mixedPlatformStatePublisher::publishMixedPlatformJointStates() {
  //! Keep send the same valuest that the platform is broadcasting
  //_mutex.lock();
  
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
    _msgMixedPlatformState.mixedPlatformOffset[k] = -_platformPositionOffset[_currentToolIT](k);
  }
  

  _pubMixedPlatformJointStates.publish(_msgMixedPlatformState);
  //_mutex.unlock();
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

