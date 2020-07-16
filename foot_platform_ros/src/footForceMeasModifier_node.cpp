#include "footForceMeasModifier.h"

int main(int argc, char **argv) {
  footForceMeasModifier::Platform_Name platform_id_;
  
  ros::init(argc, argv, "footForceModifier");

  std::string platform_name;
  
  ros::NodeHandle nh_("~");
  nh_.getParam("platformID", platform_name);

  if (platform_name.compare("right")==0)
  {
    platform_id_ = footForceMeasModifier::RIGHT;
  } 
  else if (platform_name.compare("left")==0) {
    platform_id_ = footForceMeasModifier::LEFT;
  }
  else {
    ROS_ERROR("You didn't enter a platformID left or right");
    return -1;
  }
  
  urdf::Model modelLoad;
  if (!modelLoad.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  float frequency = 400.0f;

  footForceMeasModifier footForceMeasModifier(nh_, frequency, platform_id_, modelLoad);
    
    if (!footForceMeasModifier.init()) {
      ROS_ERROR("Failed to initialize the node");
      return -1;
      
    } else {
      footForceMeasModifier.run();
      cout<<"error<<endl;
    }
    return 0;
}
