#include "footPlatformVirtual.h"

int main(int argc, char **argv) {
  footPlatformVirtual::Platform_Name platform_id_;
  
  ros::init(argc, argv, "footPlatformVirtual");

  std::string platform_name;
  
  ros::NodeHandle nh_("~");
  nh_.getParam("platformID", platform_name);

  if (platform_name.compare("right")==0)
  {
      platform_id_ = footPlatformVirtual::RIGHT;
  } 
  else if (platform_name.compare("left")==0) {
    platform_id_ = footPlatformVirtual::LEFT;
  }
    else {
  ROS_ERROR("You didn't enter a platformID left or right");
  return -1;
  }
  
  float frequency = 450.0f;
  footPlatformVirtual footPlatformVirtual(nh_, frequency, platform_id_);

    if (!footPlatformVirtual.init()) {
      return -1;
    } else {
      footPlatformVirtual.run();
    }
    return 0;
}
