#include "footStatePublisher.h"

int main(int argc, char **argv) {
  footStatePublisher::Platform_Name platform_id_;
  
  ros::init(argc, argv, "platformStatePublisher");

  std::string platform_name;
  
  ros::NodeHandle nh_("~");
  nh_.getParam("platformID", platform_name);

  if (platform_name.compare("right")==0)
  {
      platform_id_ = footStatePublisher::RIGHT;
  } 
  else if (platform_name.compare("left")==0) {
    platform_id_ = footStatePublisher::LEFT;
  }
    else {
  ROS_ERROR("You didn't enter a platformID left or right");
  return -1;
  }
  
  float frequency = 500.0f;
  footStatePublisher footStatePublisher(nh_, frequency, platform_id_);

    if (!footStatePublisher.init()) {
      return -1;
    } else {
      footStatePublisher.run();
    }
    return 0;
}
