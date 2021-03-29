#include "footHapticController.h"

int main(int argc, char **argv) {
  
  std::vector<footHapticController::FEET_ID> feetID;
  
  ros::init(argc, argv, "footHapticController");

  ros::NodeHandle nh_("~");
  
  std::string platform_name;

  nh_.getParam("platformID", platform_name);

  if (platform_name.compare("both")==0)
  {
    feetID.push_back(footHapticController::RIGHT_FOOT_ID);
    feetID.push_back(footHapticController::LEFT_FOOT_ID);
  } else if (platform_name.compare("right")==0)
  {
    feetID.push_back(footHapticController::RIGHT_FOOT_ID);
  } 
  else if (platform_name.compare("left")==0) {
    feetID.push_back(footHapticController::LEFT_FOOT_ID);
  }
  else {
    ROS_ERROR("[Hapic Controller]: You didn't enter a platformID left or right");
    return -1;
  }


  float frequency = 500.0f;

  footHapticController footHapticController(nh_, frequency, feetID);
    
    if (!footHapticController.init()) {
      ROS_ERROR("Failed to initialize the node for hapticController");
      return -1;
      
    } else {
      footHapticController.run();
    }
    return 0;
}
