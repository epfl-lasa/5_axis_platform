#include "footHapticController.h"

int main(int argc, char **argv) {
  
  footHapticController::FEET_ID feetID[NB_PLATFORMS] = {footHapticController::FEET_ID::NO_FOOT_ID,footHapticController::FEET_ID::NO_FOOT_ID};
  
  ros::init(argc, argv, "footHapticController");

  ros::NodeHandle nh_("~");
  
  std::string feet_name;

  nh_.getParam("feetID", feet_name);

  if (feet_name.compare("both")==0)
  {
    feetID[0] = footHapticController::RIGHT_FOOT_ID;
    feetID[1] = footHapticController::LEFT_FOOT_ID;
  } else if (feet_name.compare("right")==0)
  {
    feetID[0]=footHapticController::RIGHT_FOOT_ID;
  } 
  else if (feet_name.compare("left")==0) {
    feetID[0] = footHapticController::LEFT_FOOT_ID;
  }
  else {
    ROS_ERROR("[Hapic Controller]: You didn't enter a feetID left or right");
    return -1;
  }


  float frequency = 300.0f;

  footHapticController footHapticController(nh_, frequency, feetID);
    
    if (!footHapticController.init()) {
      ROS_ERROR("Failed to initialize the node for hapticController");
      return -1;
      
    } else {
      footHapticController.run();
    }
    return 0;
}
