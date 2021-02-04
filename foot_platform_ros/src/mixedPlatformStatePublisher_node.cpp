#include "mixedPlatformStatePublisher.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "mixedPlatformStatePublisher");

  ros::NodeHandle nh_("~");

  float frequency = 450.0f;
  mixedPlatformStatePublisher mixedPlatformStatePublisher(nh_, frequency);

    if (!mixedPlatformStatePublisher.init()) {
      return -1;
    } else {
      mixedPlatformStatePublisher.run();
    }
    return 0;
}
