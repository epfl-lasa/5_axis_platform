#include "mbed.h"
#include "Platform.h"
#include "definitions.h"
#include "rtos.h"


Platform platform;
Thread th_Control;

void doControlTH()
{
  while (!platform._stop)
  {
      platform.step();   
  }
}

int main()
{
    platform.init();
    th_Control.start(doControlTH);
    while (1) {
      platform.communicateToRos();
    }
  return 0;
  }

// int main()
// {
//   platform.init();
//   while (!platform._stop)
//   {
//     platform.step();
//     platform.communicateToRos();
//   }
//   return 0;
// }
