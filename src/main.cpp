#include "mbed.h"
#include "Platform.h"
#include "definitions.h"

#if (BOARD==NUCLEO64)
  #include "rtos.h"
#endif

Platform platform;
Thread th_Control;
//Ticker t_Comm;
//volatile bool flagComm = false;


// void doCommCb()
// {
//    flagComm=true;
// }


void doControlTH()
{
  //t_Control.attach_us(&doControlCb, CTRL_LOOP);
  while (!platform._stop)
  {
    // if (true) {
      platform.step();   
      // flagControl=false;
    // }
  }
}


int main()
{
    platform.init();
    th_Control.start(doControlTH);
    //t_Comm.attach_us(&doCommCb, 1000);
    while (1)
    {
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
