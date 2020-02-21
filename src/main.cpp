#include <mbed.h>
#include <Platform.h>
#include <definitions.h>

#if (BOARD==NUCLEO64)
  #include <rtos.h>
#endif

Platform platform;

// Ticker t_Comm;
// Thread th_Control;
// volatile bool flagComm = false;

// void doCommCB()
// {
//   flagComm = true;
// }
// //! Thread for Communication
// void doControlTH()
// {
//   while (!platform._stop)
//   {
//     platform.step();
//   }
// }


// int main()
// {
//     platform.init();
//     t_Comm.attach_us(&doCommCB, COMM_LOOP);
//     th_Control.start(doControlTH);
//     while (1)
//     {
//       if (flagComm)
//       {
//         platform.communicateToRos();
//         flagComm=false;
//       }
//     }
//   return 0;
//   }

int main()
{
  platform.init();
  while (!platform._stop)
  {
    platform.step();
    platform.communicateToRos();
  }
  return 0;
}
