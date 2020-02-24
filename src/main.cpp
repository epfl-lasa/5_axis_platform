#include <mbed.h>
#include <Platform.h>
#include <definitions.h>

#if (BOARD==NUCLEO64)
  #include <rtos.h>
#endif

Platform platform;
// Thread th_Control;

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
//     th_Control.start(doControlTH);
//     while (1)
//     {
//         platform.communicateToRos();
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
