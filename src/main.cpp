#include <mbed.h>
#include <Platform.h>
#include <definitions.h>

#if (BOARD==NUCLEO64)
  #include <rtos.h>
#endif

Platform platform;

int main() 
{
  platform.init();  
  while(!platform._stop) {   
      platform.step();
      platform.communicateToRos(); 
  }
  return 0;
}

