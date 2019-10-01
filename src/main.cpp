#include <mbed.h>
#include <Platform.h>
#include <definitions.h>

#if (BOARD==NUCLEO64)
  #include <rtos.h>
#endif

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
    AXES};
#undef ListofAxes

Platform platform;

int main() 
{
  platform.init();  
  while(1) {   
      platform.step();
      platform.communicateToRos(); 
  }
  return 0;
}

