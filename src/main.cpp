#include <mbed.h>
#include <Platform.h>
#include <definitions.h>
#include <rtos.h>

#define INTERRUPTS 1
#define THREADS 2
#define ALL_IN_MAIN 3

#define METHOD ALL_IN_MAIN

Platform platform;

#if (METHOD==INTERRUPTS)
Thread t_Control;
Ticker t_set 

// Control Loop High Priority Thread Running @ 2.5KHz... Since we're limited by the PWM Freq for the ESCON
// This is a thread not an interruption!
void doControl() {
  while(1){
    platform._spi->lock(); //! Lock the SPI for use in the thread
    platform.step();
    platform._spi->unlock();
    wait_us(CTRL_LOOP);
  }
}

int main() 
{
  platform.init();
  t_Control.start(doControl);
  
  while(1) {   
      platform.communicateToRos(); //! This one publishes the message to ROS
      platform._nh.spinOnce(); // For Retrieving and Publishing to ROS. Separate in case we want to put it in an interruption
      //wait_us(COMM_LOOP);
  }
  return 0;
}

#elif (METHOD==INTERRUPTS) //! THE PROBLEM OF USING AN TIMMER INTERRUPT FOR THE ALL_IN_MAIN IS THAT IT WILL PROBABLY NOT HEAR THE PIN INTERRUPTS FOR LIMIT SWITCHES
Ticker t_Control;

void doControl() {
    platform.step(); //! Update the motors without Platform::getMotion()
  }

int main() 
{
  platform.init();
  t_Control.attach_us(&doControl,CTRL_LOOP);
  
  while(1) {   
      platform.getMotion(); //! SPI
      platform.communicateToRos(); //! This one publishes the message to ROS
      platform._nh.spinOnce(); // For Retrieving and Publishing to ROS. Separate in case we want to put it in an interruption
  }
  return 0;
}

#elif (METHOD==ALL_IN_MAIN) //! FASTER FOR THE ROSTOPIC HZ e.g. 224 Hz

int main() 
{
  platform.init();
    
  while(1) {   
      platform.getMotion(); //! SPI
      platform.step();
      platform.communicateToRos(); //! This one publishes the message to ROS
      platform._nh.spinOnce(); // For Retrieving and Publishing to ROS. Separate in case we want to put it in an interruption
  }
  return 0;
}

#endif