#include <mbed.h>
#include <Platform.h>
#include <definitions.h>
#include <rtos.h>

Platform platform;
Thread t_Control;

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