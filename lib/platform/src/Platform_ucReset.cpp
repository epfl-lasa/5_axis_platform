#include "Platform.h"

//! 1
void Platform::resetEscons(){

  totalEffortDClear(-1);
  setEfforts();
  rtos::ThisThread::sleep_for(250);
  _enableMotors->write(1);
  rtos::ThisThread::sleep_for(750);
  _enableMotors->write(0);
}

//! 2
void Platform::softReset(){
   NVIC_SystemReset(); 
}