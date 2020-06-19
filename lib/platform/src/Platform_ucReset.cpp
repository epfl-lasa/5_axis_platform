#include "Platform.h"

//! 1
void Platform::resetEscons(){

  totalEffortDClear(-1);
  setEfforts();
  Thread::wait(250);
  _enableMotors->write(1);
  Thread::wait(750);
  _enableMotors->write(0);
}

//! 2
void Platform::softReset(){
   NVIC_SystemReset(); 
}