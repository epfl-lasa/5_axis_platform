#include "Platform.h"
#include "definitions.h"

//! 1
void Platform::resetEscons(){

  totalEffortDClear(-1);
  setEfforts();
  wait_us(250000);
  _enableMotors->write(1);
  wait_us(750000);
  _enableMotors->write(0);
}

//! 2
void Platform::softReset(){
   NVIC_SystemReset(); 
}