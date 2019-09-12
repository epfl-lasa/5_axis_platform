#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! 1
void Platform::resetEscons(){

  totalEffortDClear(-1);
  setEfforts();
  wait_ms(250);
  _enableMotors->write(1);
  wait_ms(750);
  _enableMotors->write(0);
}

//! 2
void Platform::softReset(){
   NVIC_SystemReset(); 
}