#include "mainpp.h"

#include "dwt_stm32_delay.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "Platform.h"
#include "definitions.h"

Platform platform;
Ticker doCommFlipper;
Thread th_Control;

volatile bool flag_doComm;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  platform._nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  platform._nh.getHardware()->reset_rbuf();
}

void doCommCb()
{
  flag_doComm=true;
}

void doControlTH() {
  while (!platform._stop) {
    platform.step();
  }
}

void setup(void)
{
  platform.init();
  flag_doComm=false;

  doCommFlipper.attach_us(&doCommCb,COMM_LOOP);
  th_Control.start(doControlTH);
}

void loop(void)
{
  if (flag_doComm)
  {
    platform.communicateToRos();
    flag_doComm=false;
  }

}

