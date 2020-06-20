#include "mainpp.h"

#include "dwt_stm32_delay.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "Platform.h"
#include "definitions.h"

Platform platform;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  platform._nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  platform._nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  platform.init();
}

void loop(void)
{
  platform.communicateToRos();
  DWT_Delay_us(2000);
}

