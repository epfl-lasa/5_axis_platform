#include "mainpp.h"

#include "Platform.h"
#include "definitions.h"
#include "dwt_stm32_delay.h"
#include "ros.h"
#include "std_msgs/String.h"

Platform platform;
Ticker doCommFlipper;
// Thread th_Control;

volatile bool flag_doComm;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  platform._nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  platform._nh.getHardware()->reset_rbuf();
}

void doCommCb()
{
  flag_doComm = true;
}

// void doControlTH() {

// }

void setup(void)
{
  platform.init();
  flag_doComm = false;
  doCommFlipper.attach_us(&doCommCb, COMM_LOOP);
  // th_Control.start(doControlTH);
}

void loop(void)
{
  while (!platform._stop)
  {
    platform.step();

    if (flag_doComm)
    {
      if (platform.waitUntilRosConnect())
      {
        platform._innerCounterSoftReset=0;
        platform._recoveringFromError=false;
        if (!platform._flagLoadParams)
        {
          platform.retrieveParams(ALL);
        }
        else
        {
          platform.communicateToRos();
        }
      }
      else
      {
        platform._flagLoadParams = false;
        
        if (platform._recoveringFromError)
        {
          platform._innerCounterSoftReset++;
        }

        if (platform._innerCounterSoftReset > 4500)
        {
          platform.softReset();
        }
      }
      flag_doComm = false;
    }
  }
}
