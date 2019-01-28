#include <stdio.h>
#include "BodyThermometer_config.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include "OTA_btl.h"

/* stubs to support library that calls into SDK code */
void SdkEvalLedInit(uint8_t led) {

}

void SdkEvalLedOff(uint8_t led) {

}

void SdkEvalLedOn(uint8_t led) {
  
}

void APP_Tick(void)
{

}


int main(void)
{
  uint8_t ret;

  SystemInit();

  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    /* TODO: Reboot */
  }

  while(1) {
    BTLE_StackTick();

    APP_Tick();

    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
    if (OTA_Tick() == 1)
    {
      OTA_Jump_To_New_Application();
    }
  }
}
