#include <stdio.h>
#include "BodyThermometer_config.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "clock.h"
#include "osal.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include <string.h>
#include "gatt_server.h"
#include "sensor.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/** Set the watchdog reload interval in [s] = (WDT_LOAD + 3) / (clock frequency in Hz). */
#define RC32K_FREQ		32768
#define RELOAD_TIME(sec)        ((sec*RC32K_FREQ)-3)

#define SENSOR_TIMER 0
#define SENSOR_READ_DURATION_MS 50
static uint16_t sensor_update_rate = 5000;
static volatile uint8_t sensorTimer_expired = FALSE;

static void APP_Init(void)
{
  sensor_init();
  Add_Live_Sensor_Service();
  HAL_VTimerStart_ms(SENSOR_TIMER, sensor_update_rate);
}

void APP_Tick(void)
{
  static uint8_t sensor_read_request_active = 0;
  int res;

  if(sensorTimer_expired) {
    sensorTimer_expired = FALSE;
    if(sensor_read_request_active) {
      sensor_read_request_active = FALSE;
      res = HAL_VTimerStart_ms(SENSOR_TIMER, sensor_update_rate);
      if (res != BLE_STATUS_SUCCESS) {
        while(1);
      }
      int16_t temperature = sensor_read_temperature();
      Temp_Update(temperature);
    } else {
      sensor_measure();
      sensor_read_request_active = TRUE;
      res = HAL_VTimerStart_ms(SENSOR_TIMER, SENSOR_READ_DURATION_MS);
      if (res != BLE_STATUS_SUCCESS) {
        while(1);
      }
    }
  }
}

void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
  if (timerNum == SENSOR_TIMER) {
    sensorTimer_expired = TRUE;
  }
}

void WDG_Reload(void)
{
  WDG_SetReload(RELOAD_TIME(180));
}


int main(void)
{
  uint8_t ret;

  SystemInit();
  Clock_Init();
  /* Reenable watchdog that should have been enabled by ServiceManager beforehand */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_WDG, ENABLE);
  WDG_Reload();

  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    /* TODO: Reboot */
    while(1);
  }

  ret = BLEDeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    /* TODO: Reboot */
    while(1);
  }

  APP_Init();

  while(1) {
    BTLE_StackTick();
    BLETick();

    APP_Tick();

    BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
  }
}
