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

static void uart_putc(uint8_t tx_data)
{
  while (UART_GetFlagStatus(UART_FLAG_TXFF) == SET);
  UART_SendData(tx_data);
}

void debug(char *msg)
{
  while(*msg) {
    uart_putc(*(msg++));
  }
}

void uart_init(void)
{
  UART_InitType UART_InitStructure;
  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);
  
  GPIO_InitType GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);

  UART_InitStructure.UART_BaudRate = 115200;
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Tx;
  UART_InitStructure.UART_FifoEnable = ENABLE;
  UART_Init(&UART_InitStructure);

  UART_Cmd(ENABLE);
}

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
  uart_init();
  /* Reenable watchdog that should have been enabled by ServiceManager beforehand */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_WDG, ENABLE);
  WDG_Reload();

  debug("Init bluenrg stack...\n");
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

  debug("Init app...\n");
  APP_Init();

  while(1) {
    BTLE_StackTick();
    BLETick();

    APP_Tick();

    BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
  }
}
