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
#include "app_flags.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define ASSERT_FALSE() while(1) { BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0); }

/** Set the watchdog reload interval in [s] = (WDT_LOAD + 3) / (clock frequency in Hz). */
#define RC32K_FREQ		32768
#define RELOAD_TIME(sec)        ((sec*RC32K_FREQ)-3)

#define SENSOR_TIMER 0
#define SENSOR_READ_DURATION_MS 50
static uint16_t sensor_update_rate = 5000;
static volatile uint8_t sensorTimer_expired = FALSE;

volatile int app_flags;

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

void debug_int(uint32_t num)
{
  char buf[8];
  buf[7] = 0;
  for(int i = 0; i < 7; ++i) {
    int j = (num >> 4*i) & 0x0F;
    if(j < 10) {
      buf[6-i] = '0' + j;
    } else {
      buf[6-i] = 'A' + j - 10;
    }
  }
  debug(buf);
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

static void adc_init(void)
{
  ADC_InitType adc;
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
  
  /* Configure ADC */
  adc.ADC_OSR = ADC_OSR_200;
  adc.ADC_Input = ADC_Input_BattSensor;
  adc.ADC_ConversionMode = ADC_ConversionMode_Single;
  adc.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
  adc.ADC_Attenuation = ADC_Attenuation_0dB;
  
  ADC_Init(&adc);
  ADC_Filter(ENABLE);
  ADC_AutoOffsetUpdate(ENABLE);
  ADC_Calibration(ENABLE);
}

/* returns the battery voltage in millivolts */
static int16_t adc_read_battery(void)
{
  int16_t rawResult;
  
  // Did not work without reinitialising everytime, as I am a bit lazy right now, let's do it like this
  adc_init();
  ADC_Cmd(ENABLE);
  while(!ADC_GetFlagStatus(ADC_FLAG_EOC))
    ;
  rawResult = ADC_GetRawData();
  
  assert_param(ADC->CONF_b.SKIP == 0);
  ADC_Cmd(DISABLE);
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, DISABLE);
  
  /* The result calculation is fixed for OSR 200, no SKIP, 0.6V bias and 2.4V reference.
     The effective full scale used in the calculation differs between datasheet and library code!
     Still, the implemented solution just dividing rawResult by two gives less than 10 mV error from 1.9-3.4 V
  */
  /*  base formula (lib): 4.36 * (0.6 - ((raw / 19450) * 2.4)) */
  return 2616 - rawResult / 2;
}

static void button_init(void)
{
  GPIO_InitType gpio;
  /* Enable the GPIO Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
    
  /* Configure BUTTON as interrupt sources */ 
  gpio.GPIO_Pin = GPIO_Pin_11;
  gpio.GPIO_Mode = GPIO_Input;
  gpio.GPIO_Pull = ENABLE;
  gpio.GPIO_HighPwr = DISABLE;
  GPIO_Init(&gpio);
  
  /* Set the GPIO interrupt priority and enable it */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configures EXTI line */
  GPIO_EXTIConfigType GPIO_EXTIStructure;
  GPIO_EXTIStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
  GPIO_EXTIStructure.GPIO_Event = GPIO_Event_Both;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);

  /* Clear pending interrupt */
  GPIO_ClearITPendingBit(GPIO_Pin_11);
  
  /* Enable the interrupt */
  GPIO_EXTICmd(GPIO_Pin_11, ENABLE);
}

/* called from button edge interrupt and when button wakeup source detected - detect situation inside.
   Care: We might be in interrupt context.
 */
void button_handle(void) {
  static uint32_t lastPressTime;
  static uint8_t lastState = Bit_SET;
  uint8_t state = GPIO_ReadBit(GPIO_Pin_11);
  /* The key handling is dependent on HAL timer (powered by systick timer). That should work also with sleep modes
  as the keys' state is also a wakeup source, so the time should be increasing while the button is pressed. */
  if(state != lastState) {
    uint32_t time = HAL_VTimerGetCurrentTime_sysT32();
    if(state == Bit_RESET) {
      lastPressTime = time;
    } else {
      int32_t pressDuration = HAL_VTimerDiff_ms_sysT32(time, lastPressTime);
      if(pressDuration > 6000) {
        debug("RST\n");
        NVIC_SystemReset();
      } else if(pressDuration > 3000) {
        /* activate bluetooth pairing */
        debug("PAIR\n");
        APP_FLAG_SET(SET_PUBLIC_CONNECTABLE);
      } else if(pressDuration > 500) {
        /* activate bluetooth connection */
        debug("CONN\n");
        APP_FLAG_SET(SET_DIRECTED_CONNECTABLE);
      }
    }
    lastState = state;
  }
}

static void APP_Init(void)
{
  sensor_init();
  Add_Services();
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
        ASSERT_FALSE();
      }
      int16_t temperature = sensor_read_temperature();
      Temp_Update(temperature);
      Battery_Update(adc_read_battery());
    } else {
      sensor_measure();
      sensor_read_request_active = TRUE;
      res = HAL_VTimerStart_ms(SENSOR_TIMER, SENSOR_READ_DURATION_MS);
      if (res != BLE_STATUS_SUCCESS) {
        ASSERT_FALSE();
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
  /* button is responsible for manual recovery reset. Make sure to keep this before any code that could potentially break anything.
     Recovery reset is done in button interrupt */
  button_init();
  /* Reenable watchdog that should have been enabled by ServiceManager beforehand */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_WDG, ENABLE);
  WDG_Reload();

  debug("Init bluenrg stack...\n");
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    ASSERT_FALSE();
  }

  ret = BLEDeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    ASSERT_FALSE();
  }

  debug("Init app...\n");
  APP_Init();

  while(1) {
    BTLE_StackTick();
    BLETick();

    APP_Tick();
    WDG_Reload();
    BlueNRG_Sleep(SLEEPMODE_WAKETIMER, WAKEUP_IO11, WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK);
    if(BlueNRG_WakeupSource() == WAKEUP_IO11) {
      button_handle();
    }
  }
}

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
  if(UART_GetFlagStatus(UART_FLAG_BUSY)) {
    return SLEEPMODE_CPU_HALT;
  }

  return SLEEPMODE_NOTIMER;
}
