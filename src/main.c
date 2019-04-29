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
#include "led.h"
#include "timer.h"
#include "adc.h"
#include "app.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#define ASSERT_FALSE() while(1) { BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0); }
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define ASSERT_FALSE() NVIC_SystemReset()
#define PRINTF(...)
#endif



/** Set the watchdog reload interval in [s] = (WDT_LOAD + 3) / (clock frequency in Hz). */
#define RC32K_FREQ		32768
#define RELOAD_TIME(sec)        ((sec*RC32K_FREQ)-3)

#define LED_FLASH_ON_MS 20
#define LED_FLASH_OFF_MS 1500
#define LED_FAST_FLASH_ON_MS 20
#define LED_FAST_FLASH_OFF_MS 300
#define LED_BLINK_ON_MS 150
#define LED_BLINK_OFF_MS 400

#define SENSOR_READ_DURATION_MS 30
static volatile uint32_t sensor_update_rate = 15000;
/* recorder tracks time even when measurement is disabled. Notify time counter in this interval.
Interval is limited to 52xxxxx due to internal resolution/overflow.
Also, I had effects that look like overflows in the calculation, use less than half the allowed range
to work around any potential sign-issues. */
#define STANDBY_RECORDER_TIME_UPDATE_RATE 2600000
static uint32_t next_sensor_interval;

static volatile LedMode ledMode;
static volatile uint8_t ledState;
static volatile uint8_t ledCount;

volatile int app_flags;

void led_init(void) {
  GPIO_InitType gpio;
  /* Enable the GPIO Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

  /* Configure LED as output */
  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode = GPIO_Output;
  gpio.GPIO_Pull = DISABLE;
  gpio.GPIO_HighPwr = DISABLE;
  GPIO_Init(&gpio);
}

static void led_set_ll(uint8_t state) {
  if(state) {
    GPIO_SetBits(GPIO_Pin_0);
    ledState = 1;
  } else {
    GPIO_ResetBits(GPIO_Pin_0);
    ledState = 0;
  }
}

static void led_handle(void) {
  uint32_t nextTime;
  if(ledState == 0) {
    switch(ledMode) {
      case LED_BLINK:
      nextTime = LED_BLINK_ON_MS;
      break;
      case LED_FLASH:
      nextTime = LED_FLASH_ON_MS;
      break;
      case LED_FAST_FLASH:
      nextTime = LED_FAST_FLASH_ON_MS;
      break;
      case LED_OFF:
      default:
      return;
    }
    led_set_ll(1);
    HAL_VTimerStart_ms(LED_TIMER, nextTime);
    if(ledCount) {
      if(--ledCount == 0) {
        ledMode = LED_OFF;
      }
    }
  } else {
     switch(ledMode) {
      case LED_BLINK:
      nextTime = LED_BLINK_OFF_MS;
      break;
      case LED_FLASH:
      nextTime = LED_FLASH_OFF_MS;
      break;
      case LED_FAST_FLASH:
      nextTime = LED_FAST_FLASH_OFF_MS;
      break;
      case LED_OFF:
      default:
      led_set_ll(0);
      return;
    }
    led_set_ll(0);
    HAL_VTimerStart_ms(LED_TIMER, nextTime);
  }
}

/* set LED mode. If count is non-zero, only play pattern count times. */
void led_set(LedMode mode, uint8_t count) {
  HAL_VTimer_Stop(LED_TIMER);
  ledMode = mode;
  ledCount = count;
  led_set_ll(0);
  led_handle();
}

#if DEBUG
static void uart_putc(uint8_t tx_data)
{
  while (UART_GetFlagStatus(UART_FLAG_TXFF) == SET);
  UART_SendData(tx_data);
}
#endif

void _debug(char *msg)
{
  #if DEBUG
  while(*msg) {
    uart_putc(*(msg++));
  }
  #endif
}

void _debug_int(uint32_t num)
{
  #if DEBUG
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
  #endif
}

void _debug_uint8(uint8_t num)
{
  #if DEBUG
  char buf[3];
  buf[2] = 0;
  for(int i = 0; i < 2; ++i) {
    int j = (num >> 4*i) & 0x0F;
    if(j < 10) {
      buf[1-i] = '0' + j;
    } else {
      buf[1-i] = 'A' + j - 10;
    }
  }
  debug(buf);
  #endif
}

void uart_init(void)
{
  #if DEBUG
  UART_InitType UART_InitStructure;

  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);

  GPIO_InitType GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);

  UART_InitStructure.UART_BaudRate = 460800;
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Tx;
  UART_InitStructure.UART_FifoEnable = ENABLE;
  UART_Init(&UART_InitStructure);

  UART_Cmd(ENABLE);
  #endif
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
      if(pressDuration > 25000) {
        NVIC_SystemReset();
      } else if(pressDuration > 6000) {
        /* activate bluetooth pairing */
        debug("PAIR\n");
        APP_FLAG_SET(SET_PUBLIC_CONNECTABLE);
      } else if(pressDuration > 1500) {
        if(APP_FLAG(MEASUREMENT_ENABLED)) {
          APP_FLAG_SET(REQUEST_DISABLE_RECORDING);
        } else {
          APP_FLAG_SET(REQUEST_ENABLE_RECORDING);
        }
      } else if(pressDuration > 300) {
        /* (de)activate bluetooth connection */
        if(APP_FLAG(CONNECTABLE) || APP_FLAG(CONNECTED)) {
          APP_FLAG_SET(REQUEST_DISABLE_BLE);
        } else {
          debug("CONN\n");
          APP_FLAG_SET(SET_DIRECTED_CONNECTABLE);
        }
      }
    }
    lastState = state;
  }
}


void enable_measurement(void)
{
  sensor_init();
  if(!APP_FLAG(SENSOR_UNAVAILABLE)) {
    next_sensor_interval = HAL_VTimerGetCurrentTime_sysT32();
    /* initially measure sensor now */
    APP_FLAG_SET(MEASUREMENT_ENABLED);
    HAL_VTimerStart_ms(SENSOR_TIMER, 100);
  }
}

static void APP_Init(void)
{
  led_init();
  recorder_ring_clear();
  Add_Services();
  enable_measurement();
}


void disable_measurement(void)
{
  APP_FLAG_CLEAR(MEASUREMENT_ENABLED);
  APP_FLAG_CLEAR(SENSOR_MEASUREMENT_TRIGGERED);
  sensor_deinit();
  recorder_disable_recording();
}

void set_measurement_interval(uint16_t seconds)
{
  /* update recorder with measurement interval _previous_ to this update */
  recorder_set_recording_interval(get_measurement_interval());
  sensor_update_rate = seconds * 1000;
}

uint16_t get_measurement_interval(void)
{
  return sensor_update_rate / 1000;
}

static void trigger_sensor_measurement(void) {
  if(sensor_measure() != SENSOR_SUCCESS) {
    APP_FLAG_SET(SENSOR_UNAVAILABLE);
    debug("trigger sensor fail\n");
  } else {
    APP_FLAG_SET(SENSOR_MEASUREMENT_TRIGGERED);
    /* Use sensor timer for conversion timing. Do not alter sensor interval timing to be able to achieve constant period. */
    HAL_VTimerStart_ms(SENSOR_TIMER, SENSOR_READ_DURATION_MS);
  }
}

static void APP_Tick(void)
{
  if(APP_FLAG(REQUEST_DISABLE_RECORDING)) {
    APP_FLAG_CLEAR(REQUEST_DISABLE_RECORDING);
    disable_measurement();
    APP_FLAG_CLEAR(REQUEST_DISABLE_RECORDING);
    led_set(LED_FAST_FLASH, 2);
  }
  if(APP_FLAG(REQUEST_ENABLE_RECORDING)) {
    APP_FLAG_CLEAR(REQUEST_ENABLE_RECORDING);
    enable_measurement();
    recorder_enable_recording();
    led_set(LED_FAST_FLASH, 4);
  }
  if(APP_FLAG(CONNECTED) && !APP_FLAG(MEASUREMENT_ENABLED)) {
    /* always measure while a connection is active */
    enable_measurement();
  }
  if(!APP_FLAG(CONNECTED) && !APP_FLAG(RECORDING_ENABLED)) {
    /* stop measurement again when disconnected and not recording */
    disable_measurement();
  }
  if(APP_FLAG(ADC_START_BATTERY_MEASUREMENT_TIME)) {
    APP_FLAG_CLEAR(ADC_START_BATTERY_MEASUREMENT_TIME);
    HAL_VTimerStart_ms(ADC_TIMER, BATTERY_MEASUREMENT_TIME);
  }
  if(APP_FLAG(SENSOR_TIMER_ELAPSED)) {
    APP_FLAG_CLEAR(SENSOR_TIMER_ELAPSED);
    if(APP_FLAG(MEASUREMENT_ENABLED) && !APP_FLAG(SENSOR_UNAVAILABLE)) {
      if(APP_FLAG(SENSOR_MEASUREMENT_TRIGGERED)) {
        APP_FLAG_CLEAR(SENSOR_MEASUREMENT_TRIGGERED);
        debug("S READ\n");
        int16_t temperature = sensor_read_temperature();
        SensorError res = sensor_get_error(0);
        if(res != SENSOR_SUCCESS) {
          debug("S ERR\n");
          /* This temperature result is invalid. Sensor code already reset hardware, but there is no result available at this point */
          if(res == SENSOR_ERROR_PERSISTENT) {
            debug("SENSOR UNVAILABLE\n");
            APP_FLAG_SET(SENSOR_UNAVAILABLE);
          }
          if(res == SENSOR_ERROR_TEMPORARY) {
            debug("retrigger sensor after error\n");
            trigger_sensor_measurement();
          }
        } else {
          Temp_Update(temperature, next_sensor_interval);
          /* step absolute sensor interval for constant period */
          next_sensor_interval = HAL_VTimerAcc_sysT32_ms(next_sensor_interval, sensor_update_rate);
          debug("sens int\n");
          HAL_VTimerStart_sysT32(SENSOR_TIMER, next_sensor_interval);
        }
      } else {
        trigger_sensor_measurement();
      }
    } else {
      Recorder_notify_time(next_sensor_interval);
      next_sensor_interval = HAL_VTimerAcc_sysT32_ms(next_sensor_interval, STANDBY_RECORDER_TIME_UPDATE_RATE);
      HAL_VTimerStart_sysT32(SENSOR_TIMER, next_sensor_interval);
    }
  }
  if(APP_FLAG(SENSOR_UNAVAILABLE)) {
    disable_measurement();
  }
  if(APP_FLAG(HANDLE_LED)) {
    APP_FLAG_CLEAR(HANDLE_LED);
    led_handle();
  }

}

/* timer callback will be called out of BTLE_StackTick before all BTLE handling is done.
It is still unclear to me if calling BTLE methods out of here is okay - but it seems to work.
(BLE timers are processed first and we might change expectations between timer op and stack execution?)
Doing any type of timer activity should be alright - but is also not really documented anywhere.
(Do we change the internal timer list while it is being iterated?) */
void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
  if (timerNum == SENSOR_TIMER) {
    debug("S");
    APP_FLAG_SET(SENSOR_TIMER_ELAPSED);
    debug("\n");
  }
  if(timerNum == LED_TIMER) {
    debug("L\n");
    APP_FLAG_SET(HANDLE_LED);
  }
  if(timerNum == ADC_TIMER) {
    debug("A\n");
    if(APP_FLAG(ADC_LOAD_CONVERSION_IN_PROGRESS)) {
      if(adc_result_ready()) {
        Battery_Load_Update(adc_read_battery());
      } else {
        Battery_Load_Update(0);
      }
      APP_FLAG_CLEAR(ADC_LOAD_CONVERSION_IN_PROGRESS);
    } else if(APP_FLAG(ADC_IDLE_CONVERSION_IN_PROGRESS)) {
      if(adc_result_ready()) {
        Battery_Idle_Update(adc_read_battery());
      } else {
        Battery_Idle_Update(0);
      }
      APP_FLAG_CLEAR(ADC_IDLE_CONVERSION_IN_PROGRESS);
    } else if(APP_FLAG(ADC_IDLE_CONVERSION_REQUEST)) {
      APP_FLAG_CLEAR(ADC_IDLE_CONVERSION_REQUEST);
      adc_trigger_read_battery();
      APP_FLAG_SET(ADC_IDLE_CONVERSION_IN_PROGRESS);
      APP_FLAG_SET(ADC_START_BATTERY_MEASUREMENT_TIME);
    }
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

  debug("Init bluenrg...\n");
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
    if(BlueNRG_WakeupSource() == WAKEUP_IO11) {
      button_handle();
      if(APP_FLAG(ADC_IDLE_CONVERSION_REQUEST) && !APP_FLAG(ADC_LOAD_CONVERSION_IN_PROGRESS) && !APP_FLAG(CONNECTABLE) && !APP_FLAG(CONNECTED)) {
        /* No radio activity, button pressed so likely there was none lately, assume mostly idle and do battery level conversion */
        /* Actually, this does not make sense as the button press is a quite battery draining activity. Rethink at some point :-) */
        APP_FLAG_CLEAR(ADC_IDLE_CONVERSION_REQUEST);
        APP_FLAG_SET(ADC_IDLE_CONVERSION_IN_PROGRESS);
        adc_trigger_read_battery();
        HAL_VTimerStart_ms(ADC_TIMER, BATTERY_MEASUREMENT_TIME);
      }
    }

    APP_Tick();
    WDG_Reload();
    /* Careful: I have had wakeup problems with SLEEPMODE_NOTIMER. After SLEEPMODE_NOTIMER has been used, later sleeps with
    SLEEPMODE_WAKETIMER did not wake the system when Vtimers elapsed. This was reproducable ~1/3 of the time. Button wakeup still
    worked, but vtimer timings seemed really broken. Similarly broken behaviour has not been observed without NOTIMER.
    I don't know if there is a bug in the library somewhere or I did another mistake. Anyway, by keeping WAKETIMER active, we get
    time tracking for free. That's a cool feature, so accept it without further digging. */
    BlueNRG_Sleep(SLEEPMODE_WAKETIMER, WAKEUP_IO11, WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK);
    /* After a wakeup, BTLE_StackTick should always happen first so do not execute any code at the end of the main loop */
  }
}

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
  if(APP_FLAG(SENSOR_TIMER_ELAPSED)) {
    /* we might have no timer active but would activate it next cycle */
    return SLEEPMODE_RUNNING;
  }
  #if DEBUG
  if(UART_GetFlagStatus(UART_FLAG_BUSY)) {
    return SLEEPMODE_CPU_HALT;
  }
  #endif
  if(!GPIO_ReadBit(GPIO_Pin_11) || ledState || APP_FLAG(ADC_IDLE_CONVERSION_IN_PROGRESS) || APP_FLAG(ADC_LOAD_CONVERSION_IN_PROGRESS)) {
    /* Button is pressed: Deepsleep would not happen, but CPU_HALT can wait for (GPIO) interrupt
       led is on: need to keep GPIO peripheral active
       ADC conversion running: need to keep ADC active */
    return SLEEPMODE_CPU_HALT;
  }
  if(APP_FLAG(MEASUREMENT_ENABLED) || ledMode) {
    return SLEEPMODE_WAKETIMER;
  }

  return SLEEPMODE_NOTIMER;
}
