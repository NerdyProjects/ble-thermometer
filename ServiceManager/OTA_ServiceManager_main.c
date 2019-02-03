
/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : OTA_ServiceManager_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 15-January-2016
* Description        : Code demostrating the BLE OTA Service Manager application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_conf.h"
#include "bluenrg1_stack.h"
#include "clock.h"
#include "sleep.h"

#include "OTA_ServiceManager.h"
#include "OTA_ServiceManager_config.h"
#include "ble_const.h"
#include "SDK_EVAL_Config.h"
#include "OTA_btl.h" 
#include "bluenrg1_it_stub.h"


/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);
/* Private define ------------------------------------------------------------*/

#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BLE_OTA_SERVICE_MANAGER_VERSION_STRING "1.0.0" 

/* Private macro -------------------------------------------------------------*/
/** Set the watchdog reload interval in [s] = (WDT_LOAD + 3) / (clock frequency in Hz). */
#define RC32K_FREQ		32768
#define RELOAD_TIME(sec)        ((sec*RC32K_FREQ)-3)
#define APPLICATION_WAIT_MS     60000
/* Private variables ---------------------------------------------------------*/  
extern volatile uint32_t ota_sw_activation;

/* Private function prototypes -----------------------------------------------*/

/**
* @brief  It defines the valid application address where to jump
*         by checking the OTA application validity tags for the lower and
*         higher applications
* @param  None.
* @retval appaddress: the application base address where to jump
*
* @note The API code could be subject to change in future releases.
*/
static uint32_t OTA_Check_Application_Tags_Value(void)
{
  uint32_t appAddress = 0;
  
  if (((TAG_VALUE(APP_OTA_SERVICE_ADDRESS) == OTA_SERVICE_MANAGER_TAG) && (TAG_VALUE(APP_WITH_OTA_SERVICE_ADDRESS) == OTA_IN_PROGRESS_TAG))|| /* 10 */
      ((TAG_VALUE(APP_OTA_SERVICE_ADDRESS) == OTA_SERVICE_MANAGER_TAG) && (TAG_VALUE(APP_WITH_OTA_SERVICE_ADDRESS) == OTA_INVALID_OLD_TAG))) /* 11 */ 
  {
    /* Jump to OTA Service Manager Application */
    appAddress = APP_OTA_SERVICE_ADDRESS;
  }
  else if ((TAG_VALUE(APP_OTA_SERVICE_ADDRESS) == OTA_SERVICE_MANAGER_TAG) && (TAG_VALUE(APP_WITH_OTA_SERVICE_ADDRESS) == OTA_VALID_TAG)) /* 12 */
  {
    /* Jump to Application using OTA Service Manager */
    appAddress = APP_WITH_OTA_SERVICE_ADDRESS;
  }  
  
  return appAddress;
}

/**
* @brief  NVIC configuration
* @param  None
* @retval None
*/
void WDG_Configuration(void)
{
  NVIC_InitType NVIC_InitStructure;
  
  /* Enable watchdog clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_WDG, ENABLE);
  
  /* WDG reload time configuration */
  WDG_SetReload(RELOAD_TIME(30));
  
  /* Clear pending interrupt on cortex */
  NVIC->ICPR[0] = 0xFFFFFFFF;
  
  /* Enable the RTC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = WDG_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void exitBootloader()
{
  pFunction Jump_To_Application;
  uint32_t JumpAddress;
  /* When application goes back to bootloader by reset, make it behave like a system reset */
  ota_sw_activation = OTA_INVALID_OLD_TAG;
  /* enable watchdog before jumping to application to be able to recover from bad firmware */
  WDG_Configuration();
  /* Jump to user application */
  JumpAddress = *(__IO uint32_t*) (APP_WITH_OTA_SERVICE_ADDRESS + 4);
  Jump_To_Application = (pFunction) JumpAddress;
  /* Initialize user application's Stack Pointer */
  __set_MSP(*(__IO uint32_t*) APP_WITH_OTA_SERVICE_ADDRESS);
  Jump_To_Application();
  
  /* Infinite loop */
  while (1)
  {
  }
}

int main(void)
{
  /* flag storing if the code should install a timeout handler to start application
   by resetting with ota_sw_activation flag set to OTA_APP_SWITCH_OP_CODE_GO_TO_NEW_APP
   in some seconds. This allows recovery from broken firmware images, as long as they timeout
   */
  uint8_t resetToApplication = 0;
  uint8_t ret;
    
  /* Check Service manager RAM Location to verify if a jump to Service Manager has been set from the Application */
  switch(ota_sw_activation) {
    case OTA_APP_SWITCH_OP_CODE_GO_TO_OTA_SERVICE_MANAGER:
      break;
    case OTA_APP_SWITCH_OP_CODE_GO_TO_NEW_APP:
    /* go to application immediately */
      if(OTA_Check_Application_Tags_Value() == APP_WITH_OTA_SERVICE_ADDRESS)
      {
        exitBootloader();
      }
      break;
    default:
    /* go to application after a timeout where service manager is active */
      if(OTA_Check_Application_Tags_Value() == APP_WITH_OTA_SERVICE_ADDRESS)
      {
        resetToApplication = 1;
      }
      break;
  }
  
  /* Here Ota Service Manager Application is started */
  
  /* System Init */
  SystemInit();
  
  Clock_Init();
  
  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  PRINTF("\r\nBlueNRG-1 BLE OTA Service Manager (version: %s)\r\n", BLE_OTA_SERVICE_MANAGER_VERSION_STRING); 
  
  /* Init OTA Service Manager Device */
  ret = OTA_ServiceManager_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("OTA_ServiceManager_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }

  if(resetToApplication) {
    HAL_VTimerStart_ms(0, APPLICATION_WAIT_MS);
  }
  
  while(1)
  {
    /* BlueNRG-1 stack tick */
    BTLE_StackTick();
    
    /* Application tick */
    APP_Tick();
    
    /* Check if the OTA firmware upgrade session has been completed */
    if (OTA_Tick() == 1)
    {
      /* Jump to the new application */
      OTA_Jump_To_New_Application();
    }
    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
  }
  
}

void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
  if (timerNum == 0) {
    /* load application immediately after reset */
    ota_sw_activation = OTA_APP_SWITCH_OP_CODE_GO_TO_NEW_APP;
    NVIC_SystemReset();
  }
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
  return SLEEPMODE_NOTIMER;
}

/***************************************************************************************/

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
