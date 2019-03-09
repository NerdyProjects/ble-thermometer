/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : OTA_ServiceManager.c
* Author             : AMS - VMA division
* Version            : V1.0.0
* Date               : 15-January-2016
* Description        : BlueNRG-1 OTA Service Manager APIs.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_conf.h"
#include "bluenrg1_stack.h"
#include "clock.h"
#include "OTA_ServiceManager.h"
#include "ble_const.h"
#include "SDK_EVAL_Config.h"
#include "OTA_btl.h" 
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/* Private macro -------------------------------------------------------------*/
/** Set the watchdog reload interval in [s] = (WDT_LOAD + 3) / (clock frequency in Hz). */
#define RC32K_FREQ		32768
#define RELOAD_TIME(sec)        ((sec*RC32K_FREQ)-3)
/* Private variables ---------------------------------------------------------*/
/* peek into OTA_btl.c NewImageCharHandle to be able to detect a starting flash operation */
extern uint16_t btlNewImageCharHandle;
static volatile uint8_t set_connectable = 1;

/* Private function prototypes -----------------------------------------------*/
/**
* @brief  It check if  flash storage area has to be erased or not
* @param  None.
* @retval Status: 1 (erase flash); 0 (don't erase flash).
*
* @note The API code could be subject to change in future releases.
*/
static uint8_t OTA_Check_Storage_Area(uint32_t start_address, uint32_t end_address)
{
  volatile uint32_t *address; 
  uint32_t i; 
  
  for(i=start_address;i<end_address; i = i +4)
  { 
    address = (volatile uint32_t *) i;
    if (*address != 0xFFFFFFFF)
      return 1; /* do flash erase */
  }
  
  return 0; /* no flash erase is required */
}


/**
* @brief  It erases destination flash erase before starting OTA upgrade session. 
* @param  None.
* @retval None.
*
* @note The API code could be subject to change in future releases.
*/
static void OTA_Erase_Flash(uint16_t startNumber, uint16_t endNumber)
{
  uint16_t k; 
  
  for(k=startNumber;k<=endNumber;k++)
  { 
    FLASH_ErasePage(k); 
  }
  
}

/*******************************************************************************
* Function Name  : setConnectable
* Description    : Enter in connectable mode.
* Input          : None.
* Return         : None.
*******************************************************************************/
static void setConnectable(void)
{  
  /* Set Name as OTAServiceMgr */
  uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'O','T','A','S','e','r','v','i','c','e','M','g','r'};
  
  /* Add OTA service UUID to scan response */
  hci_le_set_scan_response_data(18,BTLServiceUUID4Scan); 
  
  aci_gap_set_discoverable(ADV_IND, 0x0800, 0x0800, RANDOM_ADDR, NO_WHITE_LIST_USE,
                           14, local_name, 0, NULL, 0, 0);    
}


/*******************************************************************************
* Function Name  : OTA_ServiceManager_DeviceInit
* Description    : Init the OTA Service Manager device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t OTA_ServiceManager_DeviceInit(void)
{
  uint8_t ret;
  
  {
    uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
    
    aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                              bdaddr);
  }
  
  ret = aci_gatt_init();    
  
  {
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
    
    aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  
  /* Add OTA bootloader service */
  ret = OTA_Add_Btl_Service();
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("\r\nOTA service added successfully.\n");
  else
    PRINTF("\r\nError while adding OTA service.\n");
  
  /* -2 dBm output power */
  aci_hal_set_tx_power_level(1,4);
  
  return ret;
  
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{
  if(set_connectable) {
    setConnectable();
    set_connectable = 0;
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/* This function is called when there is a LE Connection Complete event.
*/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{ 
  WDG_SetReload(RELOAD_TIME(30));
}/* end hci_le_connection_complete_event() */


/* This function is called when the peer device get disconnected.
*/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  /* Make the device connectable again. */
  set_connectable = TRUE;
  
  OTA_terminate_connection();
  
}/* end hci_disconnection_complete_event() */

void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  if(Attr_Handle == btlNewImageCharHandle + 1)
  {
    /* Erase the storage area from start page to end page if necessary */
    if(OTA_Check_Storage_Area(APP_WITH_OTA_SERVICE_ADDRESS,APP_WITH_OTA_SERVICE_ADDRESS_END)) {
      OTA_Erase_Flash(APP_WITH_OTA_SERVICE_PAGE_NUMBER_START,APP_WITH_OTA_SERVICE_PAGE_NUMBER_END);
    }
    /* We Ignore BLE activity here, hopefully we do not disrupt communication as there might be a
    short pause from higher level programming application anyway */
    /* Disable potential reset to application */
    HAL_VTimer_Stop(0);
  }
  OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);      
  WDG_SetReload(RELOAD_TIME(30));
}

/* aci_gatt_read_permit_req_event()
*/ 
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  aci_gatt_allow_read(Connection_Handle);
}



void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
  if (Next_State == 0x02) /* 0x02: Connection event slave */
  {
    OTA_Radio_Activity(Next_State_SysTime);  
  }
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
