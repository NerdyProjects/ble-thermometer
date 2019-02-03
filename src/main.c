#include <stdio.h>
#include "BodyThermometer_config.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "clock.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include "OTA_btl.h"
#include "BlueNRG1_flash.h"
#include <string.h>

static volatile uint8_t set_connectable = 1;

/* stubs to support library that calls into SDK code */
void SdkEvalLedInit(uint8_t led) {

}

void SdkEvalLedOff(uint8_t led) {

}

void SdkEvalLedOn(uint8_t led) {
  
}

static void setConnectable(void)
{  
  /* Set Name as OTAServiceMgr */
  uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'O','T','A','S','e','r','v','i','c','e','M','g','r'};
  
  /* Add OTA service UUID to scan response */
  hci_le_set_scan_response_data(18,BTLServiceUUID4Scan); 
  
  aci_gap_set_discoverable(ADV_IND, 0, 0, RANDOM_ADDR, NO_WHITE_LIST_USE,
                           14, local_name, 0, NULL, 0, 0);    
}

void APP_Tick(void)
{
  if(set_connectable) {
    setConnectable();
    set_connectable = 0;
  }
}

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
  
  /* -2 dBm output power */
  aci_hal_set_tx_power_level(1,4);
  
  return ret;
  
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
  OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);      
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


int main(void)
{
  uint8_t ret;

  SystemInit();

  Clock_Init();

  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    /* TODO: Reboot */
    while(1);
  }

  ret = OTA_ServiceManager_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    /* TODO: Reboot */
    while(1);
  }

  while(1) {
    BTLE_StackTick();

    APP_Tick();

    if (OTA_Tick() == 1)
    {
      OTA_Jump_To_New_Application();
    }

    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
  }
}
