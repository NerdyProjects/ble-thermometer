#include <stdio.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "osal.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include <string.h>
#include "gatt_server.h"

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

static uint16_t envSensServHandle, tempCharHandle;
volatile uint16_t connection_handle;

static volatile uint8_t set_connectable = 1;


/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;
Char_Desc_Uuid_t char_desc_uuid;

#define COPY_LIVE_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x42,0x82,0x1a,0x40, 0xe4,0x77, 0x11,0xe2, 0x82,0xd0, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xa3,0x2e,0x55,0x20, 0xe4,0x77, 0x11,0xe2, 0xa9,0xe3, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

void setConnectable(void)
{  
  /* Set Name as OTAServiceMgr */
  uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','o','d','y','T','h','e','r','m','o','m','e','t','e','r'};
  
  /* Add OTA service UUID to scan response */
  hci_le_set_scan_response_data(0, NULL); 
  
  aci_gap_set_discoverable(ADV_IND, 0, 0, RANDOM_ADDR, NO_WHITE_LIST_USE,
                           16, local_name, 0, NULL, 0, 0);    
}

uint8_t BLEDeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t name[] = {'T','h','e','r','m','o','m','e','t','e','r'};
  
  {
    uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
    
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                              bdaddr);
  }
  
  ret = aci_gatt_init();    
  
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0x00, sizeof(name), &service_handle, 
                    &dev_name_char_handle, &appearance_char_handle);

  ret = aci_gatt_update_char_value_ext(0,service_handle, dev_name_char_handle,0,sizeof(name),0, sizeof(name), name);
  
  ret = aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
  //ret = aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_NOT_REQUIRED, SC_IS_SUPPORTED, KEYPRESS_IS_NOT_SUPPORTED, 0x07, 0x10, DONOT_USE_FIXED_PIN_FOR_PAIRING, 123456, PUBLIC_ADDR);

  /* -2 dBm output power */
  aci_hal_set_tx_power_level(1,4);
  
  return ret;
  
}

tBleStatus Add_Live_Sensor_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  uint16_t uuid16;
  charactFormat charFormat;
  uint16_t descHandle;

  COPY_LIVE_SENS_SERVICE_UUID(uuid);
  
  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 5, &envSensServHandle); 
  if (ret != BLE_STATUS_SUCCESS) goto fail;
   
  /* Temperature Characteristic */

  COPY_TEMP_CHAR_UUID(uuid);  
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ | CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                     16, 0, &tempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  charFormat.format = FORMAT_SINT16;
  charFormat.exp = -2;
  charFormat.unit = UNIT_TEMP_CELSIUS;
  charFormat.name_space = 0;
  charFormat.desc = 0;

  uuid16 = CHAR_FORMAT_DESC_UUID;

  Osal_MemCpy(&char_desc_uuid.Char_UUID_16, &uuid16, 2);

  ret = aci_gatt_add_char_desc(envSensServHandle,
                         tempCharHandle,
                         UUID_TYPE_16,
                         &char_desc_uuid, 
                         7,
                         7,
                         (void *)&charFormat, 
                         ATTR_PERMISSION_NONE,
                         ATTR_ACCESS_READ_ONLY,
                         0,
                         16,
                         FALSE,
                         &descHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  return BLE_STATUS_SUCCESS; 
  
fail:
  while(1) {
    
  }
  return BLE_STATUS_ERROR;

}

/*******************************************************************************
* Function Name  : Temp_Update
* Description    : Update temperature characteristic value
* Input          : temperature in hundreds of degree
* Return         : Status.
*******************************************************************************/
tBleStatus Temp_Update(int16_t temp)
{  
  tBleStatus ret;
  ret = aci_gatt_update_char_value_ext(0, envSensServHandle, tempCharHandle, 1,2, 0, 2, (uint8_t*)&temp);
  if (ret != BLE_STATUS_SUCCESS){
          return BLE_STATUS_ERROR ;
  }
  
  return BLE_STATUS_SUCCESS;
	
}

void BLETick(void) {
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
  connection_handle = Connection_Handle;

  
}/* end hci_le_connection_complete_event() */


/* This function is called when the peer device get disconnected.
*/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  /* Make the device connectable again. */
  set_connectable = TRUE;
  connection_handle = 0;
  
}/* end hci_disconnection_complete_event() */

void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{

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

}