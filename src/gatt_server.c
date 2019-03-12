#include <stdio.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "osal.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include <string.h>
#include "gatt_server.h"
#include "app_flags.h"

#define BLE_GATT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT 0x2728

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

static uint16_t envSensServHandle, tempCharHandle, battCharHandle;
static uint16_t recorderServHandle, recorderControlCharHandle, recorderDataCharHandle;
volatile uint16_t connection_handle;

static int32_t connectableAt;

#define DATA_PACKET_SIZE 20

#define RECORDER_RING_SIZE 4096
static uint16_t recorderRing[RECORDER_RING_SIZE];
/* pointer to next write position, e.g. oldest to be invalidated value */
static uint16_t recorderRingWritePos;

static uint16_t recorderReadLast;
static uint16_t recorderReadNext;

/* UUIDS */
#define COPY_LIVE_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x0e,0xed,0x81,0x8d,0xb4,0xb3,0x6e,0x04,0x39,0x57,0xce,0x09,0x68,0x84,0xca,0x85)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x44,0xc4,0x6f,0xa8,0xd6,0xaa,0xb1,0xfd,0xc3,0x46,0x22,0x6a,0x0a,0xf3,0xf3,0x83)
#define COPY_BATT_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x67,0xc0,0x50,0xa2,0x74,0x24,0xf8,0x6d,0xee,0x22,0x93,0x79,0xa2,0x38,0xc3,0x75)

#define COPY_RECORDER_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x89,0x65,0xc8,0xd0,0xb5,0x43,0x96,0x95,0x64,0x14,0x78,0x96,0x70,0xac,0x1c,0xf7)
#define COPY_RECORDER_CONTROL_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x8e,0x92,0x1d,0x16,0xa9,0x29,0x4c,0x0c,0x41,0xed,0x8e,0x40,0xfa,0x37,0x01,0x3f)
#define COPY_RECORDER_DATA_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x8c,0x81,0x71,0x50,0xdf,0xc8,0xf4,0x5d,0x7d,0x8e,0xe8,0xa5,0x39,0xc4,0xa1,0x01)

#define RECORDER_CMD_READ 0x1

static uint16_t ring_addr_add(uint16_t addr, int16_t offset)
{
  assert_param(offset > -RECORDER_RING_SIZE && offset < RECORDER_RING_SIZE);
  int16_t result = (int16_t)addr + offset;
  if(result < 0) {
    result += RECORDER_RING_SIZE;
  }
  if(result >= RECORDER_RING_SIZE) {
    result -= RECORDER_RING_SIZE;
  }
  return result;
}

/* adds a value to the ring. The ring is not overflow safe, it will always keep RING_SIZE most recent values overwriting the oldest. Data must be overwrite-safe
by itself: Either only ever write 16 bit data or have an in-band mechanism to detect destroyed words.
Application specific: we want to store signed temperature values, 15 bits are enough for that: -163.84-163.83 degrees.
Use a set first bit to store 15 bit meta data, e.g. measurement interval, breaks, ... (tbd) */
void ring_push(uint16_t data)
{
  recorderRing[recorderRingWritePos] = data;
  recorderRingWritePos = ring_addr_add(recorderRingWritePos, 1);
}

void handle_recorder_control(uint8_t *data, uint16_t length) {
  if(length >= 3) {
    if(data[0] == RECORDER_CMD_READ) {
      uint16_t readLength = data[1] + (data[2] << 8);
      recorderReadLast = ring_addr_add(recorderRingWritePos, -1);
      recorderReadNext = ring_addr_add(recorderRingWritePos, -readLength);
      APP_FLAG_SET(READ_RECORDER_BUFFER);
      APP_FLAG_SET(TRIGGER_DATA_TRANSFER);
    }
  }
}

void setConnectable(void)
{  
  tBleStatus ret;
  
  hci_le_set_scan_response_data(0, NULL);
  
  ret = aci_gap_set_undirected_connectable(0x0400, 0x0800, PUBLIC_ADDR, 0x00);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("ERR set_discoverable ");
    debug_int(ret);
    debug("\n");
  } else {
    APP_FLAG_SET(CONNECTABLE);
  }
}

void setPairable(void)
{  
  tBleStatus ret;
  uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','o','d','y','T','h','e','r','m','o','m','e','t','e','r'};
  
  hci_le_set_scan_response_data(0, NULL); 
  
  ret = aci_gap_set_discoverable(ADV_IND, 0x0800, 0x0800, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                           16, local_name, 0, NULL, 80, 512);    
  if(ret != BLE_STATUS_SUCCESS) {
    debug("ERR set_discoverable");
    debug_int(ret);
    debug("\n");
  } else {
    APP_FLAG_SET(CONNECTABLE);
    APP_FLAG_SET(PAIRABLE);
  }
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
  if(ret != BLE_STATUS_SUCCESS) {
    debug("GATT INIT FAILED\n");
  }
  /* -11 dBm output power */
  ret = aci_hal_set_tx_power_level(1,1);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("SET TX POWER FAILED\n");
  }

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0x00, sizeof(name), &service_handle, 
                    &dev_name_char_handle, &appearance_char_handle);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("GAP INIT FAILED\n");
  }

  ret = aci_gatt_update_char_value_ext(0,service_handle, dev_name_char_handle,0,sizeof(name),0, sizeof(name), name);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("NAME SET FAILED\n");
  }
  /* we fake having a display to be able to use the fixed pin auth method */
  ret = aci_gap_set_io_capability(IO_CAP_DISPLAY_ONLY);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("IO CAPAB FAILED\n");
  }
  ret = aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_REQUIRED, SC_IS_SUPPORTED, KEYPRESS_IS_NOT_SUPPORTED, 0x07, 0x10, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0, PUBLIC_ADDR);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("GAP SET AUTH FAILED\n");
  }
  return ret;
  
}

tBleStatus Add_Services(void)
{
  tBleStatus ret;
  Service_UUID_t service_uuid;
  Char_UUID_t char_uuid;
  Char_Desc_Uuid_t char_desc_uuid;
  uint8_t uuid[16];
  uint16_t uuid16;
  charactFormat charFormat;
  uint16_t descHandle;

  COPY_LIVE_SENS_SERVICE_UUID(uuid);
  
  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 8, &envSensServHandle); 
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add live service\n");
    debug_int(ret);
    goto fail;
  }
   
  /* Temperature Characteristic */

  COPY_TEMP_CHAR_UUID(uuid);  
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ | CHAR_PROP_NOTIFY, ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_READ, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                     16, 0, &tempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add temp char\n");
    debug_int(ret);
    goto fail;
  }

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
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add temp desc\n");
    debug_int(ret);
    goto fail;
  }

  /* Battery level characteristic */
  COPY_BATT_CHAR_UUID(uuid);  
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                     16, 0, &battCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add batt char\n");
    debug_int(ret);
    goto fail;
  }

  charFormat.format = FORMAT_SINT16;
  charFormat.exp = -3;
  charFormat.unit = BLE_GATT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT;
  charFormat.name_space = 0;
  charFormat.desc = 0;

  uuid16 = CHAR_FORMAT_DESC_UUID;

  Osal_MemCpy(&char_desc_uuid.Char_UUID_16, &uuid16, 2);

  ret = aci_gatt_add_char_desc(envSensServHandle,
                         battCharHandle,
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
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add batt desc\n");
    debug_int(ret);
    goto fail;
  }

  /* recorder service */
  COPY_RECORDER_SERVICE_UUID(uuid);
  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 10, &recorderServHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add recorder service\n");
    goto fail;
  }
  /* Control characteristic */
  COPY_RECORDER_CONTROL_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret = aci_gatt_add_char(recorderServHandle, UUID_TYPE_128, &char_uuid, 3, CHAR_PROP_WRITE, ATTR_PERMISSION_AUTHEN_WRITE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 0, &recorderControlCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add recorder control\n");
    debug_int(ret);
    goto fail;
  }
  /* Data characteristic */
  COPY_RECORDER_DATA_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret = aci_gatt_add_char(recorderServHandle,
              UUID_TYPE_128,
              &char_uuid,
              20,
              CHAR_PROP_INDICATE | CHAR_PROP_READ,
              ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHOR_READ,
              GATT_DONT_NOTIFY_EVENTS,
              16,
              0,
              &recorderDataCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add recorder data\n");
    debug_int(ret);
    goto fail;
  }

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
          return BLE_STATUS_ERROR;
  }

  ring_push(temp);
  
  return BLE_STATUS_SUCCESS;
}

/* Updates battery voltage in millivolt */
tBleStatus Battery_Update(int16_t voltage)
{
  tBleStatus ret;
  ret = aci_gatt_update_char_value_ext(0, envSensServHandle, battCharHandle, 1,2, 0, 2, (uint8_t*)&voltage);
  if (ret != BLE_STATUS_SUCCESS){
          return BLE_STATUS_ERROR;
  }
  
  return BLE_STATUS_SUCCESS;
}

void BLETick(void) {
  tBleStatus ret;
  uint32_t time = HAL_VTimerGetCurrentTime_sysT32();
  if(APP_FLAG(SET_PUBLIC_CONNECTABLE)) {
    if(APP_FLAG(CONNECTABLE)) {
      aci_gap_set_non_discoverable();
    }
    debug("Pair...\n");
    setPairable();
    APP_FLAG_CLEAR(SET_PUBLIC_CONNECTABLE);
    connectableAt = time;    
  } else if(APP_FLAG(SET_DIRECTED_CONNECTABLE)) {
    if(APP_FLAG(CONNECTABLE)) {
      aci_gap_set_non_discoverable();
    }
    debug("Connect...\n");
    setConnectable();
    APP_FLAG_CLEAR(SET_DIRECTED_CONNECTABLE);
    connectableAt = time;    
  }
  
  if(APP_FLAG(START_GAP_SLAVE_SECURITY_REQUEST))
  {
    ret = aci_gap_slave_security_req(connection_handle);
    if (ret == BLE_STATUS_SUCCESS)
    {
      APP_FLAG_CLEAR(START_GAP_SLAVE_SECURITY_REQUEST);
    } else {
      debug("SECURITY REQ ERR\n");
    }
  }
  
  if(APP_FLAG(CONNECTABLE) && HAL_VTimerDiff_ms_sysT32(time, connectableAt) > 3 * 60000) {
    /* disable connectable if not yet connected */
    ret = aci_gap_set_non_discoverable();
    if(ret != BLE_STATUS_SUCCESS) {
      debug("ERR set non discoverable\n");
    }
    APP_FLAG_CLEAR(CONNECTABLE);
  }

  if(!APP_FLAG(TX_BUFFER_FULL) && APP_FLAG(READ_RECORDER_BUFFER) && connection_handle && (APP_FLAG(CONFIRMATION_RECEIVED) || APP_FLAG(TRIGGER_DATA_TRANSFER))) {
    uint16_t buf[DATA_PACKET_SIZE / 2];
    uint8_t done = 0;
    uint16_t recorderReadNext_retry = recorderReadNext;
    for(uint8_t i = 0; i < DATA_PACKET_SIZE/2; ++i) {
      buf[i] = recorderRing[recorderReadNext];
      if(recorderReadNext == recorderReadLast) {
        done = 1;
      } else {
        recorderReadNext = ring_addr_add(recorderReadNext, 1);
      }
    }
    ret = aci_gatt_update_char_value_ext(connection_handle, recorderServHandle, recorderDataCharHandle, 0x02, DATA_PACKET_SIZE, 0, DATA_PACKET_SIZE, (uint8_t *)buf);
    if(ret != BLE_STATUS_INSUFFICIENT_RESOURCES) {
      if(ret == BLE_STATUS_SUCCESS) {
        APP_FLAG_CLEAR(CONFIRMATION_RECEIVED);
        APP_FLAG_CLEAR(TRIGGER_DATA_TRANSFER);
        debug("TX\n");
      } else {
        debug("ERR update data char value\n");
        debug_int(ret);
      }
      if(done) {
          APP_FLAG_CLEAR(READ_RECORDER_BUFFER);
      }
    } else {
      APP_FLAG_SET(TX_BUFFER_FULL);
      debug("TX BUF full, retry...\n");
      recorderReadNext = recorderReadNext_retry;
    }
    
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
  APP_FLAG_SET(CONNECTED);
  debug("CONNECTION FROM ");
  debug_int(Peer_Address[0]);
  debug(":");
  debug_int(Peer_Address[1]);
  debug(":");
  debug_int(Peer_Address[2]);
  debug(":");
  debug_int(Peer_Address[3]);
  debug(":");
  debug_int(Peer_Address[4]);
  debug(":");
  debug_int(Peer_Address[5]);
  debug(" ");
  debug_int(Peer_Address_Type);
  APP_FLAG_SET(START_GAP_SLAVE_SECURITY_REQUEST);
}/* end hci_le_connection_complete_event() */


/* This function is called when the peer device get disconnected.
*/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  /* Make the device connectable again. */
  connection_handle = 0;
  //APP_FLAG_SET(SET_DIRECTED_CONNECTABLE);  
}/* end hci_disconnection_complete_event() */

void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  if(Attr_Handle == recorderControlCharHandle + 1 && Offset == 0) {
    handle_recorder_control(Attr_Data, Attr_Data_Length);
  }

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

void aci_gap_pass_key_req_event(uint16_t Connection_Handle)
{
  if(APP_FLAG(PAIRABLE)) {
    uint8_t status; 
  
    status = aci_gap_pass_key_resp(Connection_Handle, 10134);
    if (status != BLE_STATUS_SUCCESS) {
      debug("aci_gap_pass_key_resp failed\n");
    }
    else
    {
      debug("aci_gap_pass_key_resp() OK\n");
    }
    APP_FLAG_CLEAR(PAIRABLE);
  }
}

void aci_gap_bond_lost_event(void)
{
  uint8_t ret; 
  
  ret = aci_gap_allow_rebond(connection_handle);
  debug("rebond\n");
}

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
}

void aci_gatt_server_confirmation_event(uint16_t Connection_Handle)
{
  APP_FLAG_SET(CONFIRMATION_RECEIVED);
}