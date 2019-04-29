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
#include "led.h"
#include "adc.h"
#include "timer.h"
#include "app.h"


#define BLE_GATT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT 0x2728

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

static uint16_t envSensServHandle, tempCharHandle, battIdleCharHandle, battLoadCharHandle;
static uint16_t recorderServHandle, recorderControlCharHandle, recorderStatusCharHandle, recorderDataCharHandle;
volatile uint16_t connection_handle;

static uint32_t connectableAt;
static volatile uint32_t connectedAt;
/* terminate a connection after this time */
static volatile int32_t max_connection_time = 3*60000;
static volatile int32_t max_connectable_time = 2*60000;

#define DATA_PACKET_SIZE 20

#define RECORDER_RING_SIZE 4096
static uint16_t recorderRing[RECORDER_RING_SIZE];
/* pointer to next write position, e.g. oldest to be invalidated value */
static uint16_t recorderRingWritePos;

static uint16_t recorderReadLast;
static uint16_t recorderReadNext;

static uint32_t recorderLastTimestamp;
static uint32_t lastTemperatureSecondsAgo;

#ifndef GIT_HASH
#error "GIT_HASH should be defined"
#endif

/* UUIDS */
#define COPY_LIVE_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x0e,0xed,0x81,0x8d,0xb4,0xb3,0x6e,0x04,0x39,0x57,0xce,0x09,0x68,0x84,0xca,0x85)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x44,0xc4,0x6f,0xa8,0xd6,0xaa,0xb1,0xfd,0xc3,0x46,0x22,0x6a,0x0a,0xf3,0xf3,0x83)
#define COPY_IDLE_BATT_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x74,0xbc,0x5e,0x06,0xa4,0x03,0x85,0x40,0xcc,0xf2,0xe2,0x7c,0x6e,0xe6,0x6c,0x16)
#define COPY_LOAD_BATT_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x67,0xc0,0x50,0xa2,0x74,0x24,0xf8,0x6d,0xee,0x22,0x93,0x79,0xa2,0x38,0xc3,0x75)

#define COPY_RECORDER_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x89,0x65,0xc8,0xd0,0xb5,0x43,0x96,0x95,0x64,0x14,0x78,0x96,0x70,0xac,0x1c,0xf7)
#define COPY_RECORDER_CONTROL_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x8e,0x92,0x1d,0x16,0xa9,0x29,0x4c,0x0c,0x41,0xed,0x8e,0x40,0xfa,0x37,0x01,0x3f)
#define COPY_RECORDER_STATUS_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0xdf,0xc5,0x43,0x32,0x8d,0xda,0x7f,0x88,0x26,0x99,0x63,0x04,0x83,0xc2,0xfd,0x99)
#define COPY_RECORDER_DATA_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x8c,0x81,0x71,0x50,0xdf,0xc8,0xf4,0x5d,0x7d,0x8e,0xe8,0xa5,0x39,0xc4,0xa1,0x01)

#define RECORDER_CMD_READ 0x1
#define RECORDER_CMD_STOP_RECORDING 0x2
#define RECORDER_CMD_START_RECORDING 0x3
#define RECORDER_CMD_SET_RECORDING_INTERVAL 0x4
#define RECORDER_CMD_RESET_STORAGE 0x5
#define RECORDER_CMD_RESET_TO_OTA 0x6
#define RECORDER_CMD_SET_CONNECTABLE_TIMEOUT 0x7
#define RECORDER_CMD_SET_CONNECTION_TIMEOUT 0x8
#define RECORDER_CMD_SET_RECONNECTABLE 0x9
#define RECORDER_CMD_RESET_RECONNECTABLE 0x10
#define RECORDER_CMD_READ_TIME 0x11

#define RECORDER_META_INVALID 0x0
#define RECORDER_META_STOP_RECORDING 0x0002
#define RECORDER_META_START_RECORDING 0x0003
#define RECORDER_META_RECORDING_INTERVAL 0x1000 /* 0x1000-0x1FFF -> 12 bit */
#define RECORDER_META_TIME_ELAPSED 0x4000 /* 0x4000-0x7FFF -> 14 bits. If used twice without other dataset in between, interpreted as 28 bit number, LSB first. */
#define RECORDER_META_MASK 0x8000

/* maximum measurement interval comes from theoretical maximum of data type, theoretical maximum of timer and observed overflow problems.
If needed, could implement a software timer extension to allow longer times, so far, these ~43 minutes should be enough */
#define MAX_MEASUREMENT_INTERVAL_S 2580
#define MAX_TIMEOUT_INTERVAL_S 2580

#define RECORDER_TIME_ELAPSED_MAX_VALUE 0x3FFF

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
static void ring_push(uint16_t data)
{
  recorderRing[recorderRingWritePos] = data;
  recorderRingWritePos = ring_addr_add(recorderRingWritePos, 1);
}

void ring_push_data(uint16_t data)
{
  ring_push(data & (RECORDER_META_MASK-1));
}

void ring_push_meta(uint16_t data)
{
  ring_push(data | RECORDER_META_MASK);
}

void recorder_enable_recording(void) {
  APP_FLAG_SET(RECORDING_ENABLED);
  APP_FLAG_SET(STORE_RECORD_TIME_DIFFERENCE);
  ring_push_meta(RECORDER_META_START_RECORDING);
}

void recorder_disable_recording(void) {
  if(APP_FLAG(RECORDING_ENABLED)) {
    APP_FLAG_CLEAR(RECORDING_ENABLED);
    ring_push_meta(RECORDER_META_STOP_RECORDING);
  }
}

void recorder_set_recording_interval(uint16_t interval_seconds) {
  ring_push_meta(RECORDER_META_RECORDING_INTERVAL | interval_seconds);
}

void recorder_ring_clear(void) {
  uint8_t reenable_recording = APP_FLAG(RECORDING_ENABLED);
  disable_measurement();
  for(uint16_t i = 0; i < RECORDER_RING_SIZE; ++i) {
    ring_push_meta(RECORDER_META_INVALID);
  }
  if(reenable_recording) {
    enable_measurement();
    recorder_enable_recording();
  }
}

static uint16_t ring_count_used(void) {
  uint16_t pos = ring_addr_add(recorderRingWritePos, -1);
  uint16_t used = 0;
  while(recorderRing[pos] != (RECORDER_META_MASK | RECORDER_META_INVALID) && used <= RECORDER_RING_SIZE) {
    used++;
    pos = ring_addr_add(pos, -1);
  }
  return used;
}

/* go back in the ring the given number of seconds. Returns oldest used ring address, when the data is not old enough. */
static uint16_t ring_get_past_addr(int32_t seconds) {
  uint16_t start = ring_addr_add(recorderRingWritePos, -1);
  uint16_t next = start;
  seconds -= lastTemperatureSecondsAgo;
  uint16_t measurementInterval = get_measurement_interval();
  while(seconds > 0) {
    uint16_t v = recorderRing[next];
    if(v & RECORDER_META_MASK) {
      if(v & RECORDER_META_RECORDING_INTERVAL) {
        measurementInterval = v & 0x0FFF;
      }
      if(v & RECORDER_META_TIME_ELAPSED) {
        uint32_t elapsed = v & 0x3FFF;
        uint16_t peek = recorderRing[ring_addr_add(next, -1)];
        if((peek & RECORDER_META_MASK) && (peek & RECORDER_META_TIME_ELAPSED)) {
          /* two packet time elapsed: older (this one) is LSB */
          elapsed = (elapsed << 14) | (peek & 0x3FFF);
          next = ring_addr_add(next, -1);
        }
        seconds -= elapsed;
      }
      if(v == (RECORDER_META_MASK | RECORDER_META_INVALID)) {
        /* invalid packet marks end of recorded data -> stop here */
        return ring_addr_add(next, 1);
      }
    } else {
      seconds -= measurementInterval;
    }
    next = ring_addr_add(next, -1);
    if(next == start) {
      /* worked through ring without finding old enough point -> return whole ring */
      return ring_addr_add(next, 1);
    }
  }
  return ring_addr_add(next, 1);
}

void handle_recorder_control(uint8_t *data, uint16_t length) {
  if(length >= 3) {
    uint16_t v = data[1] + (data[2] << 8);
    if(data[0] == RECORDER_CMD_READ) {
      uint16_t readLength = v;
      recorderReadLast = ring_addr_add(recorderRingWritePos, -readLength);
      recorderReadNext = ring_addr_add(recorderRingWritePos, -1);
      APP_FLAG_SET(READ_RECORDER_BUFFER);
      APP_FLAG_SET(TRIGGER_DATA_TRANSFER);
    }
    if(data[0] == RECORDER_CMD_READ_TIME) {
      int32_t readSeconds = v * 60;
      recorderReadNext = ring_addr_add(recorderRingWritePos, -1);
      recorderReadLast = ring_get_past_addr(readSeconds);
      APP_FLAG_SET(READ_RECORDER_BUFFER);
      APP_FLAG_SET(TRIGGER_DATA_TRANSFER);
    }
    if(data[0] == RECORDER_CMD_START_RECORDING) {
      APP_FLAG_SET(REQUEST_ENABLE_RECORDING);
    }
    if(data[0] == RECORDER_CMD_STOP_RECORDING) {
      APP_FLAG_SET(REQUEST_DISABLE_RECORDING);
    }
    if(data[0] == RECORDER_CMD_SET_RECORDING_INTERVAL) {
      uint16_t interval_seconds = v;
      if(interval_seconds >= 1 && interval_seconds < MAX_MEASUREMENT_INTERVAL_S) {
        set_measurement_interval(interval_seconds);
      }
    }
    if(data[0] == RECORDER_CMD_RESET_STORAGE) {
      APP_FLAG_SET(REQUEST_RING_CLEAR);
    }
    if(data[0] == RECORDER_CMD_RESET_TO_OTA) {
      NVIC_SystemReset();
    }
    if(data[0] == RECORDER_CMD_SET_CONNECTION_TIMEOUT) {
      if(v < MAX_TIMEOUT_INTERVAL_S) {
        max_connection_time = v * 1000;
      }
    }
    if(data[0] == RECORDER_CMD_SET_CONNECTABLE_TIMEOUT) {
      if(v < MAX_TIMEOUT_INTERVAL_S) {
        max_connectable_time = v * 1000;
      }
    }
    if(data[0] == RECORDER_CMD_SET_RECONNECTABLE) {
      APP_FLAG_SET(ALLOW_CONNECT_AFTER_DISCONNECT);
    }
    if(data[0] == RECORDER_CMD_RESET_RECONNECTABLE) {
      APP_FLAG_CLEAR(ALLOW_CONNECT_AFTER_DISCONNECT);
    }
  }
}

void setConnectable(void)
{
  tBleStatus ret;

  hci_le_set_scan_response_data(0, NULL);

  ret = aci_gap_set_undirected_connectable(0x0400, 0x0800, STATIC_RANDOM_ADDR, 0x00);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("ERR set_discoverable ");
    debug_int(ret);
    debug("\n");
  } else {
    led_set(LED_FAST_FLASH, 0);
    APP_FLAG_SET(CONNECTABLE);
  }
}

void setPairable(void)
{
  tBleStatus ret;
  uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','o','d','y','T','h','e','r','m','o','m','e','t','e','r'};

  hci_le_set_scan_response_data(0, NULL);

  ret = aci_gap_set_discoverable(ADV_IND, 0x0800, 0x0800, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                           16, local_name, 0, NULL, 80, 512);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("ERR set_discoverable");
    debug_int(ret);
    debug("\n");
  } else {
    led_set(LED_BLINK, 0);
    APP_FLAG_SET(CONNECTABLE);
    APP_FLAG_SET(PAIRABLE);
  }
}

uint8_t BLEDeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t name[] = {'T','h','e','r','m','o','m','e','t','e','r'};

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
  ret = aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_REQUIRED, SC_IS_SUPPORTED, KEYPRESS_IS_NOT_SUPPORTED, 0x07, 0x10, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0, STATIC_RANDOM_ADDR);
  if(ret != BLE_STATUS_SUCCESS) {
    debug("GAP SET AUTH FAILED\n");
  }

  aci_hal_set_radio_activity_mask(0x0007);
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

  debug("Add Services\n");

  COPY_LIVE_SENS_SERVICE_UUID(uuid);

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 12, &envSensServHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add live service\n");
    debug_int(ret);
    goto fail;
  }

  /* Temperature Characteristic */

  COPY_TEMP_CHAR_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ | CHAR_PROP_NOTIFY, ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_READ, 0,
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

  /* Battery idle level characteristic */
  COPY_IDLE_BATT_CHAR_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                     16, 0, &battIdleCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add batt idle char\n");
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
                         battIdleCharHandle,
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
    debug("fail add batt idle desc\n");
    debug_int(ret);
    goto fail;
  }
  /* Battery load level characteristic */
  COPY_LOAD_BATT_CHAR_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                     16, 0, &battLoadCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add batt load char\n");
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
                         battLoadCharHandle,
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
    debug("fail add batt load desc\n");
    debug_int(ret);
    goto fail;
  }

  /* recorder service */
  COPY_RECORDER_SERVICE_UUID(uuid);
  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 11, &recorderServHandle);
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
  /* Status characteristic */
  COPY_RECORDER_STATUS_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret = aci_gatt_add_char(recorderServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_READ, ATTR_PERMISSION_AUTHEN_READ, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &recorderStatusCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    debug("fail add recorder status\n");
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
  debug("BLE setup error\n");
  return BLE_STATUS_ERROR;
}

static void Temp_Record(int16_t temp, uint32_t timestamp)
{
  if(APP_FLAG(STORE_RECORD_TIME_DIFFERENCE)) {
    uint32_t seconds;
    APP_FLAG_CLEAR(STORE_RECORD_TIME_DIFFERENCE);
    /* update seconds elapsed calculation */
    Recorder_notify_time(timestamp);
    seconds = lastTemperatureSecondsAgo;
    if(seconds > RECORDER_TIME_ELAPSED_MAX_VALUE) {
      /* store use time elapsed tag twice to increment bit width, LSB first */
      ring_push_meta(RECORDER_META_TIME_ELAPSED | (seconds & RECORDER_TIME_ELAPSED_MAX_VALUE));
      ring_push_meta(RECORDER_META_TIME_ELAPSED | (seconds / (RECORDER_TIME_ELAPSED_MAX_VALUE + 1)));
    } else {
      ring_push_meta(RECORDER_META_TIME_ELAPSED | seconds);
    }
  }
  recorderLastTimestamp = timestamp;
  lastTemperatureSecondsAgo = 0;
  ring_push_data(temp);
}

/* Notify the recorder about a time that has elapsed. This is used to track time between
measurements: when recording is stopped and started again, the time difference between the last
notification before the last measurement to the last notification before the first-again measurement
is added to the ring.

internal timestamp to ms:
ms = 5*ts/2048
*/
void Recorder_notify_time(uint32_t absoluteTime)
{
  lastTemperatureSecondsAgo += HAL_VTimerDiff_ms_sysT32(absoluteTime, recorderLastTimestamp) / 1000;
  recorderLastTimestamp = absoluteTime;
}

/* Notify recorder and characteristics about updated temperature.
Timestamp should match approximately the time of the measurement so the recorder can keep
track of time intervals in between recordings stopped/started */
tBleStatus Temp_Update(int16_t temp, uint32_t timestamp)
{
  tBleStatus ret;
  ret = aci_gatt_update_char_value_ext(0, envSensServHandle, tempCharHandle, 1,2, 0, 2, (uint8_t*)&temp);
  if (ret != BLE_STATUS_SUCCESS){
          return BLE_STATUS_ERROR;
  }

  if(APP_FLAG(RECORDING_ENABLED)) {
    Temp_Record(temp, timestamp);
  }

  return BLE_STATUS_SUCCESS;
}

/* Updates idle battery voltage in millivolt */
tBleStatus Battery_Idle_Update(int16_t voltage)
{
  tBleStatus ret;
  ret = aci_gatt_update_char_value_ext(0, envSensServHandle, battIdleCharHandle, 1,2, 0, 2, (uint8_t*)&voltage);
  if (ret != BLE_STATUS_SUCCESS){
          return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/* Updates load battery voltage in millivolt */
tBleStatus Battery_Load_Update(int16_t voltage)
{
  tBleStatus ret;
  ret = aci_gatt_update_char_value_ext(0, envSensServHandle, battLoadCharHandle, 1,2, 0, 2, (uint8_t*)&voltage);
  if (ret != BLE_STATUS_SUCCESS){
          return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/* ToDo: Make sure that a device can always be connected again if flag says so */
static void disconnect(void) {
  APP_FLAG_CLEAR(REQUEST_DISABLE_BLE);
  if(APP_FLAG(CONNECTABLE)) {
    aci_gap_set_non_discoverable();
    APP_FLAG_CLEAR(CONNECTABLE);
  } else if(APP_FLAG(CONNECTED)) {
    aci_gap_terminate(connection_handle, 0x15);
    APP_FLAG_CLEAR(CONNECTED);
  }
  led_set(LED_OFF, 0);
}

void BLETick(void) {
  tBleStatus ret;
  uint32_t time = HAL_VTimerGetCurrentTime_sysT32();
  if(APP_FLAG(SET_PUBLIC_CONNECTABLE)) {
    if(APP_FLAG(CONNECTABLE)) {
      aci_gap_set_non_discoverable();
    }
    debug("Pair\n");
    setPairable();
    APP_FLAG_CLEAR(SET_PUBLIC_CONNECTABLE);
    connectableAt = time;
  } else if(APP_FLAG(SET_DIRECTED_CONNECTABLE)) {
    if(APP_FLAG(CONNECTABLE)) {
      aci_gap_set_non_discoverable();
    }
    debug("Connect\n");
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
      led_set(LED_FLASH, 0);
    } else {
      debug("SECURITY REQ ERR\n");
    }
  }

  if(APP_FLAG(REQUEST_DISABLE_BLE)) {
    disconnect();
  }

  if(APP_FLAG(CONNECTABLE) && max_connectable_time && HAL_VTimerDiff_ms_sysT32(time, connectableAt) > max_connectable_time) {
    disconnect();
  }

  if(APP_FLAG(CONNECTED) && max_connection_time && HAL_VTimerDiff_ms_sysT32(time, connectedAt) > max_connection_time) {
    disconnect();
  }

  if(APP_FLAG(REQUEST_RING_CLEAR)) {
    APP_FLAG_CLEAR(REQUEST_RING_CLEAR);
    recorder_ring_clear();
  }

  if(!APP_FLAG(TX_BUFFER_FULL) && APP_FLAG(READ_RECORDER_BUFFER) && connection_handle && (APP_FLAG(CONFIRMATION_RECEIVED) || APP_FLAG(TRIGGER_DATA_TRANSFER))) {
    uint16_t buf[DATA_PACKET_SIZE / 2];
    uint8_t done = 0;
    uint16_t recorderReadNext_retry = recorderReadNext;
    for(uint8_t i = 0; i < DATA_PACKET_SIZE/2; ++i) {
      buf[i] = recorderRing[recorderReadNext];
      if(recorderReadNext == recorderReadLast) {
        done = 1;
      }
      recorderReadNext = ring_addr_add(recorderReadNext, -1);
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

static void updateRecorderStatus(void) {
  uint16_t ring_used_space = ring_count_used();
  uint16_t max_connection_time_s = max_connection_time / 1000;
  uint16_t max_connectable_time_s = max_connectable_time / 1000;
  uint16_t sensor_update_rate_s = get_measurement_interval();
  uint32_t last_temperature_update_s = lastTemperatureSecondsAgo + HAL_VTimerDiff_ms_sysT32(HAL_VTimerGetCurrentTime_sysT32(), recorderLastTimestamp)/1000;
  uint32_t git_hash = GIT_HASH;
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 0, 4, &app_flags);
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 4, 2, &ring_used_space);
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 6, 2, &max_connection_time_s);
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 8, 2, &max_connectable_time_s);
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 10, 2, &sensor_update_rate_s);
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 12, 4, &last_temperature_update_s);
  aci_gatt_update_char_value(recorderServHandle, recorderStatusCharHandle, 16, 4, &git_hash);
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
  APP_FLAG_CLEAR(CONNECTABLE);
  APP_FLAG_SET(CONNECTED);
  debug("CONN");
  debug_uint8(Peer_Address[0]);
  debug(":");
  debug_uint8(Peer_Address[1]);
  debug(":");
  debug_uint8(Peer_Address[2]);
  debug(":");
  debug_uint8(Peer_Address[3]);
  debug(":");
  debug_uint8(Peer_Address[4]);
  debug(":");
  debug_uint8(Peer_Address[5]);
  debug(" ");
  debug_uint8(Peer_Address_Type);
  debug("\n");
  APP_FLAG_SET(START_GAP_SLAVE_SECURITY_REQUEST);
  APP_FLAG_SET(ADC_LOAD_CONVERSION_REQUEST);
  APP_FLAG_SET(ADC_IDLE_CONVERSION_REQUEST);
  connectedAt = HAL_VTimerGetCurrentTime_sysT32();
}/* end hci_le_connection_complete_event() */


/* This function is called when the peer device get disconnected.
*/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connection_handle = 0;
  APP_FLAG_CLEAR(CONNECTED);
  disconnect();
  if(APP_FLAG(ALLOW_CONNECT_AFTER_DISCONNECT)) {
    APP_FLAG_SET(SET_DIRECTED_CONNECTABLE);
  }
}/* end hci_disconnection_complete_event() */

void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  debug("mod");
  debug_int(Attr_Handle);
  debug(" ");
  debug_int(Offset);
  debug(" ");
  debug_int(Attr_Data_Length);
  for(uint8_t i = 0; i < Attr_Data_Length; ++i) {
    debug(" ");
    debug_uint8(Attr_Data[i]);
  }
  debug("\n");
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
  if(Attribute_Handle == recorderStatusCharHandle + 1) {
    updateRecorderStatus();
  }
  aci_gatt_allow_read(Connection_Handle);
}

void aci_gap_slave_security_initiated_event(void)
{
  APP_FLAG_SET(REQUEST_LOW_POWER_CONNECTION_PARAMETERS);
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
  if(Last_State == 0x01 || Last_State == 0x02 || Last_State == 0x05) {
    /* just had a load-situation: Measure load battery level */
    if(APP_FLAG(ADC_LOAD_CONVERSION_REQUEST) && !APP_FLAG(ADC_IDLE_CONVERSION_IN_PROGRESS)) {
      APP_FLAG_CLEAR(ADC_LOAD_CONVERSION_REQUEST);
      APP_FLAG_SET(ADC_LOAD_CONVERSION_IN_PROGRESS);
      adc_trigger_read_battery();
      APP_FLAG_SET(ADC_START_BATTERY_MEASUREMENT_TIME);
    }
  }
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
      debug("pass_resp OK\n");
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
