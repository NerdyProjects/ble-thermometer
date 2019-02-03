#include <stdio.h>
#include "BodyThermometer_config.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "clock.h"
#include "osal.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include "OTA_btl.h"
#include "BlueNRG1_flash.h"
#include "BlueNRG1_i2c.h"
#include <string.h>
#include "bmp280.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define SENSOR_TIMER 1
static uint16_t temperature_update_rate = 5000;
#define SENSOR_READ_DURATION_MS 50
static volatile uint8_t sensorTimer_expired = FALSE;

static uint16_t envSensServHandle, tempCharHandle;

static volatile uint16_t connection_handle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;
Char_Desc_Uuid_t char_desc_uuid;

static volatile uint8_t set_connectable = 1;
struct bmp280_dev bmp;

#define SDK_EVAL_I2C                  I2C2

#define SDK_EVAL_I2C_DATA_PIN         GPIO_Pin_5
#define SDK_EVAL_I2C_DATA_MODE        Serial0_Mode

#define SDK_EVAL_I2C_CLK_PIN          GPIO_Pin_4
#define SDK_EVAL_I2C_CLK_MODE         Serial0_Mode

#define SDK_EVAL_I2C_IRQ_HANDLER        I2C2_Handler
 
#define SDK_EVAL_I2C_DMA_TX                 DMA_CH7
#define SDK_EVAL_I2C_DMA_TX_ITTC            DMA_FLAG_TC7

#define SDK_EVAL_I2C_DMA_RX                 DMA_CH6
#define SDK_EVAL_I2C_DMA_RX_ITTC            DMA_FLAG_TC6

#define SDK_EVAL_I2C_BASE                   I2C2_BASE

#define I2C_TX_DR_BASE_ADDR                (SDK_EVAL_I2C_BASE + 0x10)

#define I2C_RX_DR_BASE_ADDR                (SDK_EVAL_I2C_BASE + 0x18)

#define COPY_ENV_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x42,0x82,0x1a,0x40, 0xe4,0x77, 0x11,0xe2, 0x82,0xd0, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xa3,0x2e,0x55,0x20, 0xe4,0x77, 0x11,0xe2, 0xa9,0xe3, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/**
 * @brief  Configures I2C interface
 * @param  baudrate I2C clock frequency
 * @retval SUCCESS in case of success, an error code otherwise
 */
ErrorStatus I2CInit(uint32_t baudrate)
{   
  GPIO_InitType GPIO_InitStructure;
  I2C_InitType I2C_InitStruct;
    
  /* Enable I2C and GPIO clocks */
  if(SDK_EVAL_I2C == I2C2) {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_I2C2, ENABLE);
  }
  else {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_I2C1, ENABLE);
  }
      
  /* Configure I2C pins */
  GPIO_InitStructure.GPIO_Pin = SDK_EVAL_I2C_CLK_PIN ;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_I2C_CLK_MODE;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = SDK_EVAL_I2C_DATA_PIN;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_I2C_DATA_MODE;  
  GPIO_Init(&GPIO_InitStructure);
      
  /* Configure I2C in master mode */
  I2C_StructInit(&I2C_InitStruct);
  I2C_InitStruct.I2C_OperatingMode = I2C_OperatingMode_Master;
  I2C_InitStruct.I2C_ClockSpeed = baudrate;
  I2C_Init((I2C_Type*)SDK_EVAL_I2C, &I2C_InitStruct);
  
  /* Clear all I2C pending interrupts */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MSK);
  
  return SUCCESS;
}


/**
 * @brief  I2C function to write registers of a slave device
 * @param  pBuffer buffer contains data to write
 * @param  DeviceAddr the I2C slave address
 * @param  RegisterAddr register address
 * @param  NumByteToRead number of byte to write
 * @retval SUCCESS in case of success, an error code otherwise
 */
ErrorStatus SdkEvalI2CWrite(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t NumByteToWrite)
{
  I2C_TransactionType t;
  
  /* Write the slave address */
  t.Operation = I2C_Operation_Write;
  t.Address = DeviceAddr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = 1+NumByteToWrite;
  
  /* Flush the slave address */
  I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);
  while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);
  
  /* Begin transaction */
  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);

  /* Fill TX FIFO with data to write */
  I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, RegisterAddr);

  for(uint8_t i=0; i<NumByteToWrite;i++) {
    I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, pBuffer[i]);
  }
  
  /* Wait loop */
  do {
    if(I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ABORTED)
      return ERROR;
   
  } while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD) == RESET);
    
  /* Clear pending bits */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);

  return SUCCESS;
}


/**
 * @brief  I2C function to read registers from a slave device
 * @param  pBuffer buffer to retrieve data from a slave
 * @param  DeviceAddr the I2C slave address
 * @param  RegisterAddr register address
 * @param  NumByteToRead number of byte to read
 * @retval SUCCESS in case of success, an error code otherwise
 */
ErrorStatus SdkEvalI2CRead(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t NumByteToRead)
{
  I2C_TransactionType t;
  
  /* Write the slave address */
  t.Operation = I2C_Operation_Write;
  t.Address = DeviceAddr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Disable;
  t.Length = 1;  
  
  /* Flush the slave address */
  I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);
  while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);
    
  /* Begin transaction */
  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
	
  /* Fill TX FIFO with data to write */
  I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, RegisterAddr);

  /* Wait loop */
  do {
    if(I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ABORTED)
      return ERROR;
    
  } while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS) == RESET);
  
  /* Clear pending bits */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS);
  
  /* read data */
  t.Operation = I2C_Operation_Read;
  t.Address = DeviceAddr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = NumByteToRead;  
  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
  
  /* Wait loop */
  do {
    if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
      return ERROR;
    if (RESET == I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_RXFE) && NumByteToRead)
    {
      /* Already read from I2C FIFO while read is in progress, could read more than fifo length */
      *pBuffer = I2C_ReceiveData((I2C_Type*)SDK_EVAL_I2C);
      pBuffer ++;
      NumByteToRead --;
    }
  } while (RESET == I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD));
  
  /* Clear pending bits */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);
  
  /* Get data from RX FIFO */
  while(NumByteToRead--) {
    *pBuffer = I2C_ReceiveData((I2C_Type*)SDK_EVAL_I2C);
    pBuffer ++;
  }
  
  return SUCCESS;
}

/* stubs to support library that calls into SDK code */
void SdkEvalLedInit(uint8_t led) {

}

void SdkEvalLedOff(uint8_t led) {

}

void SdkEvalLedOn(uint8_t led) {
  
}

/*******************************************************************************
* Function Name  : Add_Environmental_Sensor_Service
* Description    : Add the Environmental Sensor service.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_Environmental_Sensor_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  uint16_t uuid16;
  charactFormat charFormat;
  uint16_t descHandle;

  COPY_ENV_SENS_SERVICE_UUID(uuid);
  
  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 5, &envSensServHandle); 
  if (ret != BLE_STATUS_SUCCESS) goto fail;
   
  /* Temperature Characteristic */

  COPY_TEMP_CHAR_UUID(uuid);  
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
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
  ret = aci_gatt_update_char_value_ext(connection_handle, envSensServHandle, tempCharHandle, 1,2, 0, 2, (uint8_t*)&temp);
  if (ret != BLE_STATUS_SUCCESS){
          return BLE_STATUS_ERROR ;
  }
  
  return BLE_STATUS_SUCCESS;
	
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

int8_t BMP_I2C_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  ErrorStatus res = SdkEvalI2CRead(data, dev_id, reg_addr, len);
  if(res != SUCCESS)
  {
    while(1)
      ;
  }
  return res;
}

int8_t BMP_I2C_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  ErrorStatus res = SdkEvalI2CWrite(data, dev_id, reg_addr, len);
  if(res != SUCCESS)
  {
    while(1)
      ;
  }
  return res;
}

static void APP_Init(void)
{
  struct bmp280_config conf;
  int8_t res;

  I2CInit(100000);
  bmp.dev_id = BMP280_I2C_ADDR_SEC;
  bmp.intf = BMP280_I2C_INTF;
  bmp.read = BMP_I2C_Read;
  bmp.write = BMP_I2C_Write;
  bmp.delay_ms = Clock_Wait;

  res = bmp280_init(&bmp);
  if(res != BMP280_OK) {
    while(1)
      ;
  }

  res = bmp280_get_config(&conf, &bmp);
  if(res != BMP280_OK) {
    while(1)
      ;
  }

  /* Overwrite the desired settings */
  conf.filter = BMP280_FILTER_OFF;
  conf.os_pres = BMP280_OS_NONE;
  conf.os_temp = BMP280_OS_8X;
  conf.odr = BMP280_ODR_4000_MS;

  res = bmp280_set_config(&conf, &bmp);
  if(res != BMP280_OK) {
    while(1)
      ;
  }

  /* Always set the power mode after setting the configuration */
  res = bmp280_set_power_mode(BMP280_SLEEP_MODE, &bmp);
  if(res != BMP280_OK) {
    while(1)
      ;
  }

  Add_Environmental_Sensor_Service();
  HAL_VTimerStart_ms(SENSOR_TIMER, temperature_update_rate);
}

void APP_Tick(void)
{
  static uint8_t sensor_read_request_active = 0;
  int res;
  if(set_connectable) {
    setConnectable();
    set_connectable = 0;
  }

  if(sensorTimer_expired) {
    sensorTimer_expired = FALSE;
    if(sensor_read_request_active) {
      sensor_read_request_active = FALSE;
      res = HAL_VTimerStart_ms(SENSOR_TIMER, temperature_update_rate);
      if (res != BLE_STATUS_SUCCESS) {
        while(1);
      }
      struct bmp280_uncomp_data ucomp_data;
      res = bmp280_get_uncomp_data(&ucomp_data, &bmp);
      if(res != BMP280_OK) {
        while(1)
          ;
      }
      int32_t temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
      if(connection_handle) {
        Temp_Update(temp32);
      }      
    } else {
      res = bmp280_set_power_mode(BMP280_FORCED_MODE, &bmp);
      if(res != BMP280_OK) {
        while(1)
          ;
      }
      sensor_read_request_active = TRUE;
      res = HAL_VTimerStart_ms(SENSOR_TIMER, SENSOR_READ_DURATION_MS);
      if (res != BLE_STATUS_SUCCESS) {
        while(1);
      }
    }
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

void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
  if (timerNum == SENSOR_TIMER) {
    sensorTimer_expired = TRUE;
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

  APP_Init();

  while(1) {
    BTLE_StackTick();

    APP_Tick();

    if (OTA_Tick() == 1)
    {
      OTA_Jump_To_New_Application();
    }

    //BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
  }
}
