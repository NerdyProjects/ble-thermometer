#include "BlueNRG1_conf.h"
#include "bmp280.h"
#include "bluenrg1_hal.h"
#include "hal_types.h"
#include "clock.h"
#include "sensor.h"

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

static SensorError sensorError = SENSOR_NOT_INITIALIZED;
/* in case of failure, reset I2C and reinit sensor this amount of times before giving up */
#define SENSOR_ERROR_RETRY_INIT_COUNT 2
static uint8_t error_reinit_count = 0;

/* returns and optionally clears sensor error flags */
SensorError sensor_get_error(uint8_t clear) {
  SensorError ret = sensorError;
  if(clear) {
    sensorError = SENSOR_SUCCESS;
  }
  return ret;
}

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

int8_t BMP_I2C_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  ErrorStatus res = SdkEvalI2CRead(data, dev_id, reg_addr, len);
  if(res != SUCCESS)
  {
     debug("Error reading I2C\n");
     sensorError = SENSOR_ERROR_TEMPORARY;
  }
  return res;
}

int8_t BMP_I2C_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  ErrorStatus res = SdkEvalI2CWrite(data, dev_id, reg_addr, len);
  if(res != SUCCESS)
  {
    debug("Error writing I2C\n");
    sensorError = SENSOR_ERROR_TEMPORARY;
  }
  return res;
}

void sensor_deinit(void) {
  I2C_DeInit(SDK_EVAL_I2C);
  /* Enable I2C and GPIO clocks */
  if(SDK_EVAL_I2C == I2C2) {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C2, DISABLE);
  }
  else {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C1, DISABLE);
  }
}

SensorError sensor_init(void) {
  struct bmp280_config conf;
  int8_t res;

  debug("SENSOR init\n");

  I2CInit(100000);
  bmp.dev_id = BMP280_I2C_ADDR_SEC;
  bmp.intf = BMP280_I2C_INTF;
  bmp.read = BMP_I2C_Read;
  bmp.write = BMP_I2C_Write;
  bmp.delay_ms = Clock_Wait;

  res = bmp280_init(&bmp);
  if(res != BMP280_OK) {
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;
    res = bmp280_init(&bmp);
    if(res != BMP280_OK) {
      debug("could not find BMP on any I2C address\n");
      goto fail;
    } else {
      debug("Found BMP on prim. Addr\n");
    }
  } else {
    debug("Found BMP on sec. Addr\n");
  }

  res = bmp280_get_config(&conf, &bmp);
  if(res != BMP280_OK) {
    goto fail;
  }

  /* Overwrite the desired settings */
  conf.filter = BMP280_FILTER_OFF;
  conf.os_pres = BMP280_OS_NONE;
  conf.os_temp = BMP280_OS_8X;
  conf.odr = BMP280_ODR_4000_MS;

  res = bmp280_set_config(&conf, &bmp);
  if(res != BMP280_OK) {
    goto fail;
  }

  /* Always set the power mode after setting the configuration */
  res = bmp280_set_power_mode(BMP280_SLEEP_MODE, &bmp);
  if(res != BMP280_OK) {
    goto fail;
  }

  error_reinit_count = 0;
  sensorError = SENSOR_SUCCESS;
  return SENSOR_SUCCESS;

  fail:
  if(error_reinit_count < SENSOR_ERROR_RETRY_INIT_COUNT) {
    error_reinit_count++;
    return sensor_init();
  } else {
    sensorError = SENSOR_ERROR_NOT_FOUND;
    debug("Sensor permanent error: not found\n");
    return SENSOR_ERROR_NOT_FOUND;
  }
}

SensorError sensor_measure(void) {
  uint8_t res = bmp280_set_power_mode(BMP280_FORCED_MODE, &bmp);
  if(res != BMP280_OK) {
    if(error_reinit_count < SENSOR_ERROR_RETRY_INIT_COUNT) {
      if(sensor_init() != SENSOR_SUCCESS) {
        return SENSOR_ERROR_PERSISTENT;
      }
      error_reinit_count++;
      return sensor_measure();
    }
    sensorError = SENSOR_ERROR_PERSISTENT;
    debug("Sensor permanent error: failed to measure\n");
    return SENSOR_ERROR_PERSISTENT;
  }
  error_reinit_count = 0;
  return SENSOR_SUCCESS;
}

/* reads temperature from sensor. If there is any read errors,
it returns -32768 (0x8000), reinitializes the sensor and sets an error code.
Application must take care to properly handle this case. */
int16_t sensor_read_temperature(void) {
  uint8_t res;
  struct bmp280_uncomp_data ucomp_data;
  res = bmp280_get_uncomp_data(&ucomp_data, &bmp);
  if(res != BMP280_OK) {
    if(sensor_init() == SENSOR_SUCCESS) {
      sensorError = SENSOR_ERROR_TEMPORARY;
    } else {
      debug("Sensor permanent error: failed while readout\n");
      sensorError = SENSOR_ERROR_PERSISTENT;
    }
    return 0x8000;
  }  
  return bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
}