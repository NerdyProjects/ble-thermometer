#include "BlueNRG1_conf.h"
#include "adc.h"

void adc_init(void)
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

void adc_trigger_read_battery(void)
{
  adc_init();
  ADC_Cmd(ENABLE);
}

uint8_t adc_result_ready(void) {
    return ADC_GetFlagStatus(ADC_FLAG_EOC);
}

/* returns the battery voltage in millivolts */
int16_t adc_read_battery(void)
{
  int16_t rawResult;

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