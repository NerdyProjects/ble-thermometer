#include <stdint.h>
uint8_t BLEDeviceInit(void);
void BLETick(void);
tBleStatus Add_Live_Sensor_Service(void);
tBleStatus Temp_Update(int16_t temp);