#include <stdint.h>
uint8_t BLEDeviceInit(void);
void BLETick(void);
tBleStatus Add_Services(void);
tBleStatus Temp_Update(int16_t temp);
tBleStatus Battery_Update(int16_t voltage);
void enableConnectionMode(void);
void enablePairingMode(void);
