#include <stdint.h>
uint8_t BLEDeviceInit(void);
void BLETick(void);
tBleStatus Add_Services(void);
tBleStatus Temp_Update(int16_t temp);
tBleStatus Battery_Load_Update(int16_t voltage);
tBleStatus Battery_Idle_Update(int16_t voltage);
void enableConnectionMode(void);
void enablePairingMode(void);
void recorder_enable_measurement(void);
void recorder_disable_measurement(void);
void recorder_set_measurement_interval(uint16_t interval_seconds);
void recorder_ring_clear(void);