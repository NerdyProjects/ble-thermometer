#include <stdint.h>
uint8_t BLEDeviceInit(void);
void BLETick(void);
tBleStatus Add_Services(void);
tBleStatus Temp_Update(int16_t temp, uint32_t timestamp);
tBleStatus Battery_Load_Update(int16_t voltage);
tBleStatus Battery_Idle_Update(int16_t voltage);
void enableConnectionMode(void);
void enablePairingMode(void);
void recorder_enable_recording(void);
void recorder_disable_recording(void);
void recorder_set_recording_interval(uint16_t interval_seconds);
void recorder_ring_clear(void);
void Recorder_notify_time(uint32_t absoluteTime);