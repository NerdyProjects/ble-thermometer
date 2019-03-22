/* battery idle time to recover from load situation in ms */
#define BATTERY_IDLE_GUARD_INTERVAL 30
/* approximate time for a battery measurement */
#define BATTERY_MEASUREMENT_TIME 15
void adc_trigger_read_battery(void);
void adc_init(void);
uint8_t adc_result_ready(void);
int16_t adc_read_battery(void);