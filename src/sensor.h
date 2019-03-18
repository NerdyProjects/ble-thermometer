#include <stdint.h>

typedef enum {
  SENSOR_NOT_INITIALIZED = -1,
  SENSOR_SUCCESS = 0,
  SENSOR_ERROR_TEMPORARY = 1,
  SENSOR_ERROR_PERSISTENT = 2,
  SENSOR_ERROR_NOT_FOUND = 3
} SensorError;

SensorError sensor_measure(void);
int16_t sensor_read_temperature(void);
SensorError sensor_init(void);
SensorError sensor_get_error(uint8_t clear);
void sensor_deinit(void);