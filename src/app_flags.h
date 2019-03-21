#ifndef _APPFLAGS_H
#define _APPFLAGS_H
extern volatile int app_flags;

/**
 * @name Flags for application
 * @{
 */
#define CONNECTED                0x000001
#define SET_DIRECTED_CONNECTABLE 0x000002
#define SET_PUBLIC_CONNECTABLE   0x000004
#define CONNECTABLE              0x000008
#define PAIRABLE                 0x000010
#define READ_RECORDER_BUFFER     0x000020
#define TX_BUFFER_FULL           0x000040
#define CONFIRMATION_RECEIVED    0x000080

#define START_GAP_SLAVE_SECURITY_REQUEST    0x000100
#define HCI_ENCRYPTION_CHANGE_EVENT_FLAG    0x000200 
#define ACI_GAP_PAIRING_COMPLETE_EVENT_FLAG 0x000800
#define TRIGGER_DATA_TRANSFER               0x000400

#define ADC_LOAD_CONVERSION_IN_PROGRESS 0x001000
#define ADC_IDLE_CONVERSION_IN_PROGRESS 0x002000
#define ADC_LOAD_CONVERSION_REQUEST     0x004000
#define ADC_IDLE_CONVERSION_REQUEST     0x008000

#define SENSOR_UNAVAILABLE              0x010000
#define SENSOR_TIMER_ELAPSED            0x020000
#define SENSOR_MEASUREMENT_TRIGGERED    0x040000
#define MEASUREMENT_ENABLED                     0x080000

#define REQUEST_LOW_POWER_CONNECTION_PARAMETERS  0x0100000
#define REQUEST_DISABLE_BLE                              0x0200000
#define REQUEST_DISABLE_MEASUREMENT              0x0400000
#define REQUEST_ENABLE_MEASUREMENT               0x0800000

#define REQUEST_RING_CLEAR                      0x1000000
#define RECORDING_ENABLED                       0x2000000
#define STORE_RECORD_TIME_DIFFERENCE            0x4000000



/**
 * @}
 */

/* Exported macros -----------------------------------------------------------*/
#define APP_FLAG(flag) (app_flags & flag)

#define APP_FLAG_SET(flag) (app_flags |= flag)
#define APP_FLAG_CLEAR(flag) (app_flags &= ~flag)
#endif