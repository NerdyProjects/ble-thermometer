#ifndef _APPFLAGS_H
#define _APPFLAGS_H
extern volatile int app_flags;

/**
 * @name Flags for application
 * @{
 */
#define CONNECTED                0x01
#define SET_DIRECTED_CONNECTABLE 0x02
#define SET_PUBLIC_CONNECTABLE   0x04
#define CONNECTABLE              0x08
#define PAIRABLE                 0x10
#define READ_RECORDER_BUFFER     0x20
#define TX_BUFFER_FULL           0x40
#define CONFIRMATION_RECEIVED    0x80

/* Added flags for handling security steps */
#define START_GAP_SLAVE_SECURITY_REQUEST    0x100
#define HCI_ENCRYPTION_CHANGE_EVENT_FLAG    0x200 
#define ACI_GAP_PAIRING_COMPLETE_EVENT_FLAG 0x800

#define TRIGGER_DATA_TRANSFER               0x400


/**
 * @}
 */

/* Exported macros -----------------------------------------------------------*/
#define APP_FLAG(flag) (app_flags & flag)

#define APP_FLAG_SET(flag) (app_flags |= flag)
#define APP_FLAG_CLEAR(flag) (app_flags &= ~flag)
#endif