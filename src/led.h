typedef enum {
  LED_OFF,
  LED_FAST_FLASH,
  LED_FLASH,
  LED_BLINK,
} LedMode;

void led_set(LedMode mode, uint8_t count);