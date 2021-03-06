# Prototype board v1

## Errata
### HW
  * BMP280 footprint too big (BME280)
  * BMP280 VccIO not connected (seems to be no big deal, CSP is connected to VCC, ESD diodes work well, IO does only need to pulldown, slow I2C possible
  * BR1632 does not like the discharge current so much. Install min. 100µF in parallel to the battery to lower the pulse stress a lot (0.1V for 1 ms @ 10 mA)
  * DIO12 should be pulled down for less current consumption
  * DIO7/boot should not be permanently connected to GND but via pulldown (to be able to change BOOT mode by pulling it up)
  * LED towards GND on DIO0
  * BMP should be further at the side of the pcb: complete design would allow bmp to stick out into the handle
  * Reset time constant must be increased a lot, e.g. 10µF or 470k
### SW
  * edaa4536 most recent measurement time is way too big sometimes

## Operating characteristics
  * Vcc >= 1.9 V stable, at ~1.75V it turned off. BMP seemed stable to the end.
  * Connection interval leads to ~1ms of oscillator startup at ~1.4 mA, then ~1ms of operation at 7 mA peak, ~5.8 mA average
  * BMP measurement wakeup takes ~3.5 ms at ~3 mA
  * BMP measurement seems to take ~17ms at ~450µA
  * Sending wakeup peaks at ~10mA, otherwise similar to connection interval
  * TI White Paper SWRA349 (http://www.ti.com/lit/wp/swra349/swra349.pdf) suggests a ~25µF capacitor to handle these peak loads better. This would require a 50ms connection interval to allow the battery to reload the capacitor when there is only idle current usage.
  * An overnight-result counting the time driven by VTimer had an error of ~120 seconds in ~10 hours. Acceptable, but quite big...
    * This might still be another problem as it was not reproducible looking at another hour of time in the morning (deviation of 0-2 seconds / hour which is significantly less)
  * Time constant for temperature change: device was put in cold water from stable 42 degrees. After 105 seconds, temperature was at 23.9 degrees (~1xtau), after 550 seconds at 13.05 degrees (which looks stabelized / water was heating up slowly afterwards)
