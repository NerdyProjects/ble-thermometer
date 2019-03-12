# Prototype board v1

## Errata
  * BMP280 footprint too big (BME280)
  * BMP280 VccIO not connected (seems to be no big deal, CSP is connected to VCC, ESD diodes work well, IO does only need to pulldown, slow I2C possible
  * BR1632 does not like the discharge current so much. Install min. 100µF in parallel to the battery to lower the pulse stress a lot (0.1V for 1 ms @ 10 mA)
  * DIO12 should be pulled down for less current consumption
  * DIO7/boot should not be permanently connected to GND but via pulldown (to be able to change BOOT mode by pulling it up)

## Operating characteristics
  * Vcc >= 1.9 V stable, at ~1.75V it turned off. BMP seemed stable to the end.
  * Connection interval leads to ~1ms of oscillator startup at ~1.4 mA, then ~1ms of operation at 7 mA peak, ~5.8 mA average
  * BMP measurement wakeup takes ~3.5 ms at ~3 mA
  * BMP measurement seems to take ~17ms at ~450µA
  * Sending wakeup peaks at ~10mA, otherwise similar to connection interval