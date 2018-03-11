# Description

This project describes a thermometer, that should be able to measure temperatures in intervals over a certain time span.
It should be small, self-powered, reusable and easy to be used.
Its designed purpose is especially for fertility awareness methods but also for general sports and medical purposes.
Therefore, it should fulfil the following criteria:

* Small in size to be inserted into the vagina for some period of time
  * Ideally, also swalloable if possible in size
* Safe to not impose any allergic potential
* rechargable battery life time of at least 24 hours
* Wirelessly rechargable
* Wireless data transfer
* storage capacity for a measurement every 5 minutes -> 720 temperature measurements from 25 .. 45 degrees with a resolution of 0.01 degree -> min. 12 kBit of storage space

# Design proposal

## NFC and energy harvesting

A simple design could use NFC and energy harvesting.
Available hardware components could be:

* M24LR16E 16 kBit NFC I2C EEPROM with energy harvesting (avalable in MSOP package); 30µA standby (or turn off); 5ms byte write @220µA; 2.0x3.0mm
* a 15mmx15mm PCB antenna 
* a BMP280 thermometer as small, energy efficient (~2µA average, down to 1.8V); LGA 2.0x2.5mm
* an AT Tiny 13A microcontroller (~4µA PowerDown/Watchdog; 40µA active @128 kHz); 3.0x3.0mm


### Power calculation
* EEPROM write: 0.0016 As for 720 writes
* EEPROM standby: 2.6 As for 24h standby (turn off!)
* Controller: 0.432 As for 24h power down, 0.015 As for 720 measurements
* BMP280: 0.2 As for 24h power down and measuring (or turn off as well for less)

Voltage min. 1.8V, charge 3.6V -> 1.8V difference; Q/Udiff=C; 0.5 As / 1.8V ~ 0.27As/V -> 0.33F cap would fulfill the criteria

Charge time: ~2-3 minutes (small cap), ~5-8 minutes (big cap)

### Mechanical Dimensions
PCB Antenna and Chips fit on a surface of ~15mmx20mm. The board would have a thickness of 1.6mm + components (~2.5mm total)
A 0.33F Goldcap would come as 11mm diameter, 5mm thickness
A 1F Goldcap would come as 20mm diameter, 6.5mm thickness

Antenna should be kept free from both sides, so cap needs to be on the side -> 15mm + (11/20mm) total length, 15/20mm total width, 7.5/9mm thickness

Also, it needs a silicon cover, maybe 2mm thickness all around?
* Small cap: 30mm length, 20mm width, 11.5mm thick
* Big cap: 40mm length, 24mm width, 13mm thick


## Bluetooth low energy and coin battery

Another solution could be using a coin button cell and an integrated bluetooth low energy module.
Available hardware components:

* SPBTLE-1S module (~10 years standby from a 100mAh coin battery (0.9µA); ~1.9mA active; includes 24kByte of retention sram; includes all of processor/antenna/etc.) 13.5x11.5mm
* BMP280 (sleep 0.3µA; active 325µA for ~2ms/measurement)
* small circuit board to connect battery/BMP280/SPBTLE-1S
* Coin battery CR1632
* Small button to activate BTLE by pressing the whole module together? (Not for current consumption reasons but maybe more "radiation" concerns of users)

### Power consideraton

* BTLE standby/advertising rarely: ~1 µA
* BMP280 sleep: ~0.3 µA
* BMP280 measuring every minute with 16x oversampling: ~0.2 µA
* BTLE data transfer every day of 12 kBit (round 1 s of active time per day at ~10mA): ~0.1 µA

Sum + rounding: ~2 µA
Battery lifetime: ~5 years with 100mAh battery

### Mechanical dimensions
* Module with antenna: 13.5x11.5mmx2.0mm
* board underneath 1.0mm
* board overstand/total size: 15mmx16mmx3mm
* battery CR1632 aside: 16mm diameter, 3.2mm height

* Battery aside including silicon: 35x20x7.2mm
* Battery below: 20x20x10.2mm
