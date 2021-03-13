# Katydid Wearable IoT Platform
Arduino sketches for the STM32WB55-based Katydid (aka Connected Motion Sense) wearable IoT platform

BLE-enabled wearable device for ergonometrics, gait/head/limb/hand tracking, robotics, etc. Intended as an easy-to-use, ultra-low-power IoT platform. Host is the [STM32WB55CG](https://www.st.com/en/microcontrollers-microprocessors/stm32wb55cg.html) MCU in QFN48 package with 1 MByte flash and 256 kB of SRAM with clock speed up to 64 MHz. The platform includes a power on mechanical switch, 1S LiPo battery charger, 16 MByte QSPI flash, BMA400 accelerometer for wake-on-motion, and a secure element for network key storage and authentication. 

The 34 mm x 34 mm device is intended to be programmed via USB using the STM32WB55 Arduino core created by Thomas Roell but includes a tag connect for non-USB programming and debugging. 

UART, I2C, and SPI ports are exposed at the board edge for expansion.

LIR2450 3.6 V 120  mAH rechargeable coin cell or standard LiPo battery via JST connector.

The platform is an ultra-low-power device with 6 uA sleep current; sleep current due mainly to ~2 uA STBC08 battery charger, ~2 uA QSPI flash, and ~2 uA STM32WB55 MCU.

The basic platform accepts small (12.8 mm x 12.9 mm) [modules](https://oshpark.com/shared_projects/DgFZd3nx) that can be customized for particular applications. The sketches in this repository demonstrate basic function of the platform as well as several plug-in devices already designed including:

EnviroSense module measuring pressure/temperature/humidity (BME280), eqCO2 and VOC (CCS811), RGBW ambient light intensity (VEML6040) and Human or animal presence (AK9754); 

Absolute orientation estimation module ([USFSMAX](https://hackaday.io/project/160283-max32660-motion-co-processor/log/182097-max32660-motion-coprocessor-mmc5983ma-low-noise-magnetometer-results)) with heading accuracy of < 0.5 degree rms;

NeuroSense module with 576 [Neurons](http://www.theneuromorphic.com/nm500/) for hardware machine learning.

The 34 mm x 34 mm pcb is designed to mount into a small, inexpensive [Hammond 1551P](https://www.hammfg.com/part/1551PTBU?referer=742) ABS container but the mounting holes are suitable for custom (i.e., 3D-printed) enclosures.

The hardware design is open source and is [available](https://oshpark.com/shared_projects/0hElSrib) in the OSH Park shared space.

Katydid is available on [Tindie](https://www.tindie.com/products/tleracorp/katydid-wearable-ble-sensor-board/).

![](https://user-images.githubusercontent.com/6698410/105618550-6cb54600-5d9d-11eb-872f-713a82b0caf6.jpg)
![](https://user-images.githubusercontent.com/6698410/105618544-4f807780-5d9d-11eb-927c-1f55176702c0.jpg)
![](https://user-images.githubusercontent.com/6698410/111039260-efdc3b00-83e1-11eb-9b4c-4da35ecab04d.jpg)

 For questions about this project please contact Kris Winer at tleracorp@gmail.com.
