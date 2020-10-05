# ConnectedMotionSense
Arduino sketches for the STM32WB-based Connected Motion Sense wearable platform

BLE-enabled wearable device for ergonometrics, gait/head/limb/hand tracking, robotics, etc. Intended as an easy-to-use, ultra-low-power IoT platform. Host is the [STM32WB55CG](https://www.st.com/en/microcontrollers-microprocessors/stm32wb55cg.html) MCU in QFN48 package with 1 MByte flash and 256 kB of SRAM with clock speed up to 64 MHz. The platform includes a power on mechanical switch, 1 S LiPo battery charger, 16 MByte QSPI flash, BMA400 accelerometer for wake-on-motion, and a secure element for network key storage and authentication. 

The device is intended to be programmed via USB using the STM32WB55 Arduino core created by Thomas Roell but includes a tag connect for non-USB programming and debugging. 

UART, I2C, and SPI ports are exposed at the board edge for expansion.

The basic platform accepts small modules that can be customized for particular applications. The sketches in this repository demonstrated basic function of the base device as well as the several plug-in devices including:

EnviroSense module measuring pressure/temperature/humidity (BME280), eqCO2 and VOC (CCS811), RGBW ambient light sensor (VEML6040) and Human presence sensor (AK9754); 

Absolute orientation estimation module ([USFSMAX](https://hackaday.io/project/160283-max32660-motion-co-processor/log/182097-max32660-motion-coprocessor-mmc5983ma-low-noise-magnetometer-results)) with heading accuracy of < 0.5 degree rms;

NeuroSense module with 576 [Neurons](http://www.theneuromorphic.com/nm500/) for hardware machine learning.

The pcb is designed to mount into a small, inexpensive Hammond 1551P ABS container but the mounting holes are also useful for custom (i.e., 3D-printed) enclosures.

![]()