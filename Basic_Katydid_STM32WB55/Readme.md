Sketch builds on the simplest blink sketch to also check function of the I2C BMA400 accel wake-on-motion and sleep-on-no-motion interrupt capability.

Sketch configures the BMA400 for wake-on-motion based on an any-axis accel threshold (here set to 50 mg) and number of consecuting samples above threshold (here set to 4) both user-configurable in the initBMA400 function call of the BMA400.cpp file (lines 73 - 78). 

BMA400 Wake and Sleep interrupts wake the MCU from STOP mode and allow the user to set up different actions for the "in-motion" and "no-motion" conditions depending on the specific applications.

Sketch also tests QSPI flash memory as a prelude to logging dat in the QSPI flash memory in later sketches.

