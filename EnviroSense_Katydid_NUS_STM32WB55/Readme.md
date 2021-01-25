Full-monte sketch for data logging on QSPI flash and periodic BLE Tx using the Nordic UART Service (Serial BLE) of select data at rates determined by the user. 

Use Adafruit's BlueFruit smartphone app to see the data on the UART monitor.

Logged data can be read back using the readSPIFlash utility sketch and cast in a convenient comma-delimited format for easy import into Excel and other spreadheets.

At the 10 second BLE and QSPI logging interval, and the 60 second (max) CCS811 data update rate the Katydid.v02a is using on average ~850 uA which means the 120 mAH LIR2450 coin cell battery should last about ~150 hours or about six days of continuous operation; nearly a full week between chargings.
