Sketch demonstrating use of the EnviroSense Module with CCS811 air quality (CO2 and VOC) sensor, BME280 temperature, humidity, nd pressure sensor, VEML RGBW ambient light sensor, and AK9754 human presence sensor.

Katydid sleep current with the EnviroSense module is ~30 uA due to the 19 uA sleep current of the CCS811.

Main sketch logs data to the QSPI flash at a rate determinedby the user (here every 10 seconds). The QSPI flash can be read after the Katydid has completed the sensing task and the data cast in comma-delimited CMS format for eay importation into excel and other spreadsheets for analysis. Below an example of the readSPIFlash sketch output:

![](https://user-images.githubusercontent.com/6698410/105618796-3927eb00-5da0-11eb-9664-7a1f9068e396.jpg)
