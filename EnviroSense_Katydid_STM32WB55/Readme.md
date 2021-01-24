Sketch demonstrating use of the EnviroSense Module with CCS811 air quality (CO2 and VOC) sensor, BME280 temperature, humidity, nd pressure sensor, VEML RGBW ambient light sensor, and AK9754 human presence sensor.

Katydid sleep current with the EnviroSense module is ~30 uA due to the 19 uA sleep current of the CCS811.

Main sketch logs data in the form of 256-byte pages to the QSPI flash at a rate determined by the user (here every 10 seconds). The QSPI flash can be read after the Katydid has completed the sensing task and the data cast in comma-delimited CMS format for esay importation into Excel and other spreadsheets for analysis. Below is an example of the readSPIFlash sketch output showing date, time, pressure, temperatureC, temperatureF, altitude, humidity, CO2, VOC, R, G, B, etc.:

![](https://user-images.githubusercontent.com/6698410/105618859-e438a480-5da0-11eb-8b03-ec7c724489ba.jpg)
