/* readSPIFlash_Katydid.v02a_STM32WB55.ino
2021 Copyright Tlera Corporation 

January 23, 2021

Sketch to read the QSPI Flash on the Katydid.v02a, reconstruct the sensor data, and output CMS-compatible data for plotting.
 */

#include "SFLASH.h"

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
uint16_t max_page_number = 0xFFFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;
uint8_t mid;
uint16_t did;

uint32_t compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t rawVbat;
float VDDA, VBAT;
uint16_t eCO2 = 0, TVOC = 0;
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float aRes = 2.0f/8192.0f;   // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
uint8_t SOC;
/*Choices are:
 IT_40  40 ms, IT_80 80 ms, IT_160  160 ms, IT_320  320 ms, IT_640 640 ms, IT_1280 1280 ms*/
uint8_t IT = 0;  // integration time variable IT_40
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT)); // ambient light sensitivity increases with integration time


void setup(void)
{ 
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("Serial enabled!");

  // Test QSPI flash memory
  Serial.println("QSPI Flash Check");
  SFLASH.begin();
  SFLASH.identify(mid, did);
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
 
  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 10; page_number++)  { // change the page number limit to correspond to number of pages logged

  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
  SFLASH.read(page_number * 256, flashPage, sizeof(flashPage));
      
   for(sector_number = 0; sector_number < 6; sector_number++) {

    accelCount[0] = ((int16_t) flashPage[sector_number*42 + 1] << 8) | flashPage[sector_number*42 + 2];
    accelCount[1] = ((int16_t) flashPage[sector_number*42 + 3] << 8) | flashPage[sector_number*42 + 4];
    accelCount[2] = ((int16_t) flashPage[sector_number*42 + 5] << 8) | flashPage[sector_number*42 + 6];
    
    ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes/4.0f;   
    az = (float)accelCount[2]*aRes/4.0f;  

    int16_t tmp = ((int16_t) (flashPage[sector_number*42 + 0] << 8   | 0x00)) >> 8;  // reconstruct signed 8-bit accel temperature
    temperature = 0.5f * ((float) tmp )  + 23.0f; // Accel chip temperature in degrees Centigrade
    
    compTemp = ((uint32_t) flashPage[sector_number*42 + 11] << 24) |  ((uint32_t)flashPage[sector_number*42 + 12] << 16) |  ((uint32_t)flashPage[sector_number*42 + 13] << 8) | flashPage[sector_number*42 + 14];
    compHumidity = ((uint32_t) flashPage[sector_number*42 + 15] << 24) |  ((uint32_t)flashPage[sector_number*42 + 16] << 16) |  ((uint32_t)flashPage[sector_number*42 + 17] << 8) | flashPage[sector_number*42 + 18];
    compPress = ((uint32_t) flashPage[sector_number*42 + 19] << 24) |  ((uint32_t)flashPage[sector_number*42 + 20] << 16) |  ((uint32_t)flashPage[sector_number*42 + 21] << 8) | flashPage[sector_number*42 + 22];

    eCO2 = (uint16_t) ((uint16_t) flashPage[sector_number*42 + 7] << 8 | flashPage[sector_number*42 + 8]);
    TVOC = (uint16_t) ((uint16_t) flashPage[sector_number*42 + 9] << 8 | flashPage[sector_number*42 + 10]);

    Seconds = flashPage[sector_number*42 + 23];
    Minutes = flashPage[sector_number*42 + 24];
    Hours = flashPage[sector_number*42 + 25];
    Day = flashPage[sector_number*42 + 26];
    Month = flashPage[sector_number*42 + 27];
    Year = flashPage[sector_number*42 + 28];

    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    pressure = (float) compPress/25600.0f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

    RGBWData[0] = ((uint16_t) flashPage[sector_number*42 + 29] << 8) |  flashPage[sector_number*42 + 30];
    RGBWData[1] = ((uint16_t) flashPage[sector_number*42 + 31] << 8) |  flashPage[sector_number*42 + 32];
    RGBWData[2] = ((uint16_t) flashPage[sector_number*42 + 33] << 8) |  flashPage[sector_number*42 + 34];
    RGBWData[3] = ((uint16_t) flashPage[sector_number*42 + 35] << 8) |  flashPage[sector_number*42 + 36];


    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
      float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
      float CCT = 4278.6f*powf(temp, -1.2455f) + 0.5f;

    // Output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(" ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      
    Serial.print(pressure, 2); Serial.print(","); Serial.print(temperature_C, 2); Serial.print(",");Serial.print(temperature_F, 2); Serial.print(",");
    Serial.print(altitude, 2); Serial.print(","); Serial.print(humidity, 1); Serial.print(","); 
    Serial.print(eCO2); Serial.print(","); Serial.print(TVOC); Serial.print(","); 
    Serial.print((float)RGBWData[0]/96.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[1]/74.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[2]/56.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[1]*GSensitivity, 2); Serial.print(",");
    Serial.print(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]); Serial.print(",");
    Serial.print(CCT,2); Serial.print(",");
    Serial.print((int)1000*ax); Serial.print(","); Serial.print((int)1000*ay); Serial.print(","); Serial.print((int)1000*az); Serial.print(","); Serial.println(temperature, 2); 

    }
  
  }


}

void loop(void)
{
}
