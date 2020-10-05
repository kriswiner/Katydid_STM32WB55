/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 VEML6040 color sensor senses red, green, blue, and white light and incorporates photodiodes, amplifiers, 
 and analog / digital circuits into a single chip using CMOS process. With the   color   sensor   applied,   
 the   brightness,   and   color temperature of backlight can be adjusted base on ambient light  source  
 that  makes  panel  looks  more  comfortable  for  end   user’s   eyes.   VEML6040’s   adoption   of   FiltronTM
 technology  achieves  the  closest  ambient  light  spectral  sensitivity to real human eye responses.

 VEML6040  provides  excellent  temperature  compensation  capability  for  keeping  the  output  stable  
 under  changing  temperature.   VEML6040’s   function   are   easily   operated   via the simple command format 
 of I2C (SMBus compatible) interface  protocol.  VEML6040’s  operating  voltage  ranges  from   2.5   V   to   
 3.6   V.   VEML6040   is   packaged   in   a   lead  (Pb)-free  4  pin  OPLGA  package  which  offers  the  best market-proven reliability.
 
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "VEML6040.h"
#include "I2Cdev.h"

VEML6040::VEML6040(I2Cdev* i2c_bus)
{
 _i2c_bus = i2c_bus;
}


uint16_t VEML6040::getRGBWdata(uint16_t * destination)
{
    for (int j = 0; j < 4; j++)
    {
    uint8_t rawData[2] = {0, 0};
    _i2c_bus->readBytes(VEML6040_ADDRESS, VEML6040_R_DATA + j, 2, &rawData[0]);
    destination[j] = ((uint16_t) rawData[1] << 8) | rawData[0];
    }
}

void VEML6040::enableVEML6040(uint8_t IT)
{
  uint8_t data[2] = {(IT << 4), 0x00};
  _i2c_bus->writeBytes(VEML6040_ADDRESS, VEML6040_CONF, 2, &data[0]);
}

void VEML6040::disableVEML6040(uint8_t IT)
{
  uint8_t data[2] = {((IT << 4) | 0x01), 0x00};
  _i2c_bus->writeBytes(VEML6040_ADDRESS, VEML6040_CONF, 2, &data[0]);
}
