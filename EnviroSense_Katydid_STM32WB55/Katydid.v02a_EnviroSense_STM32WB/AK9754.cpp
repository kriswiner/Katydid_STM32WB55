/* 02/16/2020 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch uses SDA/SCL on pins 20/21 (back pads), respectively, and it uses the Grasshopper STM32L082 Development Board.
 The AK9754 is a simple thermopile but  with on-board processing that makes it very versatile as well as low power; 
 power consumption in only ~5 microAmp
 
 Library may be used freely and without limit with attribution.
 
*/

#include "AK9754.h"

  AK9754::AK9754(I2Cdev* i2c_bus){
  _i2c_bus = i2c_bus;
  }


  uint8_t AK9754::getCompanyID(uint8_t AK9754_ADDRESS) {
  uint8_t temp = _i2c_bus->readByte(AK9754_ADDRESS, AK9754_WIA1);  // issue general call and reload command
  return temp;
  }


  uint8_t AK9754::getDeviceID(uint8_t AK9754_ADDRESS) {
  uint8_t temp = _i2c_bus->readByte(AK9754_ADDRESS, AK9754_WIA2);  // issue general call and reload command
  return temp;
  }


  void AK9754::reset(uint8_t AK9754_ADDRESS)
  {
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL1, 0xFF);
  }


  void AK9754::configure(uint8_t AK9754_ADDRESS, uint8_t mode, uint8_t ODR, uint8_t TMPLPF, uint8_t IRLPF)
  {
  // Low noise mode en/disable (bit6 == 1/0), bit 7 reserve == 1
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL3, 0x80 | mode << 6 | ODR << 4 | TMPLPF << 2 | IRLPF); // enable auto threshold adjustment
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL4, 0xFF); // enable auto threshold adjustment
  }


  void AK9754::setGain(uint8_t AK9754_ADDRESS, uint8_t gain)
  {
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL6,  0xE0 | gain); // default gain is 0x10 == 100%
  }


  void AK9754::setThreshold(uint8_t AK9754_ADDRESS, uint16_t threshold)
  {
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL9,  (threshold & 0x00FF)); // Low byte (default 0x2C)
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL10,  (threshold & 0xFF00) >> 8); // High byte (default 0x01)
  }

  
  void  AK9754::setDetectionTime(uint8_t AK9754_ADDRESS, uint8_t DTC)
  {
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL8,  0x80 | DTC); // default is 0x01, max is 0x7F == 127
  }


  void AK9754::HumanSenseEnable(uint8_t AK9754_ADDRESS)
  {
  // Enable human detection (bit 4 == 1) , interrupt on human detection (bit 1 == 1)
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL11,  0xF0 | 0x02); 
  }


  void AK9754::ContinuousMeasurementMode(uint8_t AK9754_ADDRESS)
  {
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL12,  0xFF); 
  }

  
    void AK9754::StandbyMode(uint8_t AK9754_ADDRESS)
  {
  _i2c_bus->writeByte(AK9754_ADDRESS, AK9754_CNTL12,  0xFE); 
  }


    void AK9754::readData(uint8_t AK9754_ADDRESS, uint8_t * destination)
  {
    _i2c_bus->readBytes(AK9754_ADDRESS, AK9754_ST1,  6, destination); 
  }
