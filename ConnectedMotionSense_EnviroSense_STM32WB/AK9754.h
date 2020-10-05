/* 02/16/2020 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch uses SDA/SCL on pins 20/21 (back pads), respectively, and it uses the Grasshopper STM32L082 Development Board.
 The AK9754 is a simple thermopile but  with on-board processing that makes it very versatile as well as low power; 
 power consumption in only ~5 microAmp
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef AK9754_h
#define AK9754_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

//https://media.digikey.com/pdf/Data%20Sheets/AKM%20Semiconductor%20Inc.%20PDFs/AK9754_6-20-18.pdf// CaliPile Registers
#define AK9754_WIA1            0x00
#define AK9754_WIA2            0x01
#define AK9754_INFO1           0x02
#define AK9754_INFO2           0x03
#define AK9754_ST1             0x04
#define AK9754_IRL             0x05
#define AK9754_IRH             0x06
#define AK9754_TPMPL           0x07
#define AK9754_TPMPL           0x08
#define AK9754_ST2             0x09
#define AK9754_ST3             0x0A

// 10 position data FIFO
#define AK9754_SB0L            0x0B
#define AK9754_SB0H            0x0C
// ....
#define AK9754_SB9L            0x1D
#define AK9754_SB9H            0x1E

#define AK9754_ST4             0x1F
#define AK9754_CNTL1           0x20
#define AK9754_CNTL2           0x21
#define AK9754_CNTL3           0x22
#define AK9754_CNTL4           0x23
#define AK9754_CNTL5           0x24
#define AK9754_CNTL6           0x25
#define AK9754_CNTL7           0x26
#define AK9754_CNTL8           0x27
#define AK9754_CNTL9           0x28
#define AK9754_CNTL10          0x29
#define AK9754_CNTL11          0x2A
#define AK9754_CNTL12          0x2B


const uint8_t AK9754_ADDRESS = 0x60;
#define AK9754_0_ADDRESS 0x60
#define AK9754_1_ADDRESS 0x62

//Low noise mode
#define LN_enable  0x01
#define LN_disable 0x00

// Sample rate (ODR)
#define ODR_1Hz 0x00
#define ODR_2Hz 0x01
#define ODR_10Hz 0x02
#define ODR_50Hz 0x03

//Temperature and IR Low Pass Filter -only used at 10 Hz, otherwise not applied
#define LPF_0_0Hz   0x00  // no filter
#define LPF_0_9Hz   0x01  // 0.9 Hz
#define LPF_0_445Hz 0x02  // 0.445 Hz, default
#define LPF_None    0x03  // Not used


class AK9754
{
  public: 
  AK9754(I2Cdev* i2c_bus);
  uint8_t getCompanyID(uint8_t AK9754_ADDRESS);
  uint8_t getDeviceID(uint8_t AK9754_ADDRESS);
  void    reset(uint8_t AK9754_ADDRESS);
  void    configure(uint8_t AK9754_ADDRESS, uint8_t mode, uint8_t ODR, uint8_t TMPLPF, uint8_t IRLPF);
  void    setGain(uint8_t AK9754_ADDRESS, uint8_t gain);
  void    setThreshold(uint8_t AK9754_ADDRESS, uint16_t threshold);
  void    setDetectionTime(uint8_t AK9754_ADDRESS, uint8_t DTC);
  void    HumanSenseEnable(uint8_t AK9754_ADDRESS);
  void    ContinuousMeasurementMode(uint8_t AK9754_ADDRESS);
  void    StandbyMode(uint8_t AK9754_ADDRESS);
  void    readData(uint8_t AK9754_ADDRESS, uint8_t * dataArray);
  private:
  // Register read variables
  I2Cdev* _i2c_bus;
  };

#endif
