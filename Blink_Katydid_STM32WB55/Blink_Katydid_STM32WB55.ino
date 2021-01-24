/*  Basic blink sketch for Katydid v.02a (aka Connected Motion Sensor.v02a)
 *  
 *  2021 Copyright Tlera Corporation
 *  
 *  Demonstrate RTC time keeping and RTC alarm, 
 *  Demonstrate internal functions like measurement of:
 *     USB connect state, MCU temperature, analog reference voltage, and battery voltage  
 *     
 *  Demonstrate use of STOP mode; device uses ~6 uA in STOP mode due to MCU current ~2 uA),
 *  presence of SPI Flash (~2 uA) and STBC08 battery charger (~2 uA).
 
   This example code is in the public domain.

   All uses permitted without warranty with attribution.
*/
#include "STM32WB.h"
#include <RTC.h>

float VBAT, VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
volatile bool SerialDebug = true;

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt

const uint8_t myLed = LED_BUILTIN; // blue led

void setup() 
{
  if(SerialDebug) Serial.begin(38400);
  delay(2000);
  if(SerialDebug)   Serial.println("Serial enabled!");

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with led off, since active HIGH
 
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  } // end of setup

void loop() 
{
  /*RTC Timer*/
  if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;

  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");
  
  VDDA = STM32WB.readVDDA();
  Temperature = STM32WB.readTemperature();
  USBConnected = USBDevice.attached();
  VBAT = STM32WB.readBattery();

  if(SerialDebug)   Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  if(SerialDebug)   Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
  if(USBConnected && SerialDebug) Serial.println("USB connected!");
  if(SerialDebug)   Serial.print("VBAT = "); Serial.println(VBAT, 2); 

  digitalWrite(myLed, HIGH); delay(10);  digitalWrite(myLed, LOW); // toggle blue led on
  
  } /* End of RTC Timer Handling */
  
 STM32WB.stop(); // stay in lowest power mode until next interrupt (RTC Alarm)
} /* End of Main Loop */


// Useful functions

void alarmMatch()
{
  alarmFlag = true;
  STM32WB.wakeup();
}
