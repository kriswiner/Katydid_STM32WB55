/* Basic sketch for Katydid.v02a (aka Connected Motion Sensor.v02a)
 *  
 *  Demonstrate RTC time keeping and use of RTC alarm for serial output
 *  Demonstrate BMA400 wake-on-motion and sleep-on-no-motion
 *  
 *  Intended to run without any module installed.
 *  
 
   This example code is in the public domain.
*/
#include "Arduino.h"
#include "STM32WB.h"
#include "BLE.h"
#include "I2Cdev.h"
#include "BMA400.h"
#include "RTC.h"
#include "SFLASH.h"
#include "TimerMillis.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus//

TimerMillis BLETimer;
volatile bool BLETx_flag = false;

float VBAT, VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
volatile bool SerialDebug = true;

uint8_t mid;
uint16_t did;

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt

const uint8_t myLed = LED_BUILTIN; // blue led

//BMA400 definitions
const uint8_t BMA400_intPin1 = 3;    // interrupt1 pin definitions, wake-up from STANDBY pin
const uint8_t BMA400_intPin2 = 2;    // interrupt2 pin definitions, data ready or sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      SR_15_5Hz, SRW_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode, sleep_Mode
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t Ascale = AFS_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = false;
bool BMA400_sleep_flag = false;
bool InMotion = false;

BMA400 BMA400(&i2c_0); // instantiate BMA400 class

BLEUart SerialBLE(BLE_UART_PROTOCOL_NORDIC);


void setup() 
{
  if(SerialDebug) Serial.begin(38400);
  while (!Serial) { }
  if(SerialDebug) Serial.println("Serial enabled!");

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with led on, since active HIGH

  // Set up the I2C sensors
  pinMode(BMA400_intPin1, INPUT);  // define BMA400 wake and sleep interrupt pins as STM32WB inputs
  pinMode(BMA400_intPin2, INPUT);

  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);
  
  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect BMA400 at 0x14  
  delay(1000);

  // Check device IDs
  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(1000);   
  
  if(c == 0x90) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
  Serial.println("BMA400 is online..."); Serial.println(" ");

  aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
  BMA400.resetBMA400();                                                // software reset before initialization
  delay(100);      
  BMA400.selfTestBMA400();                                             // perform sensor self test
  BMA400.resetBMA400();                                                // software reset before initialization
  delay(1000);                                                         // give some time to read the screen
  BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
  BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application                     
  }
  else 
  {
  if(c != 0x90) Serial.println(" BMA400 not functioning!");     
  }

  digitalWrite(myLed, LOW); // turn off led after sensors are configured

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

  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC_MATCH_ANY); // alarm once a second

  BLE.begin();
  BLE.setLocalName("Katydid");
  BLE.setAdvertisedServiceUuid(SerialBLE.uuid());
  BLE.addService(SerialBLE);   
  BLE.advertise();
  delay(100);

  BLETimer.start(callbackBLETx, 5000, 10000);           //  10 second period, delayed  5 seconds

  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // attach in-motion  interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion  interrupt for INT2 pin output of BMA400 

  BMA400.getStatus(); // read status of interrupts to clear
  } // end of setup

void loop() 
{
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   Serial.println("** BMA400 is awake! **");
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 

   digitalWrite(myLed, HIGH); // turn on led when motion detected
   }

  if(BMA400_sleep_flag)
  {
   Serial.println("** BMA400 is asleep! **");
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   detachInterrupt(BMA400_intPin2);      // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 

   digitalWrite(myLed, LOW); // turn off blue led when no motion, i.e, going to sleep
   }/* end of sleep/wake detect */

  
  /*RTC Timer*/
  if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;

  if(InMotion) { // output accel data when BMA400 is active
      
   BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

  // Now we'll calculate the acceleration value into actual g's
   ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
   ay = (float)accelCount[1]*aRes - offset[1];   
   az = (float)accelCount[2]*aRes - offset[2]; 
     
  Serial.println(" ");
  Serial.print("ax = ");  Serial.print((int)1000*ax);  
  Serial.print(" ay = "); Serial.print((int)1000*ay); 
  Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
  Serial.println(" ");
  }

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

  digitalWrite(myLed, !digitalRead(myLed)); delay(1);  digitalWrite(myLed, !digitalRead(myLed)); // toggle blue led on
   
  } /* End of RTC Timer Handling */
  

    if(BLETx_flag == true) {  // On interrupt, send data via BLE
       BLETx_flag = false;    // reset BLETx_flag

    if (!BLE.advertising() && !BLE.connected()) 
    {
         BLE.advertise();
    }

  // Send some data to the BLE Serial Console
  SerialBLE.print("T = "); 
  SerialBLE.print( Temperature, 2); 
  SerialBLE.println(" C"); // temperature in degrees Celsius

  SerialBLE.print("VBAT = "); 
  SerialBLE.print(VBAT, 2); 
  SerialBLE.println("  V");
  SerialBLE.println(" ");

  digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW); // toggle blue led on

 } /* End of BLE serial section section */
  
  STM32WB.stop();
} /* End of Main Loop */


// Useful functions

void callbackBLETx()
{
  BLETx_flag = true; 
  STM32WB.wakeup();
}


void myinthandler1()
{
  BMA400_wake_flag = true; 
  STM32WB.wakeup();
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
  STM32WB.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32WB.wakeup();
}
