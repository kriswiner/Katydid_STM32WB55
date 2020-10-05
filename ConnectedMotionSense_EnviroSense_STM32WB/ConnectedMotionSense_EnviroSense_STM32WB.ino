/* Basic sketch for Connected Motion Sensor.v01
 *  
 *  Demonstrate RTC time keeping and use of RTC alarm for serial output
 *  Demonstrate BMA400 wake-on-motion and sleep-on-no-motion
 
   This example code is in the public domain.
*/
#include "STM32WB.h"
#include "I2Cdev.h"
#include "BMA400.h"
#include "BME280.h"
#include "VEML6040.h"
#include "CCS811.h"
#include "AK9754.h"
#include <RTC.h>
#include "SPIFlash.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

float VBAT, VDDA, STM32Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
bool SerialDebug = true;

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt

const uint8_t myLed1 = LED_BUILTIN; // blue led
const uint8_t myLed2 = 7; // orange led

//BMA400 definitions
const uint8_t BMA400_intPin1 = 3;    // interrupt1 pin definitions, wake-up from STANDBY pin
const uint8_t BMA400_intPin2 = 10;   // interrupt2 pin definitions, data ready or sleep interrupt

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
float   BMA400temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
volatile bool BMA400_wake_flag = false;
volatile bool BMA400_sleep_flag = false;
volatile bool InMotion = false;

BMA400 BMA400(&i2c_0); // instantiate BMA400 class


// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced,, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = BME280Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

float Temperature, Pressure, Humidity;              // stores BME280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp, rawHumidity, compTemp;   // pressure and temperature raw count output for BME280
uint32_t compHumidity, compPress;                   // variables to hold raw BME280 humidity value

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280(&i2c_0); // instantiate BME280 class


// CCS811 definitions
#define CCS811_intPin   6
#define CCS811_wakePin A4

/* Specify CCS811 sensor parameters
 *  Choices are   dt_idle , dt_1sec, dt_10sec, dt_60sec
 */
uint8_t AQRate = dt_10sec;  // set the sample rate
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;

volatile bool newCCS811Data  = false; // boolean flag for interrupt

CCS811 CCS811(&i2c_0); // instantiate CCS811 class


// Specify VEML6040 Integration time
/*Choices are:
 IT_40  40 ms, IT_80  80 ms, IT_160  160 ms, IT_320  320 ms, IT_640  640 ms, IT_1280  1280 ms*/
uint8_t IT = IT_40;  // integration time variable
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT) ); // ambient light sensitivity increases with integration time
float ambientLight;

VEML6040 VEML6040(&i2c_0);


// AK9754 Configuration
const uint8_t AK9754_intPin = 2;

uint8_t AK9754_data[6];
volatile boolean AK9754_int_flag = false;

/* 0x10 <= gain <= 0x0F
* applied gain of 0x10 = 50%, 0x1A = 100%, 0x1F = 125%, 0x00 = 130%, 0x05 = 150%, 0x0E = 200%, two's complement
*/
uint8_t gain = 0x1A;
/* 
 *  human presence detection threshold, two bytes (default 300 == 0x012C)
*/
//uint16_t threshold = 0x012C; // 300
//uint16_t threshold = 0x0190; // 400
uint16_t threshold = 0x01F4; // 500  // best so far
//uint16_t threshold = 0x0258; // 600
//uint16_t threshold = 0x0320; // 800
//uint16_t threshold = 0x03E8; // 1000
/* 
 *  DTC is detection time from 1 (0x01) to 127 (0x7F), number of consecuting samples detection threshold
    is surpassed before human detection (default 0x01 == one time)
*/
uint8_t DTC = 0x04;
/* 
 * mode = LN_enable or LN_disable (enable/disable low noise mode)
 * ODR = ODR_1Hz, ODR_2Hz, ODR_10Hz (default), ODR_50Hz
 * LPF = LPF_0.0Hz, LPF_0_9Hz, LPF_0_445Hz (default), LPF_None
 */
uint8_t mode = LN_disable, ODR = ODR_50Hz, TMPLPF = LPF_0_445Hz, IRLPF = LPF_0_9Hz;
int16_t IRdata;
volatile bool AK9754_int = false; 

AK9754 AK9754(&i2c_0);


// SPI Flash: 128 MBit (16 MByte) SPI Flash 65,536, 256-byte pages
#define csPin A2 // SPI Flash chip select pin

uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlash SPIFlash(csPin); // instantiate SPI flash class


void setup() 
{  
  if(SerialDebug) Serial.begin(38400);
  delay(2000);
  if(SerialDebug)   Serial.println("Serial enabled!");

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, LOW);  // start with led1 off, since active HIGH
  pinMode(myLed2, OUTPUT);
  digitalWrite(myLed2, LOW);  // start with led2 off, since active HIGH

  pinMode(CCS811_intPin, INPUT);  // active LOW
  pinMode(AK9754_intPin,  INPUT); // active LOW

  pinMode(csPin, OUTPUT);    // set SPI chip select as L082 output
  digitalWrite(csPin, HIGH); // initially de-select
 
  // check SPI Flash ID
  SPIFlash.init();      // start SPI
  SPIFlash.powerUp();   // MX25L12835FZNI defaults to power down state
  SPIFlash.getChipID(); // Verify SPI flash communication
  SPIFlash.powerDown(); // power down SPI flash

  // Set up the I2C sensors
  pinMode(BMA400_intPin1, INPUT_PULLDOWN);  // define BMA400 wake and sleep interrupt pins as ESP32 inputs
  pinMode(BMA400_intPin2, INPUT_PULLDOWN);

  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);

  pinMode(CCS811_wakePin, OUTPUT);
  //Enable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  
  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect BMA400 at 0x14 and BME280 at 0x77 and CCS811 at 0x5A
  delay(1000);

  //Disable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

  // Check device IDs
  uint8_t a = AK9754.getCompanyID(AK9754_ADDRESS);
  uint8_t b = AK9754.getDeviceID(AK9754_ADDRESS);
  Serial.print("AK9754 Company is 0x"); Serial.print(a, HEX); Serial.println(", should be 0x48");
  Serial.print("AK9754  Device is 0x"); Serial.print(b, HEX); Serial.println(", should be 0x15");
  Serial.println(" ");
  
  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(1000);   

  // Read the WHO_AM_I register of the BME280  
  byte d = BME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the WHO_AM_I register of the CCS811  
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  byte e = CCS811.getChipID();
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(a == 0x48 && b == 0x15 && c == 0x90 && d == 0x60 && e == 0x81) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
  Serial.println("AK9754 and BMA400 and BME280 and CCS811 are online..."); Serial.println(" ");

  // Software reset AK9754
  AK9754.reset(AK9754_ADDRESS);
  delay(200);
    
  // Configure AK9754 for human detection
  AK9754.configure(AK9754_ADDRESS, mode, ODR, TMPLPF, IRLPF);  
  AK9754.setGain(AK9754_ADDRESS, gain);
  AK9754.setThreshold(AK9754_ADDRESS, threshold);
  AK9754.setDetectionTime(AK9754_ADDRESS, DTC);
  AK9754.HumanSenseEnable(AK9754_ADDRESS);
  AK9754.ContinuousMeasurementMode(AK9754_ADDRESS);
  
  aRes = BMA400.getAres(Ascale);                                               // get sensor resolutions, only need to do this once
  BMA400.resetBMA400();                                                        // software reset before initialization
  delay(100);      
  BMA400.selfTestBMA400();                                                     // perform sensor self test
  BMA400.resetBMA400();                                                        // software reset before initialization
  delay(1000);                                                                 // give some time to read the screen
  BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
  BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);                  // Initialize sensor in desired mode for application 

  BME280.resetBME280();                                                        // reset BME280 before initilization
  delay(100);
  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);                   // Initialize BME280 altimeter
  BME280.BME280forced();                                                       // get initial data sample, then go back to sleep

  // initialize CCS811 and check version and status
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  CCS811.CCS811init(AQRate);
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

  }
  else 
  {
  if (a != 0x48 || b != 0x15) Serial.println(" AK9754 not functioning!");
  if(c != 0x90) Serial.println(" BMA400 not functioning!");     
  if(d != 0x60) Serial.println(" BME280 not functioning!");     
  if(e != 0x81) Serial.println(" CCS811 not functioning!");     
  }

  VEML6040.enableVEML6040(IT); // initalize VEML6040 sensor
  delay(150); 
  
  // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // attach wake-up    interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion  interrupt for INT2 pin output of BMA400 
  attachInterrupt(CCS811_intPin,  myinthandler3, FALLING); // enable CCS811 interrupt
  attachInterrupt(AK9754_intPin,  myinthandler4, FALLING); // attach data ready interrupt for INT pin output of AK9754

  AK9754.readData(AK9754_ADDRESS, AK9754_data); // read data to clear interrupt
  
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  CCS811.readCCS811Data(rawData);    // read CCS811 data to clear interrupt
  digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 
  
  BMA400.getStatus(); // read status of interrupts to clear
  } // end of setup


void loop() 
{
  /* AK9754 Human detection */
  if (AK9754_int_flag)
  {
    AK9754.readData(AK9754_ADDRESS, AK9754_data); // clear interrupt 0
    AK9754_int_flag = false;  

    IRdata =  (int16_t) ( ((int16_t) AK9754_data[2] << 8) | AK9754_data[1]);
    Serial.print("IRdata = "); Serial.println(IRdata);
    Serial.println(" ");
    if(AK9754_data[5] & 0x10) Serial.println("Human detected on AK9754!");
    Serial.println(" ");
  }

  
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   Serial.println("** BMA400 is awake! **");
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 

   digitalWrite(myLed2, HIGH); // turn of orange led when motion detected
   }

  if(BMA400_sleep_flag)
  {
   Serial.println("** BMA400 is asleep! **");
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   detachInterrupt(BMA400_intPin2);      // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 

   digitalWrite(myLed2, LOW); // turn off orange led when no motion
   }/* end of sleep/wake detect */


    /* CCS811 data */
    // If intPin goes LOW, all data registers have new data
    if(newCCS811Data == true) {  // On interrupt, read data
    newCCS811Data = false;  // reset newData flag
     
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.readCCS811Data(rawData);
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f); 

    // CCS811 data Print here since fastest data rate is 1 Hz and we typically run at either 10 s or 60 s period
    Serial.println("CCS811:");
    Serial.print("Eq CO2 in ppm = "); Serial.println(eCO2);
    Serial.print("TVOC in ppb = "); Serial.println(TVOC);
    Serial.print("Sensor current (uA) = "); Serial.println(Current);
    Serial.print("Sensor voltage (V) = "); Serial.println(Voltage, 2);  
    Serial.println(" ");
    }          

  
  /*RTC Timer*/
  if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;

  if(InMotion) {
      
   BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

  // Now we'll calculate the accleration value into actual g's
   ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
   ay = (float)accelCount[1]*aRes - offset[1];   
   az = (float)accelCount[2]*aRes - offset[2]; 
     
  Serial.println(" ");
  Serial.print("ax = ");  Serial.print((int)1000*ax);  
  Serial.print(" ay = "); Serial.print((int)1000*ay); 
  Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
  Serial.println(" ");
  }

    /* BME280 sensor data */
    BME280.BME280forced();  // get one data sample, then go back to sleep

    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    pressure = (float) BME280.BME280_compensate_P(rawPress)/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readBME280Humidity();
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH
 
    Serial.println("BME280:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature_C, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(temperature_F, 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.print("Altimeter humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %RH");// pressure in millibar
    Serial.println(" ");


  /* VEML6040 Data */
  VEML6040.enableVEML6040(IT); // enable VEML6040 sensor
  delay(ITime + 5);  // wait for integration of light sensor data
  VEML6040.getRGBWdata(RGBWData); // read light sensor data
  VEML6040.disableVEML6040(IT); // disable VEML6040 sensor
 
  Serial.print("Red raw counts = ");   Serial.println(RGBWData[0]);
  Serial.print("Green raw counts = "); Serial.println(RGBWData[1]);
  Serial.print("Blue raw counts = ");  Serial.println(RGBWData[2]);
  Serial.print("White raw counts = "); Serial.println(RGBWData[3]);
  Serial.print("Inferred IR raw counts = "); Serial.println(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]);
  Serial.println("  ");
 
  Serial.print("Red   light power density = "); Serial.print((float)RGBWData[0]/96.0f, 2); Serial.println(" microWatt/cm^2");
  Serial.print("Green light power density = "); Serial.print((float)RGBWData[1]/74.0f, 2); Serial.println(" microWatt/cm^2");
  Serial.print("Blue  light power density = "); Serial.print((float)RGBWData[2]/56.0f, 2); Serial.println(" microWatt/cm^2");
  Serial.println("  ");

  ambientLight = (float)RGBWData[1]*GSensitivity;
  Serial.print("Ambient light intensity = "); Serial.print(ambientLight, 2); Serial.println(" lux");
  Serial.println("  ");

  // Empirical estimation of the correlated color temperature CCT:
  // see https://www.vishay.com/docs/84331/designingveml6040.pdf
  float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
  float CCT = 4278.6f*powf(temp, -1.2455f) + 0.5f;

  Serial.print("Correlated Color Temperature = "); Serial.print(CCT); Serial.println(" Kelvin");
  Serial.println("  ");

  /* RTC */
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
  
  /* Internal sensor data */
  VDDA = STM32WB.readVDDA();
  STM32Temperature = STM32WB.readTemperature();
  USBConnected = USBDevice.attached();
  VBAT = STM32WB.readBattery();

  if(SerialDebug)   Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  if(SerialDebug)   Serial.print("STM32L4 MCU Temperature = "); Serial.println(STM32Temperature, 2);
  if(USBConnected && SerialDebug) Serial.println("USB connected!");
  if(SerialDebug)   Serial.print("VBAT = "); Serial.println(VBAT, 2); 

  digitalWrite(myLed1, HIGH); delay(1);  digitalWrite(myLed1, LOW); // toggle blue led on
  
  } /* End of RTC Timer Handling */
  
// STM32WB.stop(5000);
} /* End of Main Loop */


// Useful functions
void myinthandler1()
{
  BMA400_wake_flag = true; 
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
}


void myinthandler3()
{
  newCCS811Data = true;
}


void myinthandler4()
{
  AK9754_int_flag = true; 
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
