#include <Wire.h>
#include <SoftwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Adafruit_GPS.h>
#include <Adafruit_INA219.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include "RTClib.h"
#include <IridiumSBD.h>

// Flag to enable or disable debug prints
#define DEBUG

// what's the name of the hardware serial port?
#define GPSSerial Serial1 

// Declare sensor objects
RTC_PCF8523 rtc; // External RTC
Adafruit_INA219 ina219; // Current and power sensor
Adafruit_LIS3MDL lis3mdl; // Magnetometer (compass) sensor
Adafruit_ISM330DHCX ism330dhcx; // IMU sensor (accelerometer and gyroscope)
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware serial port

//Define pins for green and red leds
int GLED = 15;
int RLED = 4;

int signalQuality = -1;
int err;
const byte SAT_SLEEP = 13;
const byte SAT_TX = 12;
const byte SAT_RX = 27;
SoftwareSerial IridiumSerial(SAT_RX, SAT_TX);
IridiumSBD modem(IridiumSerial, SAT_SLEEP);
enum SBDState
{
  SBD_IDLE,
  SBD_WAIT_FOR_RESPONSE,
  SBD_TRANSMIT,
  SBD_RECEIVE,
  SBD_PASS
} stateSBD;
unsigned long lastActionTime = 0;
// const unsigned long timeoutInterval = 5000;  // 5 seconds timeout for each step
char sbdBuffer[100]; // Buffer to store SBD messages
char logBuffer[64]; // Buffer for logging messages
bool transmissionDone = false;
int IridiumStart = 0;
bool successfulSatTransmission = false;
// int IridiumTimeout = 5000;  //ms for irridium send (Reduced to 5 seconds for indoor testing, revert to 180 sec)

bool gpsComplete;
bool imuComplete;
bool pwrComplete;
bool satComplete;
bool tasksComplete;
bool fault = false;
bool setClock;

SemaphoreHandle_t i2cSemaphore = NULL;
SemaphoreHandle_t serialSemaphore = NULL;
SemaphoreHandle_t uartSemaphore = NULL;
SemaphoreHandle_t sdSemaphore = NULL;

DateTime now;
volatile bool EXT_RTC_INT = false;
volatile bool EXT_IMU_INT = false;
int shortSleep, wakeup;

float busvoltage;
float current_mA;
float power_mW;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

int imuWakeupCount = 0;

float temp;

QueueHandle_t IMUQueue;
char imuBuffer[128];

QueueHandle_t PWRQueue;
char pwrBuffer[128];

char gpsBuffer[128];
char fileBuffer[256];
char satBuffer[128];

char pwrFile[64];
char oldpwrFile[64];

char imuFile[64];
char oldimuFile[64];

char gpsFile[64];
char oldgpsFile[64];

char satFile[64];
char oldSatFile[64];

char logFile[1024];

char satData[1024];

const char *pwrDir = "/pwr_data";
const char *imuDir = "/imu_data";
const char *gpsDir = "/gps_data";
const char *satDir = "/sat_data";
const char *logDir = "/log";

const uint32_t DATA_INTERVAL = 2*60*1000; // Time to sleep between data collection periods
const uint32_t SAT_INTERVAL = 2*60*1000; // Time to wait between satelite transmissions
uint32_t longSleepRemaining = SAT_INTERVAL; // Initialize long sleep remaining to the satellite transmission interval so that the first transmission occurs after the full interval has passed


const uint16_t NUM_PWR_SAMPLES = 5; // Number of power data samples to collect
const uint32_t PWR_RATE = 500; // Time between PWR data readings in ms
const uint16_t NUM_IMU_SAMPLES = 50; // Number of IMU data samples to collect
const uint32_t IMU_RATE = 100; // Time between IMU data readings in ms

const uint32_t GPS_FIX_TIMEOUT = 2*60*1000; // Time to wait for a GPS fix before falling back to best available data

uint32_t bedtime = 0; // Store the time when the device went to sleep
uint32_t durationToSleep = 0; // Store the intended duration of sleep in seconds

bool sendSat = true;

uint32_t timer;

#include "ESP32_runtime.h"
#include "PCF8523_runtime.h"
#include "LIS3MDL_runtime.h"
#include "ISM330DHCX_runtime.h"
#include "INA219_runtime.h"
#include "Feather_GPS.h"
#include "IRIDIUM_runtime.h"



const uint64_t uS_TO_S_FACTOR = 1000000ULL;                     /* Conversion factor for micro seconds to seconds */
const int TIME_TO_SLEEP = 600;                                  /* Time ESP32 will go to sleep for Data Colection (in seconds) */
const uint8_t EXT_RTC_COUNTDOWN_TIMER = 1;                     // external RTC sleep timer(Must be < 255)
PCF8523TimerClockFreq countdown_unit = PCF8523_FrequencyHour; // Set the countdown timer frequency to 1 hour
const unsigned long SAT_TRANSMISSION_COOLDOWN = 10*60*1000; // Minimum time between satellite transmissions in ms
unsigned long lastSatelliteTransmissionTime = 0;

char datafile[] = "";

const bool GPSECHO = false;


uint32_t bootTime = 0; // Store the boot time in seconds since epoch

uint32_t ulNotificationValue;

const int IMUInterruptPin = 14;

const int RTCInterruptPin = 32;
esp_sleep_ext1_wakeup_mode_t wakeup_mode = ESP_EXT1_WAKEUP_ALL_LOW; // Wake up on any high level on the selected pin(s)
volatile bool countdownInterruptTriggered = false;
volatile int numCountdownInterrupts = 0;


void setup(){
  // Set GPIO pins for LEDs as outputs and turn them on
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  digitalWrite(GLED, HIGH);
  digitalWrite(RLED, HIGH);

  // Set Irridium sleep pin as output and set low to power off the modem
  pinMode(SAT_SLEEP, OUTPUT);
  digitalWrite(SAT_SLEEP, LOW);
  delay(100);

  // If debugging is active connect to serial port 
#ifdef DEBUG
  Serial.begin(115200);
  // will pause Zero, Leonardo, etc until serial console opens
  while (!Serial){
    delay(1);
  }
#endif

// Initialize the rtc 
  if (!rtc.begin()){
#ifdef DEBUG
    Serial.println("Couldn't find RTC");
#endif
    fault = true;
    return;
  }
  else{
#ifdef DEBUG
    Serial.println("RTC Found");
#endif
  }
syncTimeWithRTC(); // Ensure the internal time is synced with the RTC

  // Store the boot time in seconds since epoch
  bootTime = rtc.now().unixtime(); 

// Mount the SD card 
  if (!SD.begin()){
#ifdef DEBUG
    Serial.println("Card Mount Failed");
#endif
    fault = true;
    return;
  }
  else{
#ifdef DEBUG
    Serial.println("Card Mounted");
#endif
  }

  // Create directories and files for data storage 
  // Initialize headers for each data file with column names
  char timeBuffer[32] = "YYYYMMDD-hhmmss";
  rtc.now().toString(timeBuffer);
  Serial.println("timeBuffer initialized with current time");

  sprintf(pwrFile, "%s/pwr-data-%s.csv", pwrDir, timeBuffer);
  createDir(SD, pwrDir);
  writeFile(SD, pwrFile, "Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

  sprintf(imuFile, "%s/imu-data-%s.csv", imuDir, timeBuffer);
  createDir(SD, imuDir);
  writeFile(SD, imuFile, "Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count\n");

  sprintf(gpsFile, "%s/gps-data-%s.csv", gpsDir, timeBuffer);
  createDir(SD, gpsDir);
  writeFile(SD, gpsFile, "Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg)\n");

  sprintf(satFile, "%s/sat-data-%s.csv", satDir, timeBuffer);
  createDir(SD, satDir);
  writeFile(SD, satFile, "SatMsgSuccess?, Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg), Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count, Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

  sprintf(logFile, "%s/log-file-%s.csv", logDir, timeBuffer);
  createDir(SD, logDir);

  toLogFile(SD, logFile, "System Booted");

  // Initialize the GPS module with the appropriate settings and put it to sleep
  GPS.begin(9600);
  GPS.sendCommand("");
  vTaskDelay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  vTaskDelay(100);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  vTaskDelay(100);
  GPS.sendCommand(PGCMD_ANTENNA);
  vTaskDelay(100);
  GPS.sendCommand(PMTK_STANDBY);
  vTaskDelay(1000);
  toLogFile(SD, logFile, "Adafruit feather GPS setup commands sent and put to sleep");

  // Initialize the INA219 power sensor and log if it is not detected
  if (!ina219.begin()){
    toLogFile(SD, logFile, "Failed to find INA219 chip");
    fault = true;
  }
  else{
    toLogFile(SD, logFile, "Adafruit INA219 Test Success");
  }

  // Initialize the LIS3MDL magnetometer sensor and log if it is not detected
  if (!lis3mdl.begin_I2C()){ // hardware I2C mode, can pass in address & alt Wire
    toLogFile(SD, logFile, "Failed to find LIS3MDL chip");
    fault = true;
  }
  else{
    // Log successful initialization and the default settings for the magnetometer
    toLogFile(SD, logFile, "LIS3MDL Test Success");
    lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
    lis3mdl.setOperationMode(LIS3MDL_SINGLEMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_0_625_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  // Initialize the ISM330DHCX IMU sensor and log if it is not detected
  if (!ism330dhcx.begin_I2C()){ // hardware I2C mode, can pass in address & alt Wire
    toLogFile(SD, logFile, "Failed to find ism330dhcx chip");
    fault = true;
  }
  else{
    // Log successful initialization and the default settings for the IMU
    toLogFile(SD, logFile, "ism330dhcx Test Success");
    ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    // Set the imu to trigger interupt
    configureWakeupInterrupt_IMU();

    // Configure the interrupt pin on the Adafruit Feather ESP32 V2
    pinMode(IMUInterruptPin, INPUT_PULLUP); // Use GPIO 14, which corresponds to the pin labeled D14 on the Feather ESP32 V2
    attachInterrupt(digitalPinToInterrupt(IMUInterruptPin), wakeUpDetectedIMU, FALLING); // Trigger on falling edge since the IMU interrupt pin goes low when an interrupt is triggered
  }

  // Begin Sattelite modem initialization sequence by powering on the modem and starting the serial communication
  digitalWrite(SAT_SLEEP, HIGH);
  delay(1000);
  IridiumSerial.begin(19200);

  //Put satellire modem to sleep until it's time to transmit
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
  modem.sleep();

}

void loop(){

  digitalWrite(GLED, HIGH);
  digitalWrite(RLED, HIGH);

  // If the IMU interrupt was triggered, log the event and go back to sleep without recording data 
  if(EXT_IMU_INT){
    // Reset the IMU interrupt flag and increment the wakeup count
    EXT_IMU_INT = false;
    imuWakeupCount++;
    //Set the duration to sleep based on the previous duration to sleep and the time interupted. 
    durationToSleep = durationToSleep - (rtc.now().unixtime() - bedtime);
    if (durationToSleep < 1) { //Dont allow negative sleep times
      durationToSleep = 1;
    }
  }
  // If the IMU interrupt was not triggered then collect data
  else{
    syncTimeWithRTC(); // Ensure the internal time is synced with the RTC
    toLogFile(SD, logFile, "Time synced with RTC");

    collectGPSData(); //Stores GPS data in gpsBuffer global variable. Also updates RTC if valid fix

    collectPWRData(); //Stores power data in pwrBuffer global variable
    toLogFile(SD, logFile, "Power data collected");

    collectIMUData(); //Stores IMU data in imuBuffer global variable
    toLogFile(SD, logFile, "IMU data collected");

    if(sendSat){
    combine_data_buffers(); //Combines data from gpsBuffer, pwrBuffer, and imuBuffer into satBuffer global variable in preparation for satellite transmission
    sendSatMsg(fileBuffer); //Sends the data in satBuffer via the satellite modem and logs the result
    }
    // If the data collection interval is less than the remaining long sleep time, 
    // then set the sleep duration to the data collection interval. 
    // Otherwise, set it to the remaining long sleep time and reset the long sleep remaining time.
    if(DATA_INTERVAL < longSleepRemaining){ 
      longSleepRemaining = longSleepRemaining - DATA_INTERVAL;
      durationToSleep = DATA_INTERVAL;
      sendSat = false;  
    }
    else{
      longSleepRemaining = SAT_INTERVAL;
      durationToSleep = longSleepRemaining;
      sendSat = true; 
    }

  }
  
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
  
  bedtime = rtc.now().unixtime(); // Record the time the device went to sleep in seconds since epoch 
  esp_sleep_enable_timer_wakeup((uint64_t)durationToSleep * uS_TO_S_FACTOR);// Set the time to sleep for
  esp_sleep_enable_ext1_wakeup((1ULL << 14), ESP_EXT1_WAKEUP_ALL_LOW);  // Enable wakeup from imu interupt 
  esp_light_sleep_start();  // Put the device to sleep
  
// ---------- END ---------- //

}

