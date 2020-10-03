#include "Commands.h"
#include "Errors.h"
#include "KeypadVar.h"
#include "LCDHelper.h"

BMEwrapper bme;

#include <SPI.h>

// SD card writing libraries
#include "SdFat.h"
#include "FreeStack.h"
#include "AnalogBinLogger.h"

#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_GPS.h>

#define DEFAULTTXT "98# List Cmds"

#define READBME "1#" //output bme680 file to serial window
#define WRITEBME "2#" //write bme680 to sd card using BMEFILE
#define READGPS "4#" //output gps file to serial window
#define WRITEGPS "5#" //write gps to sd card using GPSFILE

#define RECORD "7#" //record audio
#define COLLECT "8#" //collect data routine

#define PRINTBME "51#" //prints bme680 to serial window
#define PRINTGPS "52#" //prints gps to serial window

#define BOOTSD "97#" //reboot sd card when reinserted
#define LIST "98#" //list all commands on LCD
#define CLEAR "***" //clears LCD screen

#define LOOPTIMES 5 //number of times to loop through data in seconds
#define DATAPERSECOND 1 //number of times to gather data in one second

//names of the data files
#define BMEFILE "bme"
#define GPSFILE "gps"

//variables for sd
const int chipSelect = 53;

//function declarations for sd
bool initSD();
int findNextFile(String filename);
void printSD(String filename);
void writeSD(String filename, String data);

void logData();
void audioSetup();
void audioLoop();
void adcInit(metadata_t* meta);
void adcStart();
void adcStop();
void checkOverrun();
void dumpData();

/////////////////// Arduino ADC parameters ///////////////////

// Analog pin number list for a sample.  Pins may be in any order and pin
// numbers may be repeated. My version: just use A7.
// const uint8_t PIN_LIST[ ] = {0, 1, 2, 3, 4};
// microphone amplifier to A7.
const uint8_t PIN_LIST[ ] = {0};

// ADC sample rate in samples per second; must be 0.25 or greater.
const float SAMPLE_RATE = 32000;

// The interval between samples in seconds, SAMPLE_INTERVAL, may be set to a
// constant instead of being calculated from SAMPLE_RATE.  SAMPLE_RATE is not
// used in the code below.  For example, setting SAMPLE_INTERVAL = 2.0e-4
// will result in a 200 microsecond sample interval.
const float SAMPLE_INTERVAL = 1.0/SAMPLE_RATE;

// Setting ROUND_SAMPLE_INTERVAL non-zero will cause the sample interval to
// be rounded to a a multiple of the ADC clock period and will reduce sample
// time jitter.
#define ROUND_SAMPLE_INTERVAL 1

// ADC prescale factor. Do not change this. 
// The ADC takes 13 (ADC) clock cycles to perform a digitization. 
// The prescale controls the amount of prescaling of the CPU clock 
// to yield the (derived) ADC clock.
#define ADC_PRESCALER 4 // F_CPU/16 1000 kHz ADC clock on a Mega 2560

// ADC reference voltage.  See the processor data-sheet for reference details.
// uint8_t const ADC_REF = 0; // External Reference AREF pin.
// uint8_t const ADC_REF = (1 << REFS0);  // Vcc Reference.
// uint8_t const ADC_REF = (1 << REFS1);  // Internal 1.1 (only 644 1284P Mega)
uint8_t const ADC_REF = (1 << REFS1) | (1 << REFS0);  // Internal 2.56 on a Mega 2560

// Number of analog pins to log. We're only doing A7 with this routine.
const uint8_t PIN_COUNT = sizeof(PIN_LIST)/sizeof(PIN_LIST[0]);

// Minimum ADC clock cycles per sample interval
const uint16_t MIN_ADC_CYCLES = 15;

// ADC configuration for each pin.
uint8_t adcmux[PIN_COUNT];
uint8_t adcsra[PIN_COUNT];
uint8_t adcsrb[PIN_COUNT];
uint8_t adcindex = 1;

// Insure no timer events are missed.
volatile bool timerError = false;
volatile bool timerFlag = false;

/////////////////// File definitions ///////////////////

// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.

// Set the number of buffers to be written. The largest number that will actually
// work appears to be around 2 million. This corresponds to 508 million samples,
// or (at 32 kHz) 15,875 seconds of data, or about 4 hours and 24 minutes in a 
// single file. Each buffer is 512 bytes, so this is about a gigabyte.

// The following is actually one greater than the number of buffers.
// 8 GB holds 15,625,000 512 word buffers, while 2GB holds 3,906,250 buffers.
// #define MAXIMUMBUFFERSPLUSONE 631
#define MAXIMUMBUFFERSPLUSONE 2000000

const uint32_t FILE_BLOCK_COUNT = MAXIMUMBUFFERSPLUSONE;

// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;

// audio file base name.  Must be six characters or less. Files will end up with
// names like "audio12.bin"
#define FILE_BASE_NAME "audio"

// Set RECORD_EIGHT_BITS non-zero to record only the high 8-bits of the ADC.
// We want to use all 10 bits, so keep it zero.
#define RECORD_EIGHT_BITS 0

// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "tmp_log.bin"

// Audio file base name
char binName[13] = FILE_BASE_NAME "00.bin";

// Size of file base name.  Must not be larger than six characters.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

/////////////////// Miscellaneous definitions ///////////////////

// SD breakour board chip select pin. We are using the Arduino's pin 53
// to drive this.
const uint8_t SD_CS_PIN = 53; 

/////////////////// SD file buffer definitions ///////////////////

// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional
// buffers.  QUEUE_DIM must be a power of two larger than
//(BUFFER_BLOCK_COUNT + 1).

// SRAM is the kind of Arduino memory that the executing program is allowed
// to modify. A Mega 2560 has 8 kB of SRAM. Choose the number of buffers 
// based on the amount of available SRAM.

#if RAMEND < 0X8FF
#error Too little SRAM

#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 4;  // Must be a power of two!

#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 8;  // Must be a power of two!

#elif RAMEND < 0X40FF
//0X20FF = 16,639

// Initially: code specified to use total of 13 512 byte buffers (6,656 bytes).
// But that causes problems!

/*
I find that these values for BUFFER_BLOCK_COUNT and QUEUE_DIM
cause conflicts with operating a BME680 in the same program
as 32 kHz audio recording. 

// Use total of thirteen 512 byte buffers (6,656 bytes)
const uint8_t BUFFER_BLOCK_COUNT = 12;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 16;  // Must be a power of two!

*/

// use the following values to avoid problems with conflicts between
// audio recording and BME I2C operations. 4; 8

// Use total of five 512 byte buffers (2,560 bytes)
const uint8_t BUFFER_BLOCK_COUNT = 4;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 8;  // Must be a power of two!

#else  // RAMEND
// Use total of 29 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 28;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 32;  // Must be a power of two!
#endif  // RAMEND

/////////////////// More miscellaneous definitions ///////////////////

// this stuff's obscure to me. it has to do with file I/O.

#if RECORD_EIGHT_BITS
const size_t SAMPLES_PER_BLOCK = DATA_DIM8/PIN_COUNT;
typedef block8_t block_t;
#else  // RECORD_EIGHT_BITS
const size_t SAMPLES_PER_BLOCK = DATA_DIM16/PIN_COUNT;
typedef block16_t block_t;
#endif // RECORD_EIGHT_BITS

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
volatile uint8_t fullHead;  // volatile insures non-interrupt code sees changes.
uint8_t fullTail;

// queueNext assumes QUEUE_DIM is a power of two
inline uint8_t queueNext(uint8_t ht) {
  return (ht + 1) & (QUEUE_DIM -1);
}

/////////////// Interrupt Service Routine definitions ///////////////////

// Extra cpu cycles to setup ADC with more than one pin per sample.
const uint16_t ISR_SETUP_ADC = PIN_COUNT > 1 ? 100 : 0;

// Maximum cycles for timer0 system interrupt, millis, micros.
const uint16_t ISR_TIMER0 = 160;

// Pointer to current buffer.
block_t* isrBuf;

// Need new buffer if true.
bool isrBufNeeded = true;

// overrun count
uint16_t isrOver = 0;

/////////////// instantiate file system and file objects /////////////////

SdFat sd;
SdBaseFile binFile;

/*************************************************************************************
  This program is set_RTC_with_GPS.ino

  Set a DS3231 real time clock connected to an Arduino Mega 2560 based on the GPS information 
  at our disposal. I am assuming that the Arduino is talking to an Adafruit DS3231 real
  time clock breakout board and an Adafruit "Ultimate GPS" breakout board. See 
  https://www.adafruit.com/product/746. 
  
  Note that the RTC won't be set until the GPS chip sees satellites. So this
  is best done outside! The GPS-based time will be UTC, not Central Time.

  UTC is approximately the same thing as Greenwich Mean Time, which is five hours 
  later than central daylight time, and six hours later than central standard time. 
  Once the RTC is set from GPS data that are reinforced by satellite data, the program 
  will stop setting the RTC. 
 
  This "sketch" is based in part on the Adafruit/Arduino GPS library example code
  in GPS_HardwareSerial_Parsing.ino, with subsequent modifications by George Gollin, 
  University of Illinois, 2018.

  I assume that the GPS board's PPS pin is connected to Arduino pin D43. The PPS (pulse-
  per-second) pin puts out a positive ~100 ms pulse just as the GPS clock rolls over to
  the next second, so I use this to obtain RTC syncronization with the GPS
  system that is (I hope) good to a millisecond or two. See comments in the body of the program.

  The method is to fetch the UTC time from the GPS, then add one second to it. I refer to
  this as the "bumped" time. I wait until the next time the GPS PPS pin lights up, then load
  the RTC module clock with the bumped time.

  See https://www.adafruit.com/products/3133, https://www.adafruit.com/products/1059, 
  https://www.adafruit.com/products/1272, and https://www.adafruit.com/products/746.
  
  You'll want to do Tools -> Serial Monitor, then set the baud rate to 9600.
  
  Also make sure that Tools -> Port is set to the Arduino's port.
  
  The Adafruit Ultimate GPS breakout board uses a MediaTek 3339 GPS chip set.
  The NMEA ("National Marine Electronics Association") command format includes a 
  two character checksum at the end; see http://www.hhhh.org/wiml/proj/nmeaxor.html 
  for a checksum calculator.

*************************************************************************************/

// instantiate an rtc (real time clock) object:
RTC_DS3231 rtc;

// declare which Arduino pin sees the GPS PPS signal
int GPS_PPS_pin = 43;

// Which hardware serial port shall we use? Let's use the second. Why? Who knows?
#define GPSSerial Serial2

// declare variables which we'll use to store the value (0 or 1). 
int GPS_PPS_value, GPS_PPS_value_old;

// Connect the GPS to the hardware port
Adafruit_GPS GPS(&GPSSerial);

// define the synch-GPS-with-PPS command. NMEA is "National Marine Electronics 
// Association." 
#define PMTK_SET_SYNC_PPS_NMEA "$PMTK255,1*2D"

// command string to set GPS NMEA baud rate to 9,600:
#define PMTK_SET_NMEA_9600 "$PMTK251,9600*17"

// define a command to disable all NMEA outputs from the GPS except the date/time
#define PMTK_DATE_TIME_ONLY "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*29"
 
// define a command to disable ALL NMEA outputs from the GPS
#define PMTK_ALL_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
 
// define a command to enable all NMEA outputs from the GPS
#define PMTK_ALL_ON "$PMTK314,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1*29"

#define PMTK_SET_NMEA_UPDATE_10SEC "$PMTK220,10000*2F"
#define PMTK_SET_NMEA_UPDATE_5SEC "$PMTK220,5000*2F"
 
// See https://blogs.fsfe.org/t.kandler/2013/11/17/ for additional GPS definitions.

// we don't expect a valid GPS "sentence" to be longer than this...
#define GPSMAXLENGTH 120
char* GPS_sentence;
int GPS_command_string_index;

// we'll also want to convert the GPS sentence character array to a string for convenience
String GPS_sentence_string;

// pointers into parts of a GPZDA GPS data sentence whose format is
//    $GPZDA,hhmmss.sss,dd,mm,yyyy,xx,xx*CS 
//              111111111122222222223
//    0123456789012345678901234567890             
// where CS is a two-character checksum. Identify this sentence by the presence of a Z.

const int GPZDA_hour_index1 = 7;
const int GPZDA_hour_index2 = GPZDA_hour_index1 + 2;

const int GPZDA_minutes_index1 = GPZDA_hour_index2;
const int GPZDA_minutes_index2 = GPZDA_minutes_index1 + 2;
      
const int GPZDA_seconds_index1 = GPZDA_minutes_index2;
const int GPZDA_seconds_index2 = GPZDA_seconds_index1 + 2;
      
const int GPZDA_milliseconds_index1 = GPZDA_seconds_index2 + 1;   // skip the decimal point
const int GPZDA_milliseconds_index2 = GPZDA_milliseconds_index1 + 3;
      
const int GPZDA_day_index1 = GPZDA_milliseconds_index2 + 1;  // skip the comma
const int GPZDA_day_index2 = GPZDA_day_index1 + 2;
      
const int GPZDA_month_index1 = GPZDA_day_index2 + 1;
const int GPZDA_month_index2 = GPZDA_month_index1 + 2;

const int GPZDA_year_index1 = GPZDA_month_index2 + 1;
const int GPZDA_year_index2 = GPZDA_year_index1 + 4;

// define some time variables.
unsigned long  time_ms_bumped_RTC_time_ready;

// system time from millis() at which the most recent GPS date/time
// sentence was first begun to be read
unsigned long t_GPS_read_start;

// system time from millis() at which the most recent GPS date/time
// sentence was completely parsed 
unsigned long t_GPS_read;

// system time from millis() at which the proposed bumped-by-1-second
// time is ready for downloading to the RTC
unsigned long t_bump_go; 
                
// system time from millis() at which the most recent 0 -> 1 
// transition on the GPS's PPS pin is detected
unsigned long t_GPS_PPS;  

// system time from millis() at which the RTC time load is done 
unsigned long t_RTC_update;

// keep track of whether or not we have set the RTC using satellite-informed GPS data
bool good_RTC_time_from_GPS_and_satellites;

// define some of the (self-explanatory) GPS data variables. Times/dates are UTC.
String GPS_hour_string;
String GPS_minutes_string;
String GPS_seconds_string;
String GPS_milliseconds_string;
int GPS_hour;
int GPS_minutes;
int GPS_seconds;
int GPS_milliseconds;

String GPS_day_string;
String GPS_month_string;
String GPS_year_string;
int GPS_day;
int GPS_month;
int GPS_year;

// RTC variables...
int RTC_hour;
int RTC_minutes;
int RTC_seconds;
int RTC_day;
int RTC_month;
int RTC_year;

// we will use the following to update the real time clock chip.
int RTC_hour_bumped;
int RTC_minutes_bumped;
int RTC_seconds_bumped;
int RTC_day_bumped;
int RTC_month_bumped;
int RTC_year_bumped;

// define a "DateTime" object:
DateTime now;

// limits for the timing data to be good:
const int t_RTC_update__t_GPS_PPS_min = -1;
const int t_GPS_PPS___t_bump_go_min = 200;
const int t_bump_go___t_GPS_read_min = -1;
const int t_RTC_update___t_GPS_read_min = 400;

const int t_RTC_update__t_GPS_PPS_max = 20;
const int t_GPS_PPS___t_bump_go_max = 800;
const int t_bump_go___t_GPS_read_max = 350;
const int t_RTC_update___t_GPS_read_max = 1000;

// more bookkeeping on clock setting... I will want to see several consecutive
// good reads/parses of GPS system time data to declare that all is good, and that
// we can wrap this up.
int consecutive_good_sets_so_far;
bool time_to_quit;
const int thats_enough = 5;

// the only kind of GPS sentence that can hold a Z, that I am allowing from the GPS,
// will carry date/time information.
bool sentence_has_a_Z;

// it's obvious what these are:
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", 
  "Thursday", "Friday", "Saturday"};

// times for the arrival of a new data sentence and the receipt of its last character
unsigned long t_new_sentence;
unsigned long t_end_of_sentence;

bool GPSECHO_loop = true;

// a "function prototype" so I can put the actual function at the end of the file:
void bump_by_1_sec(void);
void gpsLoop();
void gpsSetup();
int gpsQuery();

// a counter
int i_am_so_bored;

/////////////////////////////////////////////////////////////////////////

// Set GPSECHO_GPS_query to 'false' to turn off echoing the GPS data to the Serial console  
// from the GPS_query function. (Set it to true when debugging.)
#define GPSECHO_GPS_query true

// a similar debugging flag for loop():
#define GPSECHO_loop true

// also define some more stuff relating to update rates. See 
// https://blogs.fsfe.org/t.kandler/2013/11/17/set-gps-update-
// rate-on-arduino-uno-adafruit-ultimate-gps-logger-shield/
#define PMTK_SET_NMEA_UPDATE_10SEC "$PMTK220,10000*2F"
#define PMTK_SET_NMEA_UPDATE_5SEC "$PMTK220,5000*2F"
     
// we don't expect a valid GPS "sentence" to be longer than this...
#define GPSMAXLENGTH 120
// or shorter than this:
#define GPSMINLENGTH 55

// a string to hold what kind of GPS sentence we've received
String GPS_command;

// pointers into parts of a GPRMC GPS data sentence:

// pointers into parts of a GPRMC GPS data sentence:

// old: const int GPRMC_hour_index1 = 8;
const int GPRMC_hour_index1 = 7;
const int GPRMC_hour_index2 = GPRMC_hour_index1 + 2;

const int GPRMC_minutes_index1 = GPRMC_hour_index2;
const int GPRMC_minutes_index2 = GPRMC_minutes_index1 + 2;
      
const int GPRMC_seconds_index1 = GPRMC_minutes_index2;
const int GPRMC_seconds_index2 = GPRMC_seconds_index1 + 2;
      
const int GPRMC_milliseconds_index1 = GPRMC_seconds_index2 + 1;   // skip the decimal point
const int GPRMC_milliseconds_index2 = GPRMC_milliseconds_index1 + 3;
      
// const int GPRMC_AV_code_index1 = 19;
const int GPRMC_AV_code_index1 = GPRMC_hour_index1 +  11;
const int GPRMC_AV_code_index2 = GPRMC_AV_code_index1 + 1;
      
// const int GPRMC_latitude_1_index1 = 21;
const int GPRMC_latitude_1_index1 = GPRMC_AV_code_index1 + 2;
const int GPRMC_latitude_1_index2 = GPRMC_latitude_1_index1 + 4;
      
const int GPRMC_latitude_2_index1 = GPRMC_latitude_1_index2 + 1;   // skip the decimal point
const int GPRMC_latitude_2_index2 = GPRMC_latitude_2_index1 + 4;

// const int GPRMC_latitude_NS_index1 = 31;
const int GPRMC_latitude_NS_index1 = GPRMC_latitude_1_index1 + 10;
const int GPRMC_latitude_NS_index2 = GPRMC_latitude_NS_index1 + 1;

// const int GPRMC_longitude_1_index1 = 33;
const int GPRMC_longitude_1_index1 = GPRMC_latitude_NS_index1 + 2;
const int GPRMC_longitude_1_index2 = GPRMC_longitude_1_index1 + 5;    // 0 - 180 so we need an extra digit
      
const int GPRMC_longitude_2_index1 = GPRMC_longitude_1_index2 + 1;   // skip the decimal point
const int GPRMC_longitude_2_index2 = GPRMC_longitude_2_index1 + 4;
      
// const int GPRMC_longitude_EW_index1 = 44;
const int GPRMC_longitude_EW_index1 = GPRMC_longitude_1_index1 + 11;
const int GPRMC_longitude_EW_index2 = GPRMC_longitude_EW_index1 + 1;

// pointers into a GPGGA GPS data sentence:

// old: const int GPGGA_hour_index1 = 8;
const int GPGGA_hour_index1 = 7;
const int GPGGA_hour_index2 = GPGGA_hour_index1 + 2;

const int GPGGA_minutes_index1 = GPGGA_hour_index2;
const int GPGGA_minutes_index2 = GPGGA_minutes_index1 + 2;
      
const int GPGGA_seconds_index1 = GPGGA_minutes_index2;
const int GPGGA_seconds_index2 = GPGGA_seconds_index1 + 2;
      
const int GPGGA_milliseconds_index1 = GPGGA_seconds_index2 + 1;   // skip the decimal point
const int GPGGA_milliseconds_index2 = GPGGA_milliseconds_index1 + 3;
      
// const int GPGGA_latitude_1_index1 = 19;
const int GPGGA_latitude_1_index1 = GPGGA_hour_index1 + 11;
const int GPGGA_latitude_1_index2 = GPGGA_latitude_1_index1 + 4;
      
const int GPGGA_latitude_2_index1 = GPGGA_latitude_1_index2 + 1;   // skip the decimal point
const int GPGGA_latitude_2_index2 = GPGGA_latitude_2_index1 + 4;

// const int GPGGA_latitude_NS_index1 = 29;
const int GPGGA_latitude_NS_index1 = GPGGA_latitude_1_index1 + 10;
const int GPGGA_latitude_NS_index2 = GPGGA_latitude_NS_index1 + 1;

// const int GPGGA_longitude_1_index1 = 31;
const int GPGGA_longitude_1_index1 = GPGGA_latitude_NS_index1 + 2;
const int GPGGA_longitude_1_index2 = GPGGA_longitude_1_index1 + 5;    // 0 - 180 so we need an extra digit
      
const int GPGGA_longitude_2_index1 = GPGGA_longitude_1_index2 + 1;   // skip the decimal point
const int GPGGA_longitude_2_index2 = GPGGA_longitude_2_index1 + 4;
      
// const int GPGGA_longitude_EW_index1 = 42;
const int GPGGA_longitude_EW_index1 = GPGGA_longitude_1_index1 + 11;
const int GPGGA_longitude_EW_index2 = GPGGA_longitude_EW_index1 + 1;

// const int GPGGA_fix_quality_index1 = 44;
const int GPGGA_fix_quality_index1 = GPGGA_longitude_EW_index1 + 2;
const int GPGGA_fix_quality_index2 = GPGGA_fix_quality_index1 + 1;

// const int GPGGA_satellites_index1 = 46;
const int GPGGA_satellites_index1 = GPGGA_fix_quality_index1 + 2;
const int GPGGA_satellites_index2 = GPGGA_satellites_index1 + 2;

// distance in from the end of the sentence for the asterisk
const int asterisk_backup = 5;

// keep track of how many times we've read a character from the GPS device. 
long GPS_char_reads = 0;

// bail out if we exceed the following number of attempts. when set to 1,000,000 this corresponds
// to about 6 seconds. we need to do this to keep an unresponsive GPS device from hanging the program.
const long GPS_char_reads_maximum = 1000000;

// this one tells us about data validity: A is good, V is invalid.
String GPS_AV_code_string;

// latitude data
String GPS_latitude_1_string;
String GPS_latitude_2_string;
String GPS_latitude_NS_string;
int GPS_latitude_1;
int GPS_latitude_2;

// longitude data
String GPS_longitude_1_string;
String GPS_longitude_2_string;
String GPS_longitude_EW_string;
int GPS_longitude_1;
int GPS_longitude_2;

// velocity information; speed is in knots! 
String GPS_speed_knots_string;
String GPS_direction_string;
float GPS_speed_knots;
float GPS_direction;

String GPS_date_string;

String GPS_fix_quality_string;
String GPS_satellites_string;
int GPS_fix_quality;
int GPS_satellites;

String GPS_altitude_string;
float GPS_altitude;

// Let's limit the numnber of times we will spin thrugh the "loop" function before
// closing the data file and parking the program in an infinite loop.
const long maximum_times_to_loop = 20000000;
long my_counter;


//////////////////////////////////////////////////////////////////////
//////////////////// end of global parameters ////////////////////////
//////////////////////////////////////////////////////////////////////
