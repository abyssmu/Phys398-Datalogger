/*
test_suite.iso

George Gollin, University of Illinois, September 2020
 
test soldered physics Fall 2020 398DLP PCBs before distributing to students.

I am doing all the soldering of the PCBs this semester, since I am teaching 
the course remotely. To save time, frustration, irritation, and expense, I 
should test the boards before distributing them. Most boards will have
•	battery pack 
•	red LED 
•	Arduino Mega 2560
•	GPS
•	RTC
•	LCD
•	microphone
•	keypad
•	BME680
•	microSD breakout
•	INA219 (a few might have two installed)

Here’s the procedure. Solder the capacitors, trimpot, LED, 10k resistor, 
pushbutton, and required headers. Solder the leads from the battery pack 
and install. Solder leads to the Arduino and install.

•	Insert breakout boards into headers; install AA batteries. 
  Adjust LCD contrast.
•	Click the pushbutton and check that LED and GPS LED illuminate. 
  This will test the battery pack and red LED.
•	When GPS has satellites, run set_RTC_with_GPS.ino. This will test 
  the Arduino, GPS, RTC, and LCD.
•	Run audio_rms_1_second_v3.ino to test the microphone, of course.
•	Run keypad.ino for the obvious reason.
•	Run bme680test2020.ino.
•	Run ReadWrite2.ino to check the microSD breakout.
•	Run INA219_current_monitor.ino with and without the battery powering 
  the circuit (just use the pushbutton).

I am patching the above-mentioned sketches into this program.

*/

#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_INA219.h>
#include <Keypad.h>
//#include <SD.h>
#include "SdFat.h"
#include "RTClib.h"
#include <Adafruit_GPS.h>
// this is from Github:
#include <ArduinoUniqueID.h>

SdFat SD;

// instantiate an rtc (real time clock) object:
RTC_DS3231 rtc;

/////////////////////////// GPS parameters //////////////////////////

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
 
// See https://blogs.fsfe.org/t.kandler/2013/11/17/ for additional GPS definitions.

// we don't expect a valid GPS "sentence" to be longer than this...
#define GPSMAXLENGTH 120
char GPS_sentence[GPSMAXLENGTH];
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
  
// get the LCD display header file.
#include <LiquidCrystal.h>

// it's obvious what these are:
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", 
  "Thursday", "Friday", "Saturday"};

// times for the arrival of a new data sentence and the receipt of its last character
unsigned long t_new_sentence;
unsigned long t_end_of_sentence;

// a "function prototype" so I can put the actual function at the end of the file:
void bump_by_1_sec(void);

/////////////////////////// LCD parameters //////////////////////////

// initialize the LCD library by associating any needed LCD interface pins
// with the arduino pin numbers to which they are connected
const int rs = 12, en = 11, d4 = 36, d5 = 34, d6 = 32, d7 = 30;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// a counter
int i_am_so_bored;

// #define SD_CS_PIN 13
// SD chip select pin. We are jusing the Arduino's pin 53.
const uint8_t SD_CS_PIN = SS; 
// const uint8_t SD_CS_PIN = 53; 

File myFile;

// instantiate the objects representing the two INA219s:

const int I2C_default_address = 0x40;
Adafruit_INA219 ina219_a;

const int I2C_second_address = 0x41;
Adafruit_INA219 ina219_b(I2C_second_address);

// do we see both INA219s on the I2C lines?
bool Have_2_INA219s;

////////////////// microphone and sound recording parameters /////////////////

// select the analog input pin for the microphone
const int microphonePin = A7;

// ADC value each time we read the microphone
long int adc_value;

// flag for when we have a new ADC read.
bool new_ADC_data;

// number of intervals so far
long intervals_so_far;

///////////////////////////// 5 kHz read rate parameters /////////////////////////// 

// here's a function to clear one register bit without touching any of the other bits:
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

// same idea, but to set one bit:
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// timer1 interrupt parameter: since I will get an interrupt when the 16-bit
// timer overflows (exceeds 65,535), set the initial timer1 value to get a 5 kHz sampling rate.

// here's how this works. the period for a 5 kHz read rate is 200 microseconds. The Arduino
// clock ticks 16 times in 1 microsecond, so 5 kHz corresponds to 3,200 clock ticks. If we set
// timer1 to an initial value of 65,536 - 3,200 = 62,336 we'll get a timer1 overflow in about 200 
// microseconds. But there is some latency and overhead associated with handling the interrupt,
// so we'll actually want to set it about 55 clock ticks closer to overflow than this.  

// 5 kHz =  200 microsecond period = 3200 - 55 clock ticks = 3145
// 1 kHz = 1,000 microsecpond period = 16,000 - 55 clock ticks = 15,945

// set to 62,391 for 5 kHz
#define TIMER1_INITIAL_VALUE 62391 
// #define TIMER1_INITIAL_VALUE 49591 
// this is the variable that we'll use to load the initial timer1 value.
int timer1_counter_initial;

// number of samples to include in our averages
long how_many_samples_to_include;

// sample counter, reset periodically
long sample_counter;

// used in calculating average and average of squares...
long sum_amplitudes;
unsigned long sum_amplitudes_squared;

// baseline ADC value we'll subtract from everything to avoid overflows
// when calculating sums
long int baseline;

// number of samples we'll take to establish the baseline
#define BASELINESAMPLES 1000 

// averages
float average, average_square, RMS, average_of_RMSs, sum_of_RMSs;

// we'll want to keep track of how long we read the ADC before reporting the 
// average values..
unsigned long time_start;
unsigned long time_finish;

#define PLOTSTUFF false

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// our keypad has four rows and three columns. since this will never change
// while the program is runniong, declare these as constants so that they 
// will live in flash (program code) memory instead of the much smaller
// SRAM. 
const byte ROWS = 4; 
const byte COLS = 3;

// keypad layout
char keys[ROWS][COLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'}
};

// looking down on the keyboard from above (the side with the keys), the pins are
// numbered 1 - 8, going from left to right, though 8 is not used. 

// Since I am using an Arduino Mega 2560 with a number of breakout boards, 
// I have the following Arduino pin assignments in order to allow the column pins to 
// generate interrupts in some future version of this program.
byte Arduino_colPins[COLS] = {2, 3, 18}; 
byte Arduino_rowPins[ROWS] = {31, 33, 35, 37};

// now instantiate a Keypad object, call it kpd. Also map its pins.
Keypad kpd = Keypad( makeKeymap(keys), Arduino_rowPins, Arduino_colPins, ROWS, COLS );

uint32_t t_mic_start;

///////////////////////////////////////////////////////////////////////////////
///////////////////////// setup function //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  while (!Serial);
 
  ///////////////////////////////////////////////////////////setup()
  // Get the serial number for the processor board. 

  Serial.println(
    F("\n\n*******************************************************************"));
  
  Serial.println(F("Physics 398DLP PCB test"));

  Serial.print(F("This Arduino's (hex) serial number: "));
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    if (UniqueID[i] < 0x10) Serial.print(F("0"));
    Serial.print(UniqueID[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println(F(" "));

  Serial.println(F("file holding this program is ")); Serial.println(__FILE__);

  // get the time at which the program was compiled just before upload.
  // this'll be several seconds behind the actual time, should we decide 
  // to use it to set the RTC.

  Serial.print(F("System date and time of program compilation: "));
  Serial.print(__DATE__);
  Serial.print(F("  ")); Serial.println(__TIME__);
  Serial.println(F(" "));

  Serial.println(
    F("*******************************************************************\n"));

///////////////////// ADC setup inside the Arduino ////////////////////

  // Enable the ADC by setting the ADEN bit in the ADCSRA register.
  sbi(ADCSRA, ADEN);

  // The ADC takes 13 of its own clock cycles to do one digitization.
  // Its clock is a prescaled version of the 16 MHz system clock. Set the prescales
  // here:
  
  // prescale  ADPS2, ADPS1, ADPS0   clock MHz   prescale kHz
  // 2             0 0 1                8           615
  // 4             0 1 0                4           307
  // 8             0 1 1                2           153
  // 16            1 0 0                1           76.8
  // 32            1 0 1                0.5         38.4
  // 64            1 1 0                0.25        19.2
  // 128           1 1 1                0.125       9.6

  // let's use a prescale of 16: ADPS2, ADPS1, and ADPS0 are set to 1, 0, 0.
  // faster than this and the ADC becomes inaccurate. Slower than this doesn't
  // yield an appreciable increase in accuracy.

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  // Decide what voltage source the Arduino will use to as the top of the 
  // ADC range. Possible values: EXTERNAL (ARef pin is powered by the user);
  // INTERNAL2V56 (internal 2.56V reference); INTERNAL1V1 (internal 1.1 V reference);
  // DEFAULT (VCC, nominally 5 V.)

  // int myAnalogReference = INTERNAL2V56;
  // Serial.println("Just set ADC analog reference to 2.56 volts");
  int myAnalogReference = DEFAULT;
  Serial.println("Just set ADC analog reference to default (5V)");
  analogReference(myAnalogReference);

  // now read the ADC a hundred times so we can force it to uptake its parameters. I
  // am finding that the ADC is touchy, and its uptake of parameters does not always 
  // happen as one is led to expect from the manufacturer's documentation.
  for (int i = 0; i < 100; i++) {analogRead(A0);}
  // wait 15 ms just to be sure ADC is all done.
  delay(15);

  // no new ADC data yet...
  new_ADC_data = false;
  
  // fire up the serial output with a really high baud rate
  Serial.begin(115200);

  // specify the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Print the first line (line 0) of a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Open a serial   ");
  //         0123456789012345

  // now set the LCD cursor to column 0, line 1, then finish the message.
  lcd.setCursor(0, 1);
      if(!PLOTSTUFF)
      {
        lcd.print("monitor window.  ");
      } else {
        lcd.print("plotter window.  ");      
      }
      
  // delay so user can read the LCD message
  delay(2000);

  // at 5 kHz we'll want 5,000 samples for a one second measurement.
  how_many_samples_to_include = 35000;
  
  // initialize stuff
  sum_amplitudes = 0;
  sum_amplitudes_squared = 0;
  sample_counter = 0;
  intervals_so_far = 0;
  sum_of_RMSs = 0.0;

  // read the ADC a thousand times to determine a baseline value that we'll
  // subtract from everything.

  long baseline_sum = 0;

  for (int index = 0; index < BASELINESAMPLES; index++) 
    {baseline_sum = baseline_sum + analogRead(microphonePin);};

  baseline = long(baseline_sum / BASELINESAMPLES);

  Serial.print("After reading ADC a few hundred times, ADC baseline is found to be ");
  Serial.println(baseline);

  Serial.println(F("Initializing the BME680..."));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  
  // Initialize the INA219s.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  
  ina219_a.begin();
  ina219_b.begin();

  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219_a.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219_a.setCalibration_16V_400mA();

  Serial.println("Initializing the voltage- and current-measuring INA219(s)...");

  // Let's see if we have an INA219 at both addresses. Do this
  // by using the return value of Wire.endTransmission.

  byte error;

  // default address for an INA219:
  byte address = I2C_default_address; 

  Serial.print("looking for INA219 with I2C address 0x"); Serial.print(address, HEX);
  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if(error == 0)
  {
    Serial.println("... found the device.");
  }  else {
    Serial.println("... device not found!");    
  }

  // address for a second INA219:
  address = I2C_second_address; 

  Serial.print("looking for INA219 with I2C address 0x"); Serial.print(address, HEX);
  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if(error == 0)
  {
    Serial.println("... found the device.\n");
    Have_2_INA219s = true;
  }  else {
    Serial.println("... device not found!\n");    
    Have_2_INA219s = false;
  }

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print("Battery V & I:   ");
  lcd.setCursor(0, 1);
  //         0123456789012345
  lcd.print("INA.219 sensor  ");

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  Serial.println("************************************************************");
  Serial.println("Test microSD card");

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Try writing three lines to SD file test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.println("testing 4, 5, 6.");
    myFile.println("testing 7, 8, 9.");
    // close the file:
    myFile.close();
    Serial.println(" all done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening SD file test.txt");
  }

  // re-open the file for reading:
  Serial.print("Now try reading/echoing the entire SD file...");

  myFile = SD.open("test.txt");
  if (myFile) {

    Serial.println("   test.txt opened successfully.");
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();

  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  Serial.println("************************************************************\n");

  Serial.println("\nNow do some GPS setup.");

  // declare the GPS PPS pin to be an Arduino input 
  pinMode(GPS_PPS_pin, INPUT);

  // initialize a flag and some counters
  good_RTC_time_from_GPS_and_satellites = false;
  consecutive_good_sets_so_far = 0;
  i_am_so_bored = 0;

  // 9600 NMEA is the default communication and baud rate for Adafruit MTK 3339 chipset GPS 
  // units. NMEA is "National Marine Electronics Association." 
  // Note that this serial communication path is different from the one driving the serial 
  // monitor window on your laptop.
  GPS.begin(9600);

  // initialize a flag holding the GPS PPS pin status: this pin pulses positive as soon as 
  // the seconds value rolls to the next second.
  GPS_PPS_value_old = 0;
    
  // turn off most GPS outputs to reduce the rate of stuff coming at us.
  GPS.sendCommand(PMTK_DATE_TIME_ONLY);

  // Set the update rate to once per second. 
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

  // Send a synch-with-PPS command to the GPS in hopes of having a deterministic
  // relationship between the PPS line lighting up and the GPS reporting data to us. According
  // to the manufacturer, the GPS will start snding us a date/time data sentence about 170
  // milliseconds after the PPS line transitions fom 0 to 1. 
  GPS.sendCommand(PMTK_SET_SYNC_PPS_NMEA);
  
  // this keeps track of where in the string of characters of a GPS data sentence we are.
  GPS_command_string_index = 0;

  // more initialization
  sentence_has_a_Z = false;

  time_to_quit = false;

  // fire up the RTC.
  Serial.print("Fire up the RTC. return code is "); 
  int return_code = rtc.begin();
  Serial.println(rtc.begin());

  // problems?
  if(!return_code) {
    Serial.println("RTC wouldn't respond so bail out.");
    while (1) {};
  }

  // now try read back the RTC to check.       
  delay(500);
  
  now = rtc.now();
  Serial.print("Now read back the RTC to check during setup. ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if(now.minute() < 10)   Serial.print(0);
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if(now.second() < 10)   Serial.print(0);
  Serial.print(now.second(), DEC);

  Serial.print("   Date (dd/mm/yyyy): ");
  Serial.print(now.day(), DEC); Serial.print('/');
  if(int(now.month()) < 10) Serial.print("0");
  Serial.print(now.month(), DEC); Serial.print("/");
  Serial.println(now.year(), DEC);
  
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Now looking for ");
  lcd.setCursor(0, 1);
  lcd.print("GPS satellites  ");

  // also take note of the time at which we are ready to start recording
  time_start = millis();

  Serial.println("************************************************************");
  Serial.println("Measure RMS microphone noise for ten seconds");

  // only read the microphone for ten seconds.
  while(millis() - time_start < 10000)
  {
    ReadADC();
    
    // new ADC data?
    if (new_ADC_data)
    {
      new_ADC_data = false;
      
      // increment the number of samples
      sample_counter++;

      // now add this into the various sums, possibly subtracting the baseline.
      long int subtracted_value = adc_value - baseline;
      // long int subtracted_value = adc_value;
      sum_amplitudes = sum_amplitudes + subtracted_value;
      sum_amplitudes_squared = sum_amplitudes_squared + subtracted_value * subtracted_value;

      // time to calculate an average?
      if (sample_counter >= how_many_samples_to_include)
      {
        // take note of the time.
        time_finish = millis();

        // increment the number of 1-second intervals we have done
        intervals_so_far++;

        // calculate averages
        average = float(sum_amplitudes) / float(how_many_samples_to_include);
        average_square = float(sum_amplitudes_squared) / float(how_many_samples_to_include);

        float mean_sq_deviation = average_square - average*average;
        RMS = sqrt(mean_sq_deviation);

        sum_of_RMSs = sum_of_RMSs + RMS;
        average_of_RMSs = sum_of_RMSs / intervals_so_far;

        Serial.print("\naverage (after baseline subtraction) = "); Serial.println(average); 
        // Serial.print("\naverage (without baseline subtraction) = "); Serial.println(average); 
        Serial.print("RMS = "); Serial.println(RMS); 
        Serial.print("baseline = "); Serial.println(baseline); 

        // reset stuff
        sample_counter = 0;
        sum_amplitudes = 0;
        sum_amplitudes_squared = 0;

        lcd.setCursor(0, 0);
        lcd.print("RMS "); lcd.print(RMS); lcd.print(" ");   
        lcd.setCursor(0, 1);
        lcd.print("<RMS> "); lcd.print(average_of_RMSs); lcd.print("     "); 

      }
    }

  } 
  Serial.println("************************************************************\n");

  Serial.println("Now read BME680 for ten seconds");
  
  while (millis() - time_start < 20000) {

    // read the BME680 for a similar interval.
    if (! bme.performReading()) {
      Serial.println("Failed to perform BME680 reading :(");
      return;
    }

    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.print(" *C     ");
    Serial.print(1.8 * bme.temperature + 32.0);
    Serial.println(" *F"    );

    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0, 4);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");

    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m\n");

    delay(1000);
  }
  Serial.println("done with BME680 measurements \n");  

  Serial.println("************************************************************");
  Serial.println("Please type stuff on the keypad for ten seconds");  

  while(millis() - time_start < 30000) 
  {
    // we'll read a character from the keypad.
    char the_key;
    
    // kpd.getKeys fills kpd.key[ ] array with up to 10 active keys, if the user pushes
    // more than one key at the same time. But I am only going to look at the 
    // first key pressed. 
    
    // The class function getKeys() returns true if there are any active keys. Note 
    // that "active" is true when a key is depressed, but that the state of an active 
    // key can be "PRESSED" (newly depressed), HOLD (still depressed), or RELEASED (just 
    // released, of course).

    // any newly-pressed keys?
    if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {

      the_key =  kpd.key[0].kchar;
      Serial.print("Just detected the "); Serial.print(the_key); Serial.println(" key.");

      // let's see if this is an octothorpe (# key). Note the use of single 
      // (not double) quotes.
      if(the_key == '#') {Serial.println("Wow, that's an octothorpe!");}
    }
  }

  Serial.println("\ndone with keypad trials \n");  

  Serial.println("************************************************************");

  Serial.println("\nNow look at INA219(s) for ten seconds. Push the power button a few times.\n");  

  while(millis() - time_start < 40000) 
  {

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    shuntvoltage = ina219_a.getShuntVoltage_mV();
    busvoltage = ina219_a.getBusVoltage_V();
    current_mA = ina219_a.getCurrent_mA();
    power_mW = ina219_a.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print("***** INA219 with address 0x");
    Serial.print(I2C_default_address, HEX);
    Serial.println(" *****");
    Serial.print("Load (V+) Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Bus (V-) Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");

    if(Have_2_INA219s)
    {
      
      // library calls to get values for the second INA219
      shuntvoltage = ina219_b.getShuntVoltage_mV();
      busvoltage = ina219_b.getBusVoltage_V();
      loadvoltage = busvoltage + (shuntvoltage / 1000);
      current_mA = ina219_b.getCurrent_mA();
      power_mW = ina219_b.getPower_mW();
    
      Serial.print("***** INA219 with address 0x");
      Serial.print(I2C_second_address, HEX);
      Serial.println(" *****");
      Serial.print("Load (V+) Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
      Serial.print("Bus (V-) Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
      Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
      Serial.println("");
    
      // Print a message to the LCD.
      lcd.setCursor(0, 0);
      //         0123456789012345
      lcd.print("battery V = ");
      lcd.setCursor(12, 0);
      lcd.print(busvoltage);
    
      lcd.setCursor(0, 1);
      lcd.print("I (mA) = ");
      lcd.setCursor(9, 1);
      lcd.print(current_mA);
  
    } else {
  
      // Print a message to the LCD.
      lcd.setCursor(0, 0);
      //         0123456789012345
      lcd.print("5 x AA pack V = ");
      lcd.setCursor(12, 0);
      lcd.print(busvoltage);
    
      lcd.setCursor(0, 1);
      lcd.print("I (mA) = ");
      lcd.setCursor(9, 1);
      lcd.print(current_mA);
      
    }
      
    // delay a bit...
    
    delay(1000);
  }

  Serial.println("Done with INA219 tests.");
  Serial.println("************************************************************\n");

  Serial.println("Now do some GPS and RTC-setting stuff.\n");

}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// loop function //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void loop() {

  // ******************************************************************************
  /*

  First things first: check to see if we are we done setting the RTC. In order to 
  declare victory and exit, we'll need the following to happen. 

  Definitions:

    t_GPS_read    system time from millis() at which the most recent GPS date/time
                  sentence was completely parsed BEFORE the most recent PPS 0 -> 1 
                  transition was detected 
                      
    t_bump_go     system time from millis() at which the proposed bumped-by-1-second
                  time is ready for downloading to the RTC
    
    t_GPS_PPS     system time from millis() at which the most recent 0 -> 1 
                  transition on the GPS's PPS pin is detected

    t_RTC_update  system time from millis() at which the RTC time load is done 

  Typical timing for an event:   

    t_GPS_read    17,961    
    t_bump_go     17,971 (t_GPS_read +  10 ms)    
    t_GPS_PPS     18,597 (t_bump_go  + 626 ms)    
    t_RTC_update  18,598 (t_GPS_PPS  +   1 ms)

  Every once in a while we might miss the PPS 0 -> 1 transition, or the GPS might 
  not feed us a data sentence. So let's impose the following criteria.

  0 ms   <= t_RTC_update - t_GPS_PPS  <= 10 ms
  200 ms <= t_GPS_PPS - t_bump_go     <= 800 ms
  0 ms   <= t_bump_go - t_GPS_read    <= 50 ms
  400 ms <= t_RTC_update - t_GPS_read <= 1000 ms

  */

  if(time_to_quit) {

    // print a message to the serial monitor, but only once.
    if (i_am_so_bored == 0) Serial.print("\n\nTime to quit! We have set the RTC.");

    // Print a message to the LCD each pass through, updating the time.
    lcd.setCursor(0, 0);
    //         0123456789012345
    lcd.print("RTC is now set  ");

    // blank the LCD's second line 
    lcd.setCursor(0, 1);
    lcd.print("                ");

    // print the time
    lcd.setCursor(0, 1);
    now = rtc.now();
    
    if(now.hour() < 10)   lcd.print(0);
    lcd.print(now.hour(), DEC);
    
    lcd.print(':');
    if(now.minute() < 10)   lcd.print(0);
    lcd.print(now.minute());
    
    lcd.print(':');
    if(now.second() < 10)   lcd.print(0);
    lcd.print(now.second());

    delay(50);

    // increment a counter
    i_am_so_bored++;

    return;
  }

  // *******************************************************************************

  // now check to see if we just got a PPS 0 -> 1 transition, indicating that the
  // GPS clock has just ticked over to the next second.
  GPS_PPS_value = digitalRead(GPS_PPS_pin);
  
  // did we just get a 0 -> 1 transition?
  if (GPS_PPS_value == 1 && GPS_PPS_value_old == 0) {
    
    Serial.print("\nJust saw a PPS 0 -> 1 transition at time (ms) = ");
    t_GPS_PPS = millis();
    Serial.println(t_GPS_PPS);

    // load the previously established time values into the RTC now.
    if (good_RTC_time_from_GPS_and_satellites) {

      // now set the real time clock to the bumped-by-one-second value that we have 
      // already calculated. To set the RTC with an explicit date & time, for example 
      // January 21, 2014 at 3am you would call
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    
      rtc.adjust(DateTime(int(RTC_year_bumped), int(RTC_month_bumped), int(RTC_day_bumped), int(RTC_hour_bumped), 
        int(RTC_minutes_bumped), int(RTC_seconds_bumped)));

      // take note of when we're back from setting the real time clock:
      t_RTC_update = millis();

      // Serial.print("Just returned from updating RTC at system t = "); Serial.println(t_RTC_update);

      Serial.print("Proposed new time fed to the RTC was ");
      Serial.print(RTC_hour_bumped, DEC); Serial.print(':');
      if(RTC_minutes_bumped < 10) Serial.print("0");
      Serial.print(RTC_minutes_bumped, DEC); Serial.print(':');
      if(RTC_seconds_bumped < 10) Serial.print("0");
      Serial.print(RTC_seconds_bumped, DEC); 
      Serial.print("   Date (dd/mm/yyyy): ");
      Serial.print(RTC_day_bumped, DEC); Serial.print('/');
      if(RTC_month_bumped < 10) Serial.print("0");
      Serial.print(RTC_month_bumped, DEC); Serial.print("/");
      Serial.println(RTC_year_bumped, DEC);  

      // now read back the RTC to check.       
      now = rtc.now();
      Serial.print("Now read back the RTC to check. ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      if(now.minute() < 10)   Serial.print(0);
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      if(now.second() < 10)   Serial.print(0);
      Serial.print(now.second(), DEC);

      Serial.print("   Date (dd/mm/yyyy): ");
      Serial.print(now.day(), DEC); Serial.print('/');
      if(int(now.month()) < 10) Serial.print("0");
      Serial.print(now.month(), DEC); Serial.print("/");
      Serial.println(now.year(), DEC);
      
      // now that we've used this GPS value, set the following flag to false:
      good_RTC_time_from_GPS_and_satellites = false;

      // Check that the times of various events is consistent with a good RTC setting
  
      bool ILikeIt = 
      int(t_RTC_update - t_GPS_PPS)  >= t_RTC_update__t_GPS_PPS_min   &&
      int(t_GPS_PPS - t_bump_go)     >= t_GPS_PPS___t_bump_go_min     &&
      int(t_bump_go - t_GPS_read)    >= t_bump_go___t_GPS_read_min    &&
      int(t_RTC_update - t_GPS_read) >= t_RTC_update___t_GPS_read_min &&
      int(t_RTC_update - t_GPS_PPS)  <= t_RTC_update__t_GPS_PPS_max   &&
      int(t_GPS_PPS - t_bump_go)     <= t_GPS_PPS___t_bump_go_max     &&
      int(t_bump_go - t_GPS_read)    <= t_bump_go___t_GPS_read_max    &&
      int(t_RTC_update - t_GPS_read) <= t_RTC_update___t_GPS_read_max ;
    
      if(ILikeIt) {
        consecutive_good_sets_so_far++;
      }else{
        consecutive_good_sets_so_far = 0;
      }
     
      time_to_quit = consecutive_good_sets_so_far >= thats_enough;

    }

  }

  GPS_PPS_value_old = GPS_PPS_value;

  // *******************************************************************************
  // read data from the GPS; do this one character per pass through function loop.
  // when synched to the PPS pin, the GPS sentence will start arriving about 170 ms
  // after the PPS line goes high, according to the manufacturer of the MTK3339 GPS
  // chipset. So we need to start by seeing if there's been a PPS 0 -> 1 transition.
  // *******************************************************************************

  char c;

  // is there anything new to be read?

  if(GPSSerial.available()) {

    // read the character
    c = GPS.read();

    // a "$" indicates the start of a new sentence.
    if (c == '$') {

      //reset the array index indicating where we put the characters as we build the GPS sentence.
      GPS_command_string_index = 0;
      t_new_sentence = millis();
      sentence_has_a_Z = false;

    }else{
  
    GPS_command_string_index++;

   }

    // build up the data sentence, one character at a time.
    GPS_sentence[GPS_command_string_index] = c;

    // are we reading a sentence from the GPS that carries date/time information? The
    // format is this: 
    //    $GPZDA,hhmmss.sss,dd,mm,yyyy,xx,xx*CS 
    // where CS is a checksum. Identify this kind of sentence by the presence of a Z.

    if (c == 'Z') {
      sentence_has_a_Z = true;
    }
    
    // a "*" indicates the end of the sentence, except for the two-digit checksum and the CR/LF.
    if (c == '*') {
      t_end_of_sentence = millis();
      t_GPS_read = t_end_of_sentence;
      // Serial.print("Beginning, end of reception of latest GPS sentence: "); Serial.print(t_new_sentence);
      // Serial.print(", "); Serial.println(t_end_of_sentence);

      // convert GPS data sentence from a character array to a string.
      GPS_sentence_string = String(GPS_sentence);

      // print the GPS sentence
      Serial.print("New GPS_sentence_string is "); 
      Serial.println(GPS_sentence_string.substring(0, GPS_command_string_index+1));

      // now parse the string if it corresponds to a date/time message.
      if (sentence_has_a_Z) {
        
        GPS_hour_string = GPS_sentence_string.substring(GPZDA_hour_index1, GPZDA_hour_index2);
        GPS_minutes_string = GPS_sentence_string.substring(GPZDA_minutes_index1, GPZDA_minutes_index2);
        GPS_seconds_string = GPS_sentence_string.substring(GPZDA_seconds_index1, GPZDA_seconds_index2);
        GPS_milliseconds_string = GPS_sentence_string.substring(GPZDA_milliseconds_index1, GPZDA_milliseconds_index2);
        GPS_day_string = GPS_sentence_string.substring(GPZDA_day_index1, GPZDA_day_index2);
        GPS_month_string = GPS_sentence_string.substring(GPZDA_month_index1, GPZDA_month_index2);
        GPS_year_string = GPS_sentence_string.substring(GPZDA_year_index1, GPZDA_year_index2);
  
        Serial.print("GPS time (UTC) in this sentence is " + GPS_hour_string + ":" + GPS_minutes_string + ":" + 
        GPS_seconds_string + "." + GPS_milliseconds_string);
        Serial.println("      dd/mm/yyyy = " + GPS_day_string + "/" + GPS_month_string + "/" + GPS_year_string);
  
        // now convert to integers
        GPS_hour = GPS_hour_string.toInt();
        GPS_minutes = GPS_minutes_string.toInt();
        GPS_seconds = GPS_seconds_string.toInt();
        GPS_milliseconds = GPS_milliseconds_string.toInt();
        GPS_day = GPS_day_string.toInt();
        GPS_month = GPS_month_string.toInt();
        GPS_year = GPS_year_string.toInt();
  
        // now set the RTC variables.
        RTC_hour = GPS_hour;
        RTC_minutes = GPS_minutes;
        RTC_seconds = GPS_seconds;
        RTC_day = GPS_day;
        RTC_month = GPS_month;
        RTC_year = GPS_year;
  
        // now try bumping everything by 1 second.
        bump_by_1_sec();
  
        t_bump_go = millis();
  
        // set a flag saying that we have a good proposed time to load into the RTC. We
        // will load this the next time we see a PPS 0 -> 1 transition.
        good_RTC_time_from_GPS_and_satellites = true;
        
      }
    }
  }  
}
  
/////////////////////////////////////////////////////////////////////////

void bump_by_1_sec(){

  // bump the RTC clock time by 1 second relative to the GPS value reported 
  // a few hundred milliseconds ago. I am using global variables for the ease
  // of doing this. Note that we're going to need to handle roll-overs from 59 
  // seconds to 0, and so forth.

    bool bump_flag;
    int place_holder;

    bool debug_echo = false;

    RTC_seconds_bumped = RTC_seconds + 1;

    // use "place_holder" this way so the timings through the two branches of the if blocks 
    // are the same
    place_holder = RTC_seconds + 1;
    
    if(int(RTC_seconds_bumped) >= 60) {
      bump_flag = true;
      RTC_seconds_bumped = 0;
      }else{
      bump_flag = false;
      RTC_seconds_bumped = place_holder;
      }
      
    place_holder = RTC_minutes + 1;
    
    // do we also need to bump the minutes?  
    if (bump_flag) {
      RTC_minutes_bumped = place_holder;
      }else{
      RTC_minutes_bumped = RTC_minutes;
      }

    // again, do this to equalize the time through the two branches of the if block
    place_holder = RTC_minutes_bumped;
    
    if(int(RTC_minutes_bumped) >= 60) {
      bump_flag = true;
      RTC_minutes_bumped = 0;
      }else{
      bump_flag = false;
      RTC_minutes_bumped = place_holder;
      }

    place_holder = RTC_hour + 1;
    
    // do we also need to bump the hours?  
    if (bump_flag) {
      RTC_hour_bumped = place_holder;
      }else{
      RTC_hour_bumped = RTC_hour;
      }

    place_holder = RTC_hour;

    if(int(RTC_hour_bumped) >= 24) {
      bump_flag = true;
      RTC_hour_bumped = 0;
      }else{
      bump_flag = false;
      RTC_hour_bumped = place_holder;
      }

    place_holder = RTC_day + 1;
    
    // do we also need to bump the days?  
    if (bump_flag) {
      RTC_day_bumped = place_holder;
      }else{
      RTC_day_bumped = RTC_day;
      }

    // do we need to bump the month too? Note the stuff I do to make both paths
    // through the if blocks take the same amount of execution time.
    
    int nobody_home;
    int days_in_month = 31;

    // 30 days hath September, April, June, and November...
    if (int(RTC_month) == 9 || int(RTC_month) == 4 || int(RTC_month) == 6 || int(RTC_month) == 11) {
      days_in_month = 30;
    }else{
      nobody_home = 99;
    }
      
    // ...all the rest have 31, except February...
    if (int(RTC_month) == 2 && (int(RTC_year) % 4)) {
      days_in_month = 28;
    }else{
      nobody_home = 99;
    }
    
    // ...leap year!
    if (int(RTC_month) == 2 && !(int(RTC_year) % 4)) {
      days_in_month = 29;
    }else{
      nobody_home = 99;
    }

    place_holder = RTC_day_bumped;
    
    if(int(RTC_day_bumped) > days_in_month) {
      bump_flag = true;
      RTC_day_bumped = 1;
      }else{
      bump_flag = false;
      RTC_day_bumped = place_holder;
      }

    if (bump_flag) {
      RTC_month_bumped = RTC_month + 1;
      }else{
      RTC_month_bumped = RTC_month;
      }

    place_holder = RTC_month_bumped;
              
    //... and also bump the year?
    
    if(int(RTC_month_bumped) > 12) {
      bump_flag = true;
      RTC_month_bumped = 1;
      }else{
      bump_flag = false;
      RTC_month_bumped = place_holder;
      }

    if (bump_flag) {
      RTC_year_bumped = RTC_year + 1;
      }else{
      RTC_year_bumped = RTC_year;
      }

    // keep track of when we have the proposed RTC time value ready for loading
    time_ms_bumped_RTC_time_ready = millis();

    if (debug_echo) {
      // now print the newly bumped time:
      Serial.print("Now have a proposed (1 second bumped) time ready at (ms) ");
      Serial.println(time_ms_bumped_RTC_time_ready, DEC);       
      Serial.print("Proposed (1 second bumped) time: ");
      Serial.print(RTC_hour_bumped, DEC); Serial.print(':');
      if(RTC_minutes_bumped < 10) Serial.print("0");
      Serial.print(RTC_minutes_bumped, DEC); Serial.print(':');
      if(RTC_seconds_bumped < 10) Serial.print("0");
      Serial.print(RTC_seconds_bumped, DEC); 
      Serial.print("   Date (dd/mm/yyyy): ");
      Serial.print(RTC_day_bumped, DEC); Serial.print('/');
      if(RTC_month_bumped < 10) Serial.print("0");
      Serial.print(RTC_month_bumped, DEC); Serial.print("/");
      Serial.println(RTC_year_bumped, DEC);
    }
  
}    
 
///////////////////////////////////////////////////////////////////////////////
///////////////////////// ReadADC routine ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

void ReadADC() {


  // check that the ADC isn't already busy. If so, wait for it to finish. Bit 6 (ADSC)
  // in ADCSRA will be zero when ADC isn't busy.
  while(ADCSRA & 0x40) {};
  
  // read the ADC looking at the microphone pin.
  adc_value = analogRead(microphonePin);

  // Now set a flag indicating we've read new ADC data.
  new_ADC_data = true;

  }
