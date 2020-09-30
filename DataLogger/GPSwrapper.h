#include <Arduino.h>
#include <Adafruit_GPS.h>

#include "RTClib.h"

#define GPSSerial Serial2

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

#define GPSMAXLENGTH 120

class GPSwrapper
{
  private:
    int GPS_PPS_pin = 43;
    char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    DateTime now;

    //GPS Variables
    char GPS_sentence[GPSMAXLENGTH];
    int GPS_command_string_index;
    int GPS_PPS_value, GPS_PPS_value_old;
    String GPS_sentence_string;
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
    unsigned long t_GPS_read_start;
    unsigned long t_GPS_read;
    unsigned long t_GPS_PPS;
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
    const int t_RTC_update__t_GPS_PPS_min = -1;
    const int t_GPS_PPS___t_bump_go_min = 200;
    const int t_bump_go___t_GPS_read_min = -1;
    const int t_RTC_update___t_GPS_read_min = 400;
    const int t_RTC_update__t_GPS_PPS_max = 20;
    const int t_GPS_PPS___t_bump_go_max = 800;
    const int t_bump_go___t_GPS_read_max = 350;
    const int t_RTC_update___t_GPS_read_max = 1000;
    int consecutive_good_sets_so_far;
    bool time_to_quit;
    const int thats_enough = 5;
    bool sentence_has_a_Z;
    unsigned long t_new_sentence;
    unsigned long t_end_of_sentence;
    int i_am_so_bored;

    //RTC variables
    RTC_DS3231 rtc;
    unsigned long  time_ms_bumped_RTC_time_ready;
    unsigned long t_RTC_update;
    unsigned long t_bump_go;
    bool good_RTC_time_from_GPS_and_satellites;
    int RTC_hour;
    int RTC_minutes;
    int RTC_seconds;
    int RTC_day;
    int RTC_month;
    int RTC_year;
    int RTC_hour_bumped;
    int RTC_minutes_bumped;
    int RTC_seconds_bumped;
    int RTC_day_bumped;
    int RTC_month_bumped;
    int RTC_year_bumped;
    
  public:
    GPSwrapper() {}
    ~GPSwrapper() {}

    bool init();
    String collectGPS();
    void printGPS();
    String readGPS();
};
