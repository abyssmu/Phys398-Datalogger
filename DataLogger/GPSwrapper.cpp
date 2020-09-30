#include "GPSwrapper.h"

bool GPSwrapper::init()
{
  Adafruit_GPS GPS(&GPSSerial);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

  if (!rtc.begin()) return false;

  pinMode(GPS_PPS_pin, INPUT);

  good_RTC_time_from_GPS_and_satellites = false;
  consecutive_good_sets_so_far = 0;
  i_am_so_bored = 0;
  GPS_PPS_value_old = 0;
  GPS_command_string_index = 0;
  sentence_has_a_Z = false;
  time_to_quit = false;

  GPS.sendCommand(PMTK_DATE_TIME_ONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_SYNC_PPS_NMEA);

  return true;
}

String GPSwrapper::collectGPS()
{
  String data = readGPS();
  
  if (data == "@")
  {
    return "No data.\n";
  }

  return data;
}

void GPSwrapper::printGPS()
{
  String data = readGPS();
  
  if (data == "@")
  {
    Serial.print("No data.\n");
    return;
  }

  Serial.print(data);
}

String GPSwrapper::readGPS()
{
  String data = "";
  int lineEnds = 0;
  
  while(lineEnds < 2)
  {
    if (Serial.available()) {
      char c = Serial.read();
      GPSSerial.write(c);
    }
    if (GPSSerial.available()) {
      char c = GPSSerial.read();

      data += c;

      if (c == '\n')
      {
        ++lineEnds;
      }
    }
  }

  return data;
}
