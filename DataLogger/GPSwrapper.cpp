#include "GPSwrapper.h"

Adafruit_GPS GPS(&GPSSerial);

bool GPSwrapper::init()
{
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

void GPSwrapper::updateTime()
{
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

    Serial.print("RTC is now set  ");
    Serial.print("                ");

    now = rtc.now();
    
    if(now.hour() < 10)   Serial.print(0);
    Serial.print(now.hour(), DEC);
    
    Serial.print(':');
    if(now.minute() < 10)   Serial.print(0);
    Serial.print(now.minute());
    
    Serial.print(':');
    if(now.second() < 10)   Serial.print(0);
    Serial.print(now.second());

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
        bumpBy1Sec();
  
        t_bump_go = millis();
  
        // set a flag saying that we have a good proposed time to load into the RTC. We
        // will load this the next time we see a PPS 0 -> 1 transition.
        good_RTC_time_from_GPS_and_satellites = true;
        
      }
    }
  }  
}

void GPSwrapper::bumpBy1Sec()
{
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
