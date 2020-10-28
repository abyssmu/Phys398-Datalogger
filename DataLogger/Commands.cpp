#include "Includes.h"

File audioT;
String currAudioNum = "";
String timeData = "";
String audioData = "";
String driftData = "";
int microStart = 0;
int microEnd = 0;
int dT = 0;

long t_GPS_PPS_Last = 0;

bool ppsDetected = false;

String collectBME()
{
  // time difference between data collection
  int mils = 1000 / DATAPERSECOND;

  String data = "";

  for (int i = 0; i < LOOPTIMES; ++i)
  {
    data += bme.collectBME();
    delay(mils);
  }

  return data;
}

String collectGPS()
{
  gpsQuery();

  String data = "";

  data += String(GPS_latitude_1) + "." + String(GPS_latitude_1) + "N, ";
  data += String(GPS_longitude_1) + "." + String(GPS_longitude_2) + "W";

  return data;
}

String collectTime()
{
  timeData = "";
  
  microStart = micros();
  
  timeData += "ddmmyy\n";
  timeData += String(GPS_date_string) + '\n';

  timeData += "hh,mm,ss,micros\n";

  if (GPS_hour < 10) timeData += "0";
  timeData += String(GPS_hour) + ",";

  if (GPS_minutes < 10) timeData += "0";
  timeData += String(GPS_minutes) + ",";

  if (GPS_seconds < 10) timeData += "0";
  timeData += String(GPS_seconds) + ",";

  microEnd = micros();
  dT = microEnd - microStart;

  timeData += String(dT);
  
  return timeData;
}

void CmdCenter::collectData()
{
  printLCD("Collect BME");
  writeSD(BMEFILE, collectBME());

  printLCD("Collect GPS");
  writeSD(GPSFILE, collectGPS());

  driftData += "Before audio\n";

  printLCD("Wait for PPS");
  while (!ppsDetected) { gpsLoop(); }
  audioData += String(micros()) + '\n';
  writeSD(METAFILE, collectTime());
  ppsDetected = false;

  printLCD("Record Audio");
  logAudio();
  writeSD(AUDIOFILE, audioData);

  driftData += "After audio\n";
  for (int i = 0; i < 5; ++i)
  {
    while(!ppsDetected) { gpsLoop(); }
    ppsDetected = false;
  }
  writeSD(DRIFTFILE, driftData);

  audioData = "";
  driftData = "";
}

bool CmdCenter::init()
{
  lcd.begin(16, 2);
  lcd.print(DEFAULTTXT);

  if (!bme.init())
  {
    lcd.clear();
    lcd.print("BME error.");
    error(BMEerror);
  }
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50)))
  {
    lcd.clear();
    lcd.print("SD error.");
  }

  audioSetup();
  gpsSetup();
}

int CmdCenter::checkCmd()
{
  if (cmd == READBME) return 1;
  if (cmd == WRITEBME) return 2;
  if (cmd == READGPS) return 4;
  if (cmd == WRITEGPS) return 5;

  if (cmd == RECORD) return 7;
  if (cmd == COLLECT) return 8;

  if (cmd == PRINTBME) return 51;
  if (cmd == PRINTGPS) return 52;

  if (cmd == BOOTSD) return 97;
  if (cmd == LIST) return 98;
  int clearSize = sizeof(CLEAR) - 1;
  if (cmd.substring(cmd.length() - clearSize) == CLEAR) return 99;

  return 0;
}

void CmdCenter::getInput()
{
  char keyResult = customKeypad.getKey();

  if (keyResult)
  {
    cmd += keyResult;
    printLCD(cmd);
    delay(100);
    if (runCmd())
    {
      printLCD(cmd);
      cmd = "";
    }
  }

  gpsLoop();
}

bool CmdCenter::runCmd()
{
  switch (checkCmd())
  {
    case 1:
      findNextFile(BMEFILE);
      printSD(BMEFILE);
      cmd = "Read BME";
      return true;

    case 2:
      collectBME();
      cmd = "Write BME";
      return true;

    case 4:
      printSD(GPSFILE);
      cmd = "Read GPS";
      return true;

    case 5:
      collectGPS();
      cmd = "Write GPS";
      return true;

    case 7:
      if (displayTime) displayTime = false;
      else displayTime = true;
      cmd = "Display Time";
      return true;

    case 8:
      collectData();
      cmd = "Finished";
      return true;

    case 51:
      for (int i = 0; i < LOOPTIMES; ++i)
      {
        bme.printBME();
        delay(1000);
      }
      cmd = "Print BME";
      return true;

    case 52:
      for (int i = 0; i < 10000; ++i)
      {
        gpsLoop();
      }
      collectGPS();
      cmd = "Print GPS";
      return true;

    case 97:
      if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50)))
      {
        cmd = "SD error";
      }
      else
      {
        cmd = "Reboot SD";
      }
      return true;

    case 98:
      cmd = "List cmds";
      delayLCD(cmd, 500);
      listcmd();
      return true;

    case 99:
      cmd = "Clear";
      delayLCD(cmd, 500);
      cmd = "";
      return true;
  }

  return false;
}

int findNextFile(String filename)
{
  int largest = 0;
  SdFile file;
  SdFile root;
  root.open("/");

  while (file.openNext(&root, O_RDONLY))
  {
    char files[13];
    file.getName(files, 13);
    String f = String(files);

    if (f.indexOf(filename) != -1)
    {
      int txtLen = 4;
      int numLen = 3;
      int fileNum = f.substring(f.length() - txtLen - numLen, f.length() - txtLen).toInt();

      if (fileNum > largest) largest = fileNum;
    }

    file.close();
  }

  root.close();

  return largest + 1;
}

void printSD(String filename)
{
  filename += "-000.txt";

  File dataFile = sd.open(filename);

  if (dataFile)
  {
    while (dataFile.available()) Serial.write(dataFile.read());
    dataFile.close();
  }
}

void writeSD(String filename, String data)
{
  int num = findNextFile(filename);

  filename += "-";

  if (num < 10) filename += "00" + String(num);
  else if (num < 100) filename += "0" + String(num);
  else filename += String(num);

  File dataFile = sd.open(filename + ".txt", FILE_WRITE);

  if (dataFile) dataFile.println(data);
  else Serial.println("error opening file");
  dataFile.close();
}

void audioSetup(void)
{
  // if we will blink the LED when there are errors, tell the Arduino that
  // the pin is to be an output.
  if (ERROR_LED_PIN >= 0) pinMode(ERROR_LED_PIN, OUTPUT);

  // Read the first sample pin to force the ADC to initialize itself.
  analogRead(PIN_LIST[0]);
}

ISR(ADC_vect)
{
  // This is an interrupt service routine.
  // Read ADC data.
#if RECORD_EIGHT_BITS
  uint8_t ADC_data = ADCH;
#else
  // This will access ADCL first.
  uint16_t ADC_data = ADC;
#endif

  if (isrBufNeeded && emptyHead == emptyTail)
  {
    // no buffers - count overrun
    if (isrOver < 0XFFFF) isrOver++;

    // Avoid missed timer error.
    timerFlag = false;
    return;
  }

  // Start the ADC
  if (PIN_COUNT > 1)
  {
    ADMUX = adcmux[adcindex];
    ADCSRB = adcsrb[adcindex];
    ADCSRA = adcsra[adcindex];
    
    if (adcindex == 0) timerFlag = false;

    adcindex = adcindex < (PIN_COUNT - 1) ? adcindex + 1 : 0;
  }
  else timerFlag = false;

  // Check for buffer needed.
  if (isrBufNeeded)
  {
    // Remove buffer from empty queue.
    isrBuf = emptyQueue[emptyTail];
    emptyTail = queueNext(emptyTail);
    isrBuf->count = 0;
    isrBuf->overrun = isrOver;
    isrBufNeeded = false;
  }

  // Store ADC data.
  isrBuf->data[isrBuf->count++] = ADC_data;

  // Check for buffer full.
  if (isrBuf->count >= PIN_COUNT * SAMPLES_PER_BLOCK)
  {
    // Put buffer isrIn full queue.
    uint8_t tmp = fullHead; // Avoid extra fetch of volatile fullHead.
    fullQueue[tmp] = (block_t*)isrBuf;
    fullHead = queueNext(tmp);

    // Set buffer needed and clear overruns.
    isrBufNeeded = true;
    isrOver = 0;
  }
}

ISR(TIMER1_COMPB_vect)
{
  // Make sure ADC ISR responded to timer event.
  if (timerFlag)
  {
    timerError = true;
  }
  timerFlag = true;
}

#define error(msg) { sd.errorPrint(F(msg)); fatalBlink(); }

void adcInit(metadata_t* meta)
{
  uint8_t adps; // prescaler bits for ADCSRA
  uint32_t ticks = F_CPU * SAMPLE_INTERVAL + 0.5; // Sample interval cpu cycles.

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) error("Invalid ADC reference");

#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) error("Invalid ADC prescaler");

  adps = ADC_PRESCALER;
#else // ADC_PRESCALER
  // Allow extra cpu cycles to change ADC settings if more than one pin.
  int32_t adcCycles = (ticks - ISR_TIMER0) / PIN_COUNT - ISR_SETUP_ADC;

  for (adps = 7; adps > 0; adps--)
  {
    if (adcCycles >= (MIN_ADC_CYCLES << adps)) break;
  }
#endif // ADC_PRESCALER

  meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000))
  {
    error("Sample Rate Too High");
  }

#if ROUND_SAMPLE_INTERVAL
  // Round so interval is multiple of ADC clock.
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif // ROUND_SAMPLE_INTERVAL

  if (PIN_COUNT > sizeof(meta->pinNumber) / sizeof(meta->pinNumber[0])) error("Too many pins");

  meta->pinCount = PIN_COUNT;
  meta->recordEightBits = RECORD_EIGHT_BITS;

  for (int i = 0; i < PIN_COUNT; i++)
  {
    uint8_t pin = PIN_LIST[i];
    if (pin >= NUM_ANALOG_INPUTS) error("Invalid Analog pin number");

    meta->pinNumber[i] = pin;

    // Set ADC reference and low three bits of analog pin number.
    adcmux[i] = (pin & 7) | ADC_REF;
    if (RECORD_EIGHT_BITS) adcmux[i] |= 1 << ADLAR;

    // If this is the first pin, trigger on timer/counter 1 compare match B.
    adcsrb[i] = i == 0 ? (1 << ADTS2) | (1 << ADTS0) : 0;
#ifdef MUX5
    if (pin > 7) adcsrb[i] |= (1 << MUX5);
#endif // MUX5
    adcsra[i] = (1 << ADEN) | (1 << ADIE) | adps;
    adcsra[i] |= i == 0 ? 1 << ADATE : 1 << ADSC;
  }

  // Setup timer1
  TCCR1A = 0;
  uint8_t tshift;

  if (ticks < 0X10000)
  {
    // no prescale, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    tshift = 0;
  }
  else if (ticks < 0X10000 * 8)
  {
    // prescale 8, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    tshift = 3;
  }
  else if (ticks < 0X10000 * 64)
  {
    // prescale 64, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    tshift = 6;
  }
  else if (ticks < 0X10000 * 256)
  {
    // prescale 256, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
    tshift = 8;
  }
  else if (ticks < 0X10000 * 1024)
  {
    // prescale 1024, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
    tshift = 10;
  }
  else error("Sample Rate Too Slow");

  // divide by prescaler
  ticks >>= tshift;
  // set TOP for timer reset
  ICR1 = ticks - 1;
  // compare for ADC start
  OCR1B = 0;

  // multiply by prescaler
  ticks <<= tshift;

  // Sample interval in CPU clock ticks.
  meta->sampleInterval = ticks;
  meta->cpuFrequency = F_CPU;
}

// enable ADC and timer1 interrupts
void adcStart()
{
  // initialize ISR
  isrBufNeeded = true;
  isrOver = 0;
  adcindex = 1;

  // Clear any pending interrupt.
  ADCSRA |= 1 << ADIF;

  // Setup for first pin.
  ADMUX = adcmux[0];
  ADCSRB = adcsrb[0];
  ADCSRA = adcsra[0];

  // Enable timer1 interrupts.
  timerError = false;
  timerFlag = false;
  TCNT1 = 0;
  TIFR1 = 1 << OCF1B;
  TIMSK1 = 1 << OCIE1B;
}

void adcStop()
{
  TIMSK1 = 0;
  ADCSRA = 0;
}

// read data file and check for overruns
void checkOverrun()
{
  bool headerPrinted = false;
  block_t buf;
  uint32_t bgnBlock, endBlock;
  uint32_t bn = 0;

  if (!binFile.isOpen())
  {
    lcd.print(F("No binary file"));
    return;
  }
  
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) error("contiguousRange failed");

  binFile.rewind();
  if (!binFile.read(&buf, 512) == 512) error("Read metadata failed");

  bn++;
  
  while (binFile.read(&buf, 512) == 512)
  {
    if (buf.count == 0) break;

    if (buf.overrun)
    {
      if (!headerPrinted)
      {
        Serial.println();
        Serial.println(F("Overruns:"));
        Serial.println(F("fileBlockNumber,sdBlockNumber,overrunCount"));
        headerPrinted = true;
      }
      Serial.print(bn);
      Serial.print(',');
      Serial.print(bgnBlock + bn);
      Serial.print(',');
      Serial.println(buf.overrun);
    }
    bn++;
  }
  
  if (!headerPrinted) Serial.println(F("No errors found"));
}

// dump data file to Serial
void dumpData()
{
  block_t buf;

  // an experiment...
  binFile.close();
  binFile.open(binName);

  if (!binFile.isOpen())
  {
    lcd.print(F("No binary file"));
    return;
  }
  
  binFile.rewind();
  
  if (binFile.read(&buf, 512) != 512) error("Read metadata failed");

  while (!Serial.available() && binFile.read(&buf, 512) == 512)
  {
    if (buf.count == 0) break;

    if (buf.overrun)
    {
      Serial.print(F("OVERRUN,"));
      Serial.println(buf.overrun);
    }
    for (uint16_t i = 0; i < buf.count; i++)
    {
      Serial.print(buf.data[i], DEC);
      if ((i + 1) % PIN_COUNT) Serial.print(',');
      else Serial.println();
    }
  }
}

void logAudio()
{
  // write ADC data to the SD card.
  uint32_t bgnBlock, endBlock;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT];

  // Initialize ADC and timer1.
  adcInit((metadata_t*)&block[0]);

  // Find unused file name.
  if (BASE_NAME_SIZE > 6) error("FILE_BASE_NAME too long");

  while (sd.exists(binName))
  {
    if (binName[BASE_NAME_SIZE + 1] != '9') binName[BASE_NAME_SIZE + 1]++;
    else
    {
      binName[BASE_NAME_SIZE + 1] = '0';
      
      if (binName[BASE_NAME_SIZE] == '9') error("Can't create file name");
      
      binName[BASE_NAME_SIZE]++;
    }
  }

  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME)) if (!sd.remove(TMP_FILE_NAME)) error("Can't remove tmp file");

  // Create new audio file.
  binFile.close();
  if (!binFile.createContiguous(TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) error("createContiguous failed");

  // Get the address of the file on the sd.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) error("contiguousRange failed");

  // Use SdFat's internal buffer.
  uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
  if (cache == 0) error("cacheClear failed");

  // Flash erase all data in the file.
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock)
  {
    endErase = bgnErase + ERASE_SIZE;
    
    if (endErase > endBlock) endErase = endBlock;
    if (!sd.card()->erase(bgnErase, endErase)) error("erase failed");
    
    bgnErase = endErase + 1;
  }

  // Start a multiple block write.
  if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) error("writeBegin failed");

  // Write metadata.
  if (!sd.card()->writeData((uint8_t*)&block[0])) error("Write metadata failed");

  // Initialize queues.
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;

  // Use SdFat buffer for one block.
  emptyQueue[emptyHead] = (block_t*)cache;
  emptyHead = queueNext(emptyHead);

  // Put rest of buffers in the empty queue.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++)
  {
    emptyQueue[emptyHead] = &block[i];
    emptyHead = queueNext(emptyHead);
  }

  // Give SD time to prepare for big write.
  delay(1000);

  LCD_message(F("Recording. Hit  "), F("keypad * to stop"));

  // Wait for Serial Idle.
  Serial.flush();
  delay(10);

  uint32_t bn = 1;
  uint32_t t0 = millis();
  uint32_t t1 = t0;
  uint32_t overruns = 0;
  uint32_t count = 0;
  uint32_t maxLatency = 0;

  audioData += String(micros()) + "\n";
  
  // Start logging interrupts.
  adcStart();

  while (1)
  {    
    if (fullHead != fullTail)
    {
      // Get address of block to write.
      block_t* pBlock = fullQueue[fullTail];

      // Write block to sd.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t*)pBlock)) error("write data failed");

      usec = micros() - usec;
      t1 = millis();
      if (usec > maxLatency) maxLatency = usec;

      count += pBlock->count;

      // Add overruns and possibly light LED.
      if (pBlock->overrun)
      {
        overruns += pBlock->overrun;
        if (ERROR_LED_PIN >= 0) digitalWrite(ERROR_LED_PIN, HIGH);
      }

      // Move block to empty queue.
      emptyQueue[emptyHead] = pBlock;
      emptyHead = queueNext(emptyHead);
      fullTail = queueNext(fullTail);
      bn++;

      if (bn == FILE_BLOCK_COUNT)
      {
        // File full so stop ISR calls.
        adcStop();
        break;
      }
    }
    
    if (timerError) error("Missed timer event - rate too high");

    // intialize character read to something not on the keypad.
    char the_key = '?';

    // Does the user want us to stop? Keypad * will signal this.
    bool got_a_key = customKeypad.getKeys();

    if (got_a_key) the_key = customKeypad.key[0].kchar;

    if (the_key == '*')
    {
      // Stop ISR calls to read the ADC.
      adcStop();

      if (isrBuf != 0 && isrBuf->count >= PIN_COUNT)
      {
        // Truncate to last complete sample.
        isrBuf->count = PIN_COUNT * (isrBuf->count / PIN_COUNT);

        // Put buffer in full queue.
        fullQueue[fullHead] = isrBuf;
        fullHead = queueNext(fullHead);
        isrBuf = 0;
      }

      if (fullHead == fullTail) break;
    }
  } // end of while(1) block

  audioData += String(micros());

  if (!sd.card()->writeStop()) error("writeStop failed");

  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT) if (!binFile.truncate(512L * bn)) error("Can't truncate file");

  if (!binFile.rename(sd.vwd(), binName)) error("Can't rename file");
}

void gpsSetup()
{
  // declare the GPS PPS pin to be an Arduino input
  myPin_mask = digitalPinToBitMask(GPS_PPS_PIN);
  myPin_port = portInputRegister(digitalPinToPort(GPS_PPS_PIN));

  // initialize a flag and some counters
  good_RTC_time_from_GPS_and_satellites = false;
  consecutive_good_sets_so_far = 0;
  i_am_so_bored = 0;

  GPS.begin(9600);
  
  GPS_PPS_value_old = 0;

  GPS.sendCommand(PMTK_DATE_TIME_ONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_SYNC_PPS_NMEA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5SEC);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10SEC);
  // GPS.sendCommand(PGCMD_ANTENNA);

  // GPSSerial.println(PMTK_Q_RELEASE);

  // this keeps track of where in the string of characters of a GPS data
  // sentence we are.
  GPS_command_string_index = 0;

  // more initialization
  sentence_has_a_Z = false;
  time_to_quit = false;

  if (!rtc.begin())
  {
    lcd.print("RTC failed");
    while (1) {}
  }

  t_GPS_PPS_Last = millis();
}

void gpsLoop()
{
  gpsQuery();

  GPS_PPS_value = (*myPin_port & myPin_mask);

  // did we just get a 0 -> 1 transition?
  if (GPS_PPS_value >= 1 && GPS_PPS_value_old == 0)
  {
    ppsDetected = true;

    long newT = micros();
    Serial.print("newT: ");
    Serial.println(newT);
    
    Serial.print("Just saw a PPS 0 -> 1 transition at time (ms) = ");
    t_GPS_PPS = millis();
    Serial.println(t_GPS_PPS);

    Serial.println(newT - t_GPS_PPS_Last);
    driftData += String(newT - t_GPS_PPS_Last) + "\n";
    t_GPS_PPS_Last = newT;

    // load the previously established time values into the RTC now.
    if (good_RTC_time_from_GPS_and_satellites)
    {
      rtc.adjust(DateTime(int(RTC_year_bumped), int(RTC_month_bumped), int(RTC_day_bumped), int(RTC_hour_bumped), int(RTC_minutes_bumped), int(RTC_seconds_bumped)));

      // take note of when we're back from setting the real time clock:
      t_RTC_update = millis();

      // now read back the RTC to check.
      now = rtc.now();

      // now that we've used this GPS value, set the following flag to false:
      good_RTC_time_from_GPS_and_satellites = false;

      bool ILikeIt = int(t_RTC_update - t_GPS_PPS) >= t_RTC_update__t_GPS_PPS_min
                     && int(t_GPS_PPS - t_bump_go) >= t_GPS_PPS___t_bump_go_min
                     && int(t_bump_go - t_GPS_read) >= t_bump_go___t_GPS_read_min
                     && int(t_RTC_update - t_GPS_read) >= t_RTC_update___t_GPS_read_min
                     && int(t_RTC_update - t_GPS_PPS) <= t_RTC_update__t_GPS_PPS_max
                     && int(t_GPS_PPS - t_bump_go) <= t_GPS_PPS___t_bump_go_max
                     && int(t_bump_go - t_GPS_read) <= t_bump_go___t_GPS_read_max
                     && int(t_RTC_update - t_GPS_read) <= t_RTC_update___t_GPS_read_max;

      if (ILikeIt) consecutive_good_sets_so_far++;
      else consecutive_good_sets_so_far = 0;
    }

    if (displayTime)
    {
      if (GPS_seconds < 10)
      {
        printLCD(String(GPS_hour) + ":" + String(GPS_minutes) + ":0" + String(GPS_seconds));
      }
      else
      {
        printLCD(String(GPS_hour) + ":" + String(GPS_minutes) + ":" + String(GPS_seconds));
      }
    }
  }

  GPS_PPS_value_old = GPS_PPS_value;

  char c;

  // is there anything new to be read?
  if (GPSSerial.available())
  {
    // read the character
    c = GPS.read();

    // a "$" indicates the start of a new sentence.
    if (c == '$')
    {
      GPS_command_string_index = 0;
      t_new_sentence = millis();
      sentence_has_a_Z = false;
    }
    else GPS_command_string_index++;

    // build up the data sentence, one character at a time.
    GPS_sentence[GPS_command_string_index] = c;

    if (c == 'Z') sentence_has_a_Z = true;
    
    if (c == '*')
    {
      t_end_of_sentence = millis();
      t_GPS_read = t_end_of_sentence;

      // convert GPS data sentence from a character array to a string.
      GPS_sentence_string = String(GPS_sentence);

      // now parse the string if it corresponds to a date/time message.
      if (sentence_has_a_Z)
      {
        GPS_hour_string = GPS_sentence_string.substring(GPZDA_hour_index1, GPZDA_hour_index2);
        GPS_minutes_string = GPS_sentence_string.substring(GPZDA_minutes_index1, GPZDA_minutes_index2);
        GPS_seconds_string = GPS_sentence_string.substring(GPZDA_seconds_index1, GPZDA_seconds_index2);
        GPS_milliseconds_string = GPS_sentence_string.substring(GPZDA_milliseconds_index1, GPZDA_milliseconds_index2);
        GPS_day_string = GPS_sentence_string.substring(GPZDA_day_index1, GPZDA_day_index2);
        GPS_month_string = GPS_sentence_string.substring(GPZDA_month_index1, GPZDA_month_index2);
        GPS_year_string = GPS_sentence_string.substring(GPZDA_year_index1, GPZDA_year_index2);

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
        good_RTC_time_from_GPS_and_satellites = true;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////

void bump_by_1_sec()
{
  bool bump_flag;
  int place_holder;

  RTC_seconds_bumped = RTC_seconds + 1;
  place_holder = RTC_seconds + 1;

  if (int(RTC_seconds_bumped) >= 60)
  {
    bump_flag = true;
    RTC_seconds_bumped = 0;
  }
  else
  {
    bump_flag = false;
    RTC_seconds_bumped = place_holder;
  }

  place_holder = RTC_minutes + 1;

  // do we also need to bump the minutes?
  if (bump_flag) RTC_minutes_bumped = place_holder;
  else RTC_minutes_bumped = RTC_minutes;

  place_holder = RTC_minutes_bumped;

  if (int(RTC_minutes_bumped) >= 60)
  {
    bump_flag = true;
    RTC_minutes_bumped = 0;
  }
  else
  {
    bump_flag = false;
    RTC_minutes_bumped = place_holder;
  }

  place_holder = RTC_hour + 1;

  // do we also need to bump the hours?
  if (bump_flag) RTC_hour_bumped = place_holder;
  else RTC_hour_bumped = RTC_hour;

  place_holder = RTC_hour;

  if (int(RTC_hour_bumped) >= 24)
  {
    bump_flag = true;
    RTC_hour_bumped = 0;
  }
  else
  {
    bump_flag = false;
    RTC_hour_bumped = place_holder;
  }

  place_holder = RTC_day + 1;

  // do we also need to bump the days?
  if (bump_flag) RTC_day_bumped = place_holder;
  else RTC_day_bumped = RTC_day;

  int nobody_home;
  int days_in_month = 31;

  // 30 days hath September, April, June, and November...
  if (int(RTC_month) == 9 || int(RTC_month) == 4 || int(RTC_month) == 6 || int(RTC_month) == 11) days_in_month = 30;
  else nobody_home = 99;

  // ...all the rest have 31, except February...
  if (int(RTC_month) == 2 && (int(RTC_year) % 4)) days_in_month = 28;
  else nobody_home = 99;

  // ...leap year!
  if (int(RTC_month) == 2 && !(int(RTC_year) % 4)) days_in_month = 29;
  else nobody_home = 99;

  place_holder = RTC_day_bumped;

  if (int(RTC_day_bumped) > days_in_month)
  {
    bump_flag = true;
    RTC_day_bumped = 1;
  }
  else
  {
    bump_flag = false;
    RTC_day_bumped = place_holder;
  }

  if (bump_flag) RTC_month_bumped = RTC_month + 1;
  else RTC_month_bumped = RTC_month;

  place_holder = RTC_month_bumped;

  //... and also bump the year?

  if (int(RTC_month_bumped) > 12)
  {
    bump_flag = true;
    RTC_month_bumped = 1;
  }
  else
  {
    bump_flag = false;
    RTC_month_bumped = place_holder;
  }

  if (bump_flag) RTC_year_bumped = RTC_year + 1;
  else RTC_year_bumped = RTC_year;

  // keep track of when we have the proposed RTC time value ready for loading
  time_ms_bumped_RTC_time_ready = millis();
}

int gpsQuery()
{
  // initialize the number-of-reads counter.
  GPS_char_reads = 0;
  GPS_sentence = GPS.lastNMEA();
  
  while (true)
  {
    while (GPS_char_reads <= GPS_char_reads_maximum)
    {
      char single_GPS_char = GPS.read();

      if (single_GPS_char == '\0') return -1;

      // bump the number of times we've tried to read from the GPS.
      GPS_char_reads++;

      if (GPS.newNMEAreceived())  break;
    }

    if (GPS_char_reads >= GPS_char_reads_maximum) return -1;
    
    GPS_sentence = GPS.lastNMEA();

    // convert GPS data sentence from a character array to a string.
    GPS_sentence_string = String(GPS_sentence);

    bool data_OK = GPS_sentence_string.charAt(0) == '$';
    data_OK = data_OK && (GPS_sentence_string.indexOf('$', 2) < 0);
    data_OK = data_OK && (GPS_sentence_string.indexOf('*', 0) > 0);

    GPS_command = GPS_sentence_string.substring(0, 6);

    GPS_command.trim();

    bool command_OK = GPS_command.equals("$GPRMC") || GPS_command.equals("$GPGGA");
    if (!command_OK) return -1;

    if (data_OK && GPS_command.equals("$GPRMC"))
    {
      // parse the time
      GPS_hour_string = GPS_sentence_string.substring(GPRMC_hour_index1, GPRMC_hour_index2);
      GPS_minutes_string = GPS_sentence_string.substring(GPRMC_minutes_index1, GPRMC_minutes_index2);
      GPS_seconds_string = GPS_sentence_string.substring(GPRMC_seconds_index1, GPRMC_seconds_index2);
      GPS_milliseconds_string = GPS_sentence_string.substring(GPRMC_milliseconds_index1, GPRMC_milliseconds_index2);
      GPS_AV_code_string = GPS_sentence_string.substring(GPRMC_AV_code_index1, GPRMC_AV_code_index2);

      GPS_hour = GPS_hour_string.toInt();
      GPS_minutes = GPS_minutes_string.toInt();
      GPS_seconds = GPS_seconds_string.toInt();
      GPS_milliseconds = GPS_milliseconds_string.toInt();
      
      data_OK = GPS_AV_code_string == "A";
      int asterisk_should_be_here = GPS_sentence_string.length() - asterisk_backup;

      data_OK = data_OK && (GPS_sentence_string.charAt(asterisk_should_be_here) == '*');

      // now check that the sentence is not too short.
      data_OK = data_OK && (GPS_sentence_string.length() >= GPSMINLENGTH);

      if (!data_OK) return -1;

      // now parse latitude
      GPS_latitude_1_string = GPS_sentence_string.substring(GPRMC_latitude_1_index1, GPRMC_latitude_1_index2);
      GPS_latitude_2_string = GPS_sentence_string.substring(GPRMC_latitude_2_index1, GPRMC_latitude_2_index2);
      GPS_latitude_NS_string = GPS_sentence_string.substring(GPRMC_latitude_NS_index1, GPRMC_latitude_NS_index2);

      GPS_latitude_1 = GPS_latitude_1_string.toInt();
      GPS_latitude_2 = GPS_latitude_2_string.toInt();

      // now parse longitude
      GPS_longitude_1_string = GPS_sentence_string.substring(GPRMC_longitude_1_index1, GPRMC_longitude_1_index2);
      GPS_longitude_2_string = GPS_sentence_string.substring(GPRMC_longitude_2_index1, GPRMC_longitude_2_index2);
      GPS_longitude_EW_string = GPS_sentence_string.substring(GPRMC_longitude_EW_index1, GPRMC_longitude_EW_index2);

      GPS_longitude_1 = GPS_longitude_1_string.toInt();
      GPS_longitude_2 = GPS_longitude_2_string.toInt();

      int comma_A_index = GPRMC_longitude_EW_index2;
      int comma_B_index = GPS_sentence_string.indexOf(",", comma_A_index + 1);
      int comma_C_index = GPS_sentence_string.indexOf(",", comma_B_index + 1);

      GPS_speed_knots_string = GPS_sentence_string.substring(comma_A_index + 1, comma_B_index);
      GPS_direction_string = GPS_sentence_string.substring(comma_B_index + 1, comma_C_index);

      GPS_speed_knots = GPS_speed_knots_string.toFloat();
      GPS_direction = GPS_direction_string.toFloat();

      // now get the (UTC) date, in format DDMMYY, e.g. 080618 for 8 June 2018.
      GPS_date_string = GPS_sentence_string.substring(comma_C_index + +1, comma_C_index + 7);

      return 0;

    } // end of "if (data_OK && GPS_command.equals("$GPRMC"))" block

    if (data_OK && GPS_command.equals("$GPGGA"))
    {
      // parse the time
      GPS_hour_string = GPS_sentence_string.substring(GPGGA_hour_index1, GPGGA_hour_index2);
      GPS_minutes_string = GPS_sentence_string.substring(GPGGA_minutes_index1, GPGGA_minutes_index2);
      GPS_seconds_string = GPS_sentence_string.substring(GPGGA_seconds_index1, GPGGA_seconds_index2);
      GPS_milliseconds_string = GPS_sentence_string.substring(GPGGA_milliseconds_index1, GPGGA_milliseconds_index2);

      GPS_hour = GPS_hour_string.toInt();
      GPS_minutes = GPS_minutes_string.toInt();
      GPS_seconds = GPS_seconds_string.toInt();
      GPS_milliseconds = GPS_milliseconds_string.toInt();

      // now get the fix quality and number of satellites.
      GPS_fix_quality_string = GPS_sentence_string.substring(GPGGA_fix_quality_index1, GPGGA_fix_quality_index2);
      GPS_satellites_string = GPS_sentence_string.substring(GPGGA_satellites_index1, GPGGA_satellites_index2);

      GPS_fix_quality = GPS_fix_quality_string.toInt();
      GPS_satellites = GPS_satellites_string.toInt();

      // satellites.
      bool data_OK = (GPS_fix_quality > 0) && (GPS_satellites >= 3);

      // now look for the asterisk.
      // int asterisk_should_be_here = GPS_sentence_string.length() - 4;
      int asterisk_should_be_here = GPS_sentence_string.length() - asterisk_backup;

      data_OK = data_OK && (GPS_sentence_string.charAt(asterisk_should_be_here) == '*');

      // now check that the sentence is not too short.
      data_OK = data_OK && (GPS_sentence_string.length() >= GPSMINLENGTH);

      if (!data_OK) return -1;

      // so far so good, so keep going...
      // now parse latitude
      GPS_latitude_1_string = GPS_sentence_string.substring(GPGGA_latitude_1_index1, GPGGA_latitude_1_index2);
      GPS_latitude_2_string = GPS_sentence_string.substring(GPGGA_latitude_2_index1, GPGGA_latitude_2_index2);
      GPS_latitude_NS_string = GPS_sentence_string.substring(GPGGA_latitude_NS_index1, GPGGA_latitude_NS_index2);

      GPS_latitude_1 = GPS_latitude_1_string.toInt();
      GPS_latitude_2 = GPS_latitude_2_string.toInt();

      // now parse longitude
      GPS_longitude_1_string = GPS_sentence_string.substring(GPGGA_longitude_1_index1, GPGGA_longitude_1_index2);
      GPS_longitude_2_string = GPS_sentence_string.substring(GPGGA_longitude_2_index1, GPGGA_longitude_2_index2);
      GPS_longitude_EW_string = GPS_sentence_string.substring(GPGGA_longitude_EW_index1, GPGGA_longitude_EW_index2);

      GPS_longitude_1 = GPS_longitude_1_string.toInt();
      GPS_longitude_2 = GPS_longitude_2_string.toInt();
      
      int comma_A_index = GPS_sentence_string.indexOf(",", GPGGA_satellites_index2 + 1);
      int comma_B_index = GPS_sentence_string.indexOf(",", comma_A_index + 1);

      GPS_altitude_string = GPS_sentence_string.substring(comma_A_index + 1, comma_B_index);
      GPS_altitude = GPS_altitude_string.toFloat();

      // all done with this sentence, so return.
      return 0;

    }
  }
}
