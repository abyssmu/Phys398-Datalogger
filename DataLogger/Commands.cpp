#include "Includes.h"

void collectBME()
{
  //time difference between data collection
  int dT = 1000 / DATAPERSECOND;

  String data = "";

  data += bme.collectBME();
  delay(dT);

  writeSD(BMEFILE, data);
}

void collectGPS()
{
  String data = "";

  data += String(GPS_latitude_1) + "." + String(GPS_latitude_1) + "W, ";
  data += String(GPS_longitude_1) + "." + String(GPS_longitude_2) + "N";

  writeSD(GPSFILE, data);
}

void collectTime()
{
  String metaName = "";
  for (int i = 0; i < BASE_NAME_SIZE+2; i++) {
    metaName += binName[i];
  }
  metaName += ".txt";
  Serial.println(metaName);
  File metaFile = sd.open(metaName, FILE_WRITE);
  metaFile.print("Current time is:");
  metaFile.print(GPS_hour, DEC);
  metaFile.print(':');
  if(GPS_minutes < 10)   metaFile.print(0);
  metaFile.print(GPS_minutes, DEC);
  metaFile.print(':');
  if(GPS_seconds < 10)   metaFile.print(0);
  metaFile.print(GPS_seconds, DEC);

  metaFile.print("   Date (dd/mm/yyyy): " + String(GPS_date_string));
//  metaFile.print(GPS_day, DEC); metaFile.print('/');
//  if(int(GPS_month) < 10) metaFile.print("0");
//  metaFile.print(GPS_month, DEC); metaFile.print("/");
//  metaFile.println(GPS_year, DEC);
  metaFile.close();
}

void executeAudio()
{
  
}

bool CmdCenter::collectData()
{
  for(int i = 0; i < LOOPTIMES; ++i)
  {
    collectBME();

    //this is here to keep the time up to date
    gpsQuery();
  }
  
  collectGPS();
  collectTime();
  executeAudio();
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
  if (!initSD())
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

char CmdCenter::checkKey()
{
  return customKeypad.getKey();
}

void CmdCenter::getInput()
{
  char keyResult = checkKey();

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
  switch(checkCmd())
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
      audioLoop();
      cmd = "Record";
      return true;

    case 8:
      collectData();
      cmd = "Collect Data";
      return true;

    case 51:
      for(int i = 0; i < LOOPTIMES; ++i)
      {
        bme.printBME();
        delay(1000);
      }
      cmd = "Print BME";
      return true;

    case 52:
      for(int i = 0; i < 10000; ++i)
      {
        gpsLoop();
      }
      cmd = "Print GPS";
      return true;

    case 97:
      initSD();
      cmd = "Reboot SD";
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

//sd function definitions
bool initSD()
{
  return sd.begin(chipSelect);
}

int findNextFile(String filename)
{
  SdFile dir;
  sd.chdir("/", true);
  
  String fCaps = filename;
  fCaps.toUpperCase();
  int largest = 0;
  
  while(dir.openNext(sd.vwd(), FILE_READ))
  {
    String n = "";
    //char* n[8];
    //dir.getName(n, sizeof(n));
    //Serial.println(String(n));
    
    if (n.indexOf(fCaps) != -1)
    {
      int txtLen = 4;
      int numLen = 3;
      int fileNum = n.substring(n.length() - txtLen - numLen, n.length() - txtLen).toInt();
      
      if(fileNum > largest)
      {
        largest = fileNum;
      }
    }
  }

  dir.close();

  return largest + 1;
}

void printSD(String filename)
{
  filename += "-000.txt";
  
  File dataFile = sd.open(filename);
  
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
}

void writeSD(String filename, String data)
{
  int num = findNextFile(filename);
  Serial.println(num);
  
  filename += "-";

  if(num < 10)
  {
    filename += "00" + String(num);
  }
  else if(num < 100)
  {
    filename += "0" + String(num);
  }
  else
  {
    filename += String(num);
  }

  File dataFile = sd.open(filename + ".txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}

/*************************************************** 
  This file is sound_record.ino
  
  This program logs data from the Arduino ADC to a binary file. The
  filename will be audioXY.bin, where XY is 00, 01, ... The program 
  automatically increments the value of XY to avoid overwriting 
  existing files. You'll need to delete files from your microSD
  card if you already have files numbered all the way up to audio99.bin.
  
  I will read the ADC at 32 kHz; the file buffers hold 254 (2-byte) words,
  plus two words of header information. If we were to record 160,020 samples,
  corresponding to 630 buffers, it would require slightly more than 5 seconds.

  An 8 GB microSD card can hold about 1,250,000 seconds of data, which is
  a little more than two weeks of recording time.
  
  Data logging is done in function logData().
  
  See https://forum.arduino.cc/index.php?topic=228549.0 for more information.
  According to information there, this routine can run at a sampling rate 
  of 40kHz without dropping any data. (Wow!)
 
  Samples are logged at regular intervals. Each Sample consists of the ADC
  values for the analog pins defined in the PIN_LIST array.  The pins numbers
  may be in any order. In my circuit I only use A7.
 
  The program will communicate with the user via the serial monitor window,
  LCD, and keypad while writing a large binary audio file to a 
  microSD card.

  It will fill buffers with ADC data to be written, then write
  entire buffers to the output file.

  Since the Arduino Mega 2560 has a lot of flash memory, I will use 
  13 buffers of 512 words each.

  Each 512 byte data block in the file has a four byte header followed by up
  to 508 bytes of data. The Arduino's ADC is a 10 bit device, and we'll use all
  the bits for maximum sensitivity.

  Each block contains an integral number of samples with unused space at the
  end of the block.
 
  Data are written to the file using an SD multiple block write command.

  I realize that this is ridiculously technical, but don't let that put 
  you off. Be fearless.

  George Gollin
  University of Illinos at Urbana-Champaign
  September 2018

  Make sure you have the correct port selected: go to 
    Tools -> Port...

  Also go to Tools -> Serial Monitor and set the baud rate to 9600.

  See https://www.arduino.cc/en/Serial/Print for more information.

  Some of this code is from Bill Greiman: see 
  github.com/greiman/SdFat/blob/master/examples/AnalogBinLogger/
  AnalogBinLogger.ino

  Edit the configuration constants below to modify the sample pins, 
  sample rate, and other configuration values.
 
  If your SD card has a long write latency, it may be necessary to use
  slower sample rates.  Using a Mega Arduino helps overcome latency
  problems since 13 512 byte buffers will be used.

  The binary audio file format written by the Arduino for 10-bit ADC data follows.
  
  An unsigned long is 4 bytes, while an unsigned short is 2 bytes. The following 
  is from AnalogBinLogger.h:
  
  *** First block of file (512 bytes) ***
  
    unsigned long  adcFrequency;     // ADC clock frequency
    unsigned long  cpuFrequency;     // CPU clock frequency
    unsigned long  sampleInterval;   // Sample interval in CPU cycles.
    unsigned long  recordEightBits;  // Size of ADC values, nonzero for 8-bits.
    unsigned long  pinCount;         // Number of analog pins in a sample.
    unsigned long  pinNumber[123];   // List of pin numbers in a sample.
  
  *** Subsequent blocks (also 512 bytes each) ***
  
    unsigned short count;      // count of data values
    unsigned short overrun;    // count of overruns since last block
    unsigned short data[254];  // ADC data, low byte comes before by high byte
                               // for example, 511 (= 255 + 256) will have
                               // data[0] = 255 (= 0xFF) and data[1] = 1 (= 0x01)
                               // This is "little endian" format.
    
*****************************************************/

//////////////////////////////////////////////////////////////////////
///////////////////////////    function ///////////////////////////
//////////////////////////////////////////////////////////////////////

void audioSetup(void)
{
  // if we will blink the LED when there are errors, tell the Arduino that
  // the pin is to be an output.
  if (ERROR_LED_PIN >= 0) {pinMode(ERROR_LED_PIN, OUTPUT);}

  // Read the first sample pin to force the ADC to initialize itself.
  analogRead(PIN_LIST[0]);

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    sd.initErrorPrint();
  }
}

//////////////////////////////////////////////////////////////////////
/////////////////////////// loop function ////////////////////////////
//////////////////////////////////////////////////////////////////////

void audioLoop(void)
{
  // tell the user what's happening.
  Serial.println(F("About to record microphone data. Hit keypad * to stop."));
  //             0123456789012345       0123456789012345
  LCD_message(F("Record at 32kHz "), F("Stop: keypad *  "));

  // record ADC data and write as binary to the sd.
  logData();
  
  // status message
  Serial.println(F("Hit keyboard # to record again.\n"));
  //             0123456789012345       0123456789012345
  LCD_message(F("Stopped."), F(""));
}

//////////////////////////////////////////////////////////////////////
///////////////////////// ADC done ISR ///////////////////////////////
//////////////////////////////////////////////////////////////////////

ISR(ADC_vect) {

  // This is an interrupt service routine.
  
  // Read ADC data.
#if RECORD_EIGHT_BITS
  uint8_t ADC_data = ADCH;
#else 
  // This will access ADCL first.
  uint16_t ADC_data = ADC;
#endif

  if (isrBufNeeded && emptyHead == emptyTail) {
    // no buffers - count overrun
    if (isrOver < 0XFFFF) {
      isrOver++;
    }

    // Avoid missed timer error.
    timerFlag = false;
    return;
  }
  
  // Start the ADC
  if (PIN_COUNT > 1) {
    ADMUX = adcmux[adcindex];
    ADCSRB = adcsrb[adcindex];
    ADCSRA = adcsra[adcindex];
    if (adcindex == 0) {
      timerFlag = false;
    }
    adcindex =  adcindex < (PIN_COUNT - 1) ? adcindex + 1 : 0;
  } else {
    timerFlag = false;
  }
  
  // Check for buffer needed.
  if (isrBufNeeded) {
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
  if (isrBuf->count >= PIN_COUNT*SAMPLES_PER_BLOCK) {
    // Put buffer isrIn full queue.
    uint8_t tmp = fullHead;  // Avoid extra fetch of volatile fullHead.
    fullQueue[tmp] = (block_t*)isrBuf;
    fullHead = queueNext(tmp);

    // Set buffer needed and clear overruns.
    isrBufNeeded = true;
    isrOver = 0;
  }
}

//////////////////////////////////////////////////////////////////////
///////////////////// timer1 interrupt ISR ///////////////////////////
//////////////////////////////////////////////////////////////////////

// timer1 interrupt to clear OCF1B. See
// http://web.ics.purdue.edu/~jricha14/Timer_Stuff/TIFR.htm for information about 
// this: OCF1B is bit 3 "(low order bit is bit 0) of TIFR, the"Timer/Counter Interrupt 
// Flag Register." It is "set (one) when a compare match occurs between the 
// Timer/Counter1 and the data in OCR1B - Output Compare Register 1B.  OCF1B is 
// cleared by hardware when executing the corresponding interrupt handling vector. 
// Alternatively, OCF1B is cleared by writing a logic one to the flag.  When the 
// I-bit in SREG, and OCIE1B (Timer/Counter1 Compare match InterruptB Enable), and 
// the OCF1B are set (one), the Timer/Counter1B Compare match Interrupt is executed.

ISR(TIMER1_COMPB_vect) {
  // Make sure ADC ISR responded to timer event.
  if (timerFlag) {
    timerError = true;
  }
  timerFlag = true;
}

//////////////////////////////////////////////////////////////////////
//////////////////// define error message function ///////////////////
//////////////////////////////////////////////////////////////////////

// Text for error messages is stored in flash (program) memory, not SRAM.
// That's what the F() does: makes sure the compiler puts msg into flash
// instead of SRAM. The Arduino Mega 2560 has 256k of flash, but only 8k of SRAM.

#define error(msg) {sd.errorPrint(F(msg));fatalBlink();}

//////////////////////////////////////////////////////////////////////
///////////////////// adcInit function ///////////////////////////////
//////////////////////////////////////////////////////////////////////

// initialize ADC and timer1
void adcInit(metadata_t* meta) {
  uint8_t adps;  // prescaler bits for ADCSRA
  uint32_t ticks = F_CPU*SAMPLE_INTERVAL + 0.5;  // Sample interval cpu cycles.

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) {
    error("Invalid ADC reference");
  }
  
#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) {
    error("Invalid ADC prescaler");
  }
  adps = ADC_PRESCALER;
#else  // ADC_PRESCALER
  // Allow extra cpu cycles to change ADC settings if more than one pin.
  int32_t adcCycles = (ticks - ISR_TIMER0)/PIN_COUNT - ISR_SETUP_ADC;

  for (adps = 7; adps > 0; adps--) {
    if (adcCycles >= (MIN_ADC_CYCLES << adps)) {
      break;
    }
  }
#endif  // ADC_PRESCALER

  meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000)) {
    error("Sample Rate Too High");
  }
  
#if ROUND_SAMPLE_INTERVAL
  // Round so interval is multiple of ADC clock.
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif  // ROUND_SAMPLE_INTERVAL

  if (PIN_COUNT > sizeof(meta->pinNumber)/sizeof(meta->pinNumber[0])) {
    error("Too many pins");
  }
  
  meta->pinCount = PIN_COUNT;
  meta->recordEightBits = RECORD_EIGHT_BITS;

  for (int i = 0; i < PIN_COUNT; i++) {
    uint8_t pin = PIN_LIST[i];
    if (pin >= NUM_ANALOG_INPUTS) {
      error("Invalid Analog pin number");
    }
    
    meta->pinNumber[i] = pin;

    // Set ADC reference and low three bits of analog pin number.
    adcmux[i] = (pin & 7) | ADC_REF;
    if (RECORD_EIGHT_BITS) {
      adcmux[i] |= 1 << ADLAR;
    }

    // If this is the first pin, trigger on timer/counter 1 compare match B.
    adcsrb[i] = i == 0 ? (1 << ADTS2) | (1 << ADTS0) : 0;
#ifdef MUX5
    if (pin > 7) {
      adcsrb[i] |= (1 << MUX5);
    }
#endif  // MUX5
    adcsra[i] = (1 << ADEN) | (1 << ADIE) | adps;
    adcsra[i] |= i == 0 ? 1 << ADATE : 1 << ADSC;
  }

  // Setup timer1
  TCCR1A = 0;
  uint8_t tshift;
  
  if (ticks < 0X10000) {
    // no prescale, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    tshift = 0;
  } else if (ticks < 0X10000*8) {
    // prescale 8, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    tshift = 3;
  } else if (ticks < 0X10000*64) {
    // prescale 64, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    tshift = 6;
  } else if (ticks < 0X10000*256) {
    // prescale 256, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
    tshift = 8;
  } else if (ticks < 0X10000*1024) {
    // prescale 1024, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
    tshift = 10;
  } else {
    error("Sample Rate Too Slow");
  }
  
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
  
  float sampleRate = (float)meta->cpuFrequency/meta->sampleInterval;
  Serial.print(F("Arduino ADC pin numbers to sample:"));
  for (uint8_t i = 0; i < meta->pinCount; i++) {
    Serial.print(' ');
    Serial.print(meta->pinNumber[i], DEC);
  }
  
  Serial.println();
  Serial.print(F("ADC bits: "));
  Serial.print(meta->recordEightBits ? 8 : 10);
  
  Serial.print(F("   ADC clock kHz: "));
  Serial.print(meta->adcFrequency/1000);
  
  Serial.print(F("   Sample Rate: "));
  Serial.print(sampleRate);
  
  Serial.print(F("   Sample interval usec: "));
  Serial.println(1000000.0/sampleRate, 4);
}

//////////////////////////////////////////////////////////////////////
//////////////////////// adcStart function ///////////////////////////
//////////////////////////////////////////////////////////////////////

// enable ADC and timer1 interrupts
void adcStart() {
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

//////////////////////////////////////////////////////////////////////
////////////////////// adcStop function //////////////////////////////
//////////////////////////////////////////////////////////////////////

void adcStop() {
  TIMSK1 = 0;
  ADCSRA = 0;
}

//////////////////////////////////////////////////////////////////////
//////////////////////// checkOverrun function ///////////////////////
//////////////////////////////////////////////////////////////////////

// read data file and check for overruns
void checkOverrun() {
  bool headerPrinted = false;
  block_t buf;
  uint32_t bgnBlock, endBlock;
  uint32_t bn = 0;

  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return;
  }
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  binFile.rewind();
  Serial.println();
  Serial.println(F("Checking overrun errors - type any character to stop"));
  if (!binFile.read(&buf , 512) == 512) {
    error("Read metadata failed");
  }
  bn++;
  while (binFile.read(&buf, 512) == 512) {
    if (buf.count == 0) {
      break;
    }
    if (buf.overrun) {
      if (!headerPrinted) {
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
  if (!headerPrinted) {
    Serial.println(F("No errors found"));
  }
}

//////////////////////////////////////////////////////////////////////
/////////////////////////// dumpData function ////////////////////////
//////////////////////////////////////////////////////////////////////

// dump data file to Serial
void dumpData() {
  block_t buf;

  // an experiment...
  Serial.print("into dumpData. Close file, then open it. Filename: ");
  Serial.println(binName);
  binFile.close();
  binFile.open(binName);

  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  if (binFile.read(&buf , 512) != 512) {
    error("Read metadata failed");
  }
  Serial.println();
  Serial.println(F("Type any character to stop"));
  delay(1000);
  while (!Serial.available() && binFile.read(&buf , 512) == 512) {
    if (buf.count == 0) {
      break;
    }
    if (buf.overrun) {
      Serial.print(F("OVERRUN,"));
      Serial.println(buf.overrun);
    }
    for (uint16_t i = 0; i < buf.count; i++) {
      Serial.print(buf.data[i], DEC);
      if ((i+1)%PIN_COUNT) {
        Serial.print(',');
      } else {
        Serial.println();
      }
    }
  }
  Serial.println(F("Done"));
}

//////////////////////////////////////////////////////////////////////
/////////////////////////// logData function /////////////////////////
//////////////////////////////////////////////////////////////////////

void logData() {

  // write ADC data to the SD card.
  
  // Print a message to the LCD.
  //           0123456789012345    0123456789012345
  LCD_message(F("Begin recording "), F("audio signal A7 "));

  uint32_t bgnBlock, endBlock;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT];

  // Initialize ADC and timer1.
  adcInit((metadata_t*) &block[0]);

  // Find unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  
  while (sd.exists(binName)) {
    if (binName[BASE_NAME_SIZE + 1] != '9') {
      binName[BASE_NAME_SIZE + 1]++;
    } else {
      binName[BASE_NAME_SIZE + 1] = '0';
      if (binName[BASE_NAME_SIZE] == '9') {
        error("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }
  
  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME)) {
    // Serial.println(F("Deleting tmp file"));
    if (!sd.remove(TMP_FILE_NAME)) {
      error("Can't remove tmp file");
    }
  }
  
  // Create new audio file.
  // Serial.println(F("Creating new file"));
  binFile.close();
  if (!binFile.createContiguous(TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
    error("createContiguous failed");
  }
  
  // Get the address of the file on the sd.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  
  // Use SdFat's internal buffer.
  uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
  if (cache == 0) {
    error("cacheClear failed");
  }

  // Flash erase all data in the file.
  // Serial.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock) {
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock) {
      endErase = endBlock;
    }
    if (!sd.card()->erase(bgnErase, endErase)) {
      error("erase failed");
    }
    bgnErase = endErase + 1;
  }
  
  // Start a multiple block write.
  if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) {
    error("writeBegin failed");
  }
  
  // Write metadata.
  if (!sd.card()->writeData((uint8_t*)&block[0])) {
    error("Write metadata failed");
  }
  
  // Initialize queues.
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;

  // Use SdFat buffer for one block.
  emptyQueue[emptyHead] = (block_t*)cache;
  emptyHead = queueNext(emptyHead);

  // Put rest of buffers in the empty queue.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyQueue[emptyHead] = &block[i];
    emptyHead = queueNext(emptyHead);
  }
  
  // Give SD time to prepare for big write.
  delay(1000);

  // status message
  Serial.println(F("Recording - type keypad * to stop"));
  //             0123456789012345           0123456789012345
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

  // Start logging interrupts.
  adcStart();
  
  while (1) {
    if (fullHead != fullTail) {
      // Get address of block to write.
      block_t* pBlock = fullQueue[fullTail];

      // Write block to sd.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t*)pBlock)) {
        error("write data failed");
      }
      usec = micros() - usec;
      t1 = millis();
      if (usec > maxLatency) {
        maxLatency = usec;
      }

      // pBlock->count refers to the variable "count" inside the
      // structure named pBlock. When we write 10 bit ADC values
      // each block will comprise two header words (four bytes total)
      // and 254 ADC data words (also two bytes each). So each time
      // a block is filled we'll have another 254 ADC reads to report.
      
      count += pBlock->count;

      // Add overruns and possibly light LED.
      if (pBlock->overrun) {
        overruns += pBlock->overrun;
        if (ERROR_LED_PIN >= 0) {
          digitalWrite(ERROR_LED_PIN, HIGH);
        }
      }
      
      // Move block to empty queue.
      emptyQueue[emptyHead] = pBlock;
      emptyHead = queueNext(emptyHead);
      fullTail = queueNext(fullTail);
      bn++;
      
      if (bn == FILE_BLOCK_COUNT) {
        // File full so stop ISR calls.
        adcStop();
        break;
      }
      
    }
    
    if (timerError) {
      error("Missed timer event - rate too high");
    }


    // The class function getKeys() returns true if there are any active keys. Note 
    // that "active" is true when a key is depressed, but that the state of an active 
    // key can be "PRESSED" (newly depressed), HOLD (still depressed), or RELEASED (just 
    // released, of course).

    // customKeypad.getKeys fills customKeypad.key[ ] array with up to 10 active keys, if the user pushes
    // more than one key at the same time. But I am only going to look at the 
    // first key pressed. 
    
    // intialize character read to something not on the keypad.
    char the_key = '?'; 

    // Does the user want us to stop? Keypad * will signal this.
    bool got_a_key = customKeypad.getKeys();

    if (got_a_key) {the_key = customKeypad.key[0].kchar;}
    
    if (the_key == '*') {
      
      // Stop ISR calls to read the ADC.
      adcStop();

      if (isrBuf != 0 && isrBuf->count >= PIN_COUNT) {

        // Truncate to last complete sample.
        isrBuf->count = PIN_COUNT*(isrBuf->count/PIN_COUNT);

        // Put buffer in full queue.
        fullQueue[fullHead] = isrBuf;
        fullHead = queueNext(fullHead);
        isrBuf = 0;
      }
      
      if (fullHead == fullTail) {
        break;
      }
    }

    
  }   // end of while(1) block
  
  if (!sd.card()->writeStop()) {
    error("writeStop failed");
  }
  
  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT) {
    Serial.println(F("Truncating file"));
    if (!binFile.truncate(512L * bn)) {
      error("Can't truncate file");
    }
  }
  
  if (!binFile.rename(sd.vwd(), binName)) {
    error("Can't rename file");
  }

  // Print a message to the LCD.
  //           0123456789012345    0123456789012345
  LCD_message(F("Stop recording  "), F("audio signal A7 "));

  Serial.print(F("File renamed: "));
  Serial.println(binName);
  Serial.print(F("Max block write usec: "));
  Serial.println(maxLatency);
  Serial.print(F("Record time sec: "));
  Serial.println(0.001*(t1 - t0), 3);
  Serial.print(F("Sample count: "));
  Serial.println(count/PIN_COUNT);
  Serial.print(F("Samples/sec: "));
  Serial.println((1000.0/PIN_COUNT)*count/(t1-t0));
  Serial.print(F("Overruns: "));
  Serial.println(overruns);
  Serial.println(F("Done"));
}

//////////////////////////////////////////////////////////////////////
////////////////////////////GPS info//////////////////////////////////
//////////////////////////////////////////////////////////////////////

void gpsSetup()
{
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

  // turn on RMC (recommended minimum) and GGA (fix data, including altitude)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // uncomment this line to turn on only the "minimum recommended" data:
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // Set the update rate to once per second. Faster than this might make it hard for
  // the serial communication line to keep up: you'll need to check this.
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  
  // You'll want to check that the faster read rates work reliably for you before
  // using them. The readout rates are, of course, 1, 2, 5, and 10 Hz.
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ); 
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); 
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); 
  
  // Uncomment this to set the update rate to once per 5 seconds.
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5SEC);
     
  // Uncomment this to set the update rate to once per 10 seconds.
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10SEC);
     
  // Request updates on antenna status, comment out to keep quiet.
  // GPS.sendCommand(PGCMD_ANTENNA);

  // Ask for firmware version, write this to the serial line. Comment out to keep quiet.
  // GPSSerial.println(PMTK_Q_RELEASE);
  
  // this keeps track of where in the string of characters of a GPS data sentence we are.
  GPS_command_string_index = 0;

  // more initialization
  sentence_has_a_Z = false;
  time_to_quit = false;

  // fire up the RTC.
  int return_code = rtc.begin();

  // problems?
  if(!return_code) {
    Serial.println("RTC wouldn't respond so bail out.");
    while (1) {};
  }
}

/////////////////////////////////////////////////////////////////////////

void gpsLoop () {

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
    // increment a counter
    i_am_so_bored++;

    return;
  }

  gpsQuery();
  
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

int gpsQuery()
{

  // return 0 if we found good GPS navigational data and -1 if not. we might
  // return a -1 when there is an error in the data, or when the sentence
  // doesn't carry navigational information.
  
  // The GPS device has its own microprocessor and, once we have loaded its parameters,
  // free-runs at a fixed sampling rate. We do not trigger its registration of
  // latitude and longitude, rather we just read from it the last data record
  // it has stored. And we do it one character at a time!

  // I will ask the GPS for a single character, which will be an ASCII null ('\0')
  // if there are no data to be read, and something else if there are data present.
  // If I get an ASCII null, just return immediately with a -1 return code. If I DO find
  // some data, then keep reading it until we are finished, or else hit another ASCII null.

  // When the GPS has yielded a complete, entire sentence of navigation data, we'll parse 
  // it and load that information into some global variables. 

  // initialize the number-of-reads counter. 
  GPS_char_reads = 0;

  // This gets the last sentence read from GPS and clears a newline flag in the Adafruit 
  // library code. If there aren't any data, we'll get an ASCII null.
  GPS_sentence = GPS.lastNMEA();  

  // Stay inside the following loop until we've read a complete GPS sentence with
  // good navigational data, or else the loop times out. With GPS_char_reads_maximum 
  // set to a million this'll take about 6 seconds to time out. 

  while (true) {
  
    while(GPS_char_reads <= GPS_char_reads_maximum) 
      {
  
      // try to read a single character from the GPS device. we'll get an ascii 
      // null ('\0') if there's nothing to read, either becauser we're reading 
      // to fast for the GPS to keep up, or becauser there's not data to repprt.
      char single_GPS_char = GPS.read();
  
      if(single_GPS_char == '\0')
        {
        // Serial.println("\ncharacter was a null so bail out.");
        return -1;
        }
  
      // echo the character to the screen.
      if(GPSECHO_GPS_query) Serial.print(single_GPS_char);
      
      // bump the number of times we've tried to read from the GPS.
      GPS_char_reads++;
  
      // now ask if we've received a complete data sentence. If yes, break
      // out of the "while(GPS_char_reads <= GPS_char_reads_maximum)" loop. 
      
      if(GPS.newNMEAreceived()) break;
  
      }
    
    // if we've landed here because we hit the limit on the number of character reads we've 
    // tried, print a message and bail out.
    
    if (GPS_char_reads >= GPS_char_reads_maximum) 
      {
      Serial.println("Having trouble reading GPS navigation data. Try again later.");
      return -1;        
      }

    // Land here because we have received a complete GPS sentence and executed the "break" 
    // in the above while loop. get the last complete sentence read from the GPS.
    GPS_sentence = GPS.lastNMEA();
    
    // convert GPS data sentence from a character array to a string.
    GPS_sentence_string = String(GPS_sentence);
  
    if (GPSECHO_GPS_query) 
      {
      Serial.println("\n******************\njust received a complete sentence, so parse stuff. Sentence is");
      Serial.println(GPS_sentence_string);
      }
  
    // now do a cursory check that the sentence we've just read is OK. Check that there is only
    // one $, as the first character in the sentence, and that there's an asterisk (which comes 
    // immediately before the checksum).
     
    // sentence starts with a $? 
    bool data_OK = GPS_sentence_string.charAt(0) == '$';    
  
    // sentence contains no other $? The indexOf call will return -1 if $ is not found.
    data_OK = data_OK && (GPS_sentence_string.indexOf('$', 2) <  0);
    
    // now find that asterisk...
    data_OK = data_OK && (GPS_sentence_string.indexOf('*', 0) >  0);
  
    // now parse the GPS sentence. I am only interested in sentences that begin with
    // $GPGGA ("GPS fix data") or $GPRMC ("recommended minimum specific GPS/Transit data").
  
    if(GPSECHO_GPS_query)
      {
      Serial.print("length of GPS_sentence_string just received...");
      Serial.println(GPS_sentence_string.length());
      }
  
    // now get substring holding the GPS command. Only proceed if it is $GPRMC or $GPGGA.
    GPS_command = GPS_sentence_string.substring(0, 6);
  
    // also trim it to make sure we don't have hidden stuff or white space sneaking in.
    GPS_command.trim();
  
    if(GPSECHO_GPS_query) 
      {
      Serial.print("GPS command is "); Serial.println(GPS_command);
      }   
  
    // if data_OK is true then we have a good sentence. but we also need the sentence
    // to hold navigational data we can use. we can only work with GPRMC and GPGGA sentences. 
  
    bool command_OK = GPS_command.equals("$GPRMC") || GPS_command.equals("$GPGGA"); 
    
    // if we have a sentence that, upon cursory inspection, is well formatted AND might
    // hold navigational data, continue to parse the sentence. If the GPS device
    // hasn't found any satellites yet, we'll want to bail out.
  
     if (!command_OK) 
       {

       if(GPSECHO_GPS_query) 
         {Serial.println("GPS sentence isn't a navigational information sentence.");        
       }

      return -1;        
      }
      
    //////////////////////////////////////////////////////////////////////
    /////////////////////////// GPRMC sentence ///////////////////////////
    //////////////////////////////////////////////////////////////////////
    
     if (data_OK && GPS_command.equals("$GPRMC"))
        {
            
        if(GPSECHO_GPS_query) 
          {
          Serial.print("\nnew GPS sentence: "); Serial.println(GPS_sentence_string);
          }
    
        // parse the time
        GPS_hour_string = GPS_sentence_string.substring(GPRMC_hour_index1, GPRMC_hour_index2);
        GPS_minutes_string = GPS_sentence_string.substring(GPRMC_minutes_index1, GPRMC_minutes_index2);
        GPS_seconds_string = GPS_sentence_string.substring(GPRMC_seconds_index1, GPRMC_seconds_index2);
        GPS_milliseconds_string = GPS_sentence_string.substring(GPRMC_milliseconds_index1, 
        GPRMC_milliseconds_index2);
        GPS_AV_code_string = GPS_sentence_string.substring(GPRMC_AV_code_index1, GPRMC_AV_code_index2);
    
        GPS_hour = GPS_hour_string.toInt();
        GPS_minutes = GPS_minutes_string.toInt();
        GPS_seconds = GPS_seconds_string.toInt();
        GPS_milliseconds = GPS_milliseconds_string.toInt();
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
          Serial.print("A/V code is "); Serial.println(GPS_AV_code_string);
          }
    
        // now see if the data are valid: we'll expect an "A" as the AV code string.
        // We also expect an asterisk two characters from the end. Also check that the sentence 
        // is at least as long as the minimum length expected.
    
        data_OK = GPS_AV_code_string == "A";
    
        // now look for the asterisk after trimming any trailing whitespace in the GPS sentence.
        // the asterisk preceeds the sentence's checksum information, which I won't bother to check.
        // int asterisk_should_be_here = GPS_sentence_string.length() - 4; 
        int asterisk_should_be_here = GPS_sentence_string.length() - asterisk_backup; 
    
        data_OK = data_OK && (GPS_sentence_string.charAt(asterisk_should_be_here) == '*');

        if(GPSECHO_GPS_query)
          {
          Serial.print("expected asterisk position "); Serial.print(asterisk_should_be_here); 
          Serial.print(" at that position: "); Serial.println(GPS_sentence_string.charAt(asterisk_should_be_here));
          }
    
        // now check that the sentence is not too short.      
        data_OK = data_OK && (GPS_sentence_string.length() >= GPSMINLENGTH);
    
        if (!data_OK) 
          {

          if (GPSECHO_GPS_query)
            {
            Serial.print("GPS sentence not good for navigation: "); Serial.println(GPS_sentence_string);
            Serial.println("I will keep trying...");
            }

          return -1;        
  
          }
                
        // so far so good, so keep going...
        
        // now parse latitude 
        
        GPS_latitude_1_string = GPS_sentence_string.substring(GPRMC_latitude_1_index1, 
        GPRMC_latitude_1_index2);
        GPS_latitude_2_string = GPS_sentence_string.substring(GPRMC_latitude_2_index1, 
        GPRMC_latitude_2_index2);
        GPS_latitude_NS_string = GPS_sentence_string.substring(GPRMC_latitude_NS_index1, 
        GPRMC_latitude_NS_index2);
    
        GPS_latitude_1 = GPS_latitude_1_string.toInt();      
        GPS_latitude_2 = GPS_latitude_2_string.toInt();      
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.println(GPS_latitude_NS_string);
          }
          
        // now parse longitude 
        
        GPS_longitude_1_string = GPS_sentence_string.substring(GPRMC_longitude_1_index1, GPRMC_longitude_1_index2);
        GPS_longitude_2_string = GPS_sentence_string.substring(GPRMC_longitude_2_index1, GPRMC_longitude_2_index2);
        GPS_longitude_EW_string = GPS_sentence_string.substring(GPRMC_longitude_EW_index1, GPRMC_longitude_EW_index2);
    
        GPS_longitude_1 = GPS_longitude_1_string.toInt();      
        GPS_longitude_2 = GPS_longitude_2_string.toInt();      
          
        if(GPSECHO_GPS_query)
          {
          Serial.print("Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.println(GPS_longitude_EW_string); 
          }
    
        // now parse speed and direction. we'll need to locate the 7th and 8th commas in the
        // data sentence to do this. so use the indexOf function to find them.
        // it returns -1 if string wasn't found. the number of digits is not uniquely defined 
        // so we need to find the fields based on the commas separating them from others.
        
        int comma_A_index = GPRMC_longitude_EW_index2;
        int comma_B_index = GPS_sentence_string.indexOf(",", comma_A_index + 1);
        int comma_C_index = GPS_sentence_string.indexOf(",", comma_B_index + 1);
    
        GPS_speed_knots_string = GPS_sentence_string.substring(comma_A_index + 1, comma_B_index); 
        GPS_direction_string = GPS_sentence_string.substring(comma_B_index + 1, comma_C_index); 
        
        GPS_speed_knots = GPS_speed_knots_string.toFloat();
        GPS_direction = GPS_direction_string.toFloat();
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("Speed (knots) = "); Serial.println(GPS_speed_knots);
          Serial.print("Direction (degrees) = "); Serial.println(GPS_direction);
          }
          
        // now get the (UTC) date, in format DDMMYY, e.g. 080618 for 8 June 2018.
        GPS_date_string = GPS_sentence_string.substring(comma_C_index+ + 1, comma_C_index + 7);
        
        if(GPSECHO_GPS_query)
          {
          Serial.print("date, in format ddmmyy = "); Serial.println(GPS_date_string);    
          }

        // print a summary of the data and parsed results:
        if(GPSECHO_GPS_query)
          {
          Serial.print("GPS sentence: "); Serial.println(GPS_sentence_string);

          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
        
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.print(" "); Serial.print(GPS_latitude_NS_string);

          Serial.print("    Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.print(" "); Serial.println(GPS_longitude_EW_string); 

          Serial.print("Speed (knots) = "); Serial.print(GPS_speed_knots);
          Serial.print("     Direction (degrees) = "); Serial.println(GPS_direction);

          Serial.println("There is no satellite or altitude information in a GPRMC data sentence.");
              
          }
      
        // all done with this sentence, so return.
        return 0;
          
        }  // end of "if (data_OK && GPS_command.equals("$GPRMC"))" block

    //////////////////////////////////////////////////////////////////////
    /////////////////////////// GPGGA sentence ///////////////////////////
    //////////////////////////////////////////////////////////////////////
    
      if (data_OK && GPS_command.equals("$GPGGA"))
        {

        if(GPSECHO_GPS_query) 
          {
          Serial.print("\nnew GPS sentence: "); Serial.println(GPS_sentence_string);
          }
    
        // parse the time
    
        GPS_hour_string = GPS_sentence_string.substring(GPGGA_hour_index1, GPGGA_hour_index2);
        GPS_minutes_string = GPS_sentence_string.substring(GPGGA_minutes_index1, GPGGA_minutes_index2);
        GPS_seconds_string = GPS_sentence_string.substring(GPGGA_seconds_index1, GPGGA_seconds_index2);
        GPS_milliseconds_string = GPS_sentence_string.substring(GPGGA_milliseconds_index1, 
        GPGGA_milliseconds_index2);
    
        GPS_hour = GPS_hour_string.toInt();
        GPS_minutes = GPS_minutes_string.toInt();
        GPS_seconds = GPS_seconds_string.toInt();
        GPS_milliseconds = GPS_milliseconds_string.toInt();
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
          }
    
        // now get the fix quality and number of satellites.
    
        GPS_fix_quality_string = GPS_sentence_string.substring(GPGGA_fix_quality_index1, 
        GPGGA_fix_quality_index2);
        GPS_satellites_string = GPS_sentence_string.substring(GPGGA_satellites_index1, 
        GPGGA_satellites_index2);
    
        GPS_fix_quality = GPS_fix_quality_string.toInt();      
        GPS_satellites = GPS_satellites_string.toInt();      
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("fix quality (1 for GPS, 2 for DGPS) = "); Serial.println(GPS_fix_quality);
          Serial.print("number of satellites = "); Serial.println(GPS_satellites);
          }
    
        // now see if the data are valid: we'll expect a fix, and at least three satellites.
    
        bool data_OK = (GPS_fix_quality > 0) && (GPS_satellites >= 3); 
    
        // now look for the asterisk.
        // int asterisk_should_be_here = GPS_sentence_string.length() - 4; 
        int asterisk_should_be_here = GPS_sentence_string.length() - asterisk_backup; 
    
        data_OK = data_OK && (GPS_sentence_string.charAt(asterisk_should_be_here) == '*');
    
        // now check that the sentence is not too short.      
        data_OK = data_OK && (GPS_sentence_string.length() >= GPSMINLENGTH);

        if (!data_OK) 
          {
           
          if (GPSECHO_GPS_query)
            {
            Serial.print("GPS sentence not good for navigation: "); Serial.println(GPS_sentence_string);
            Serial.println("I will keep trying...");
            }
          }
    
        // if data are not good, go back to the top of the loop by breaking out of this if block.
        
        if (!data_OK) break;
            
        // so far so good, so keep going...
        
        // now parse latitude 
        
        GPS_latitude_1_string = GPS_sentence_string.substring(GPGGA_latitude_1_index1, 
        GPGGA_latitude_1_index2);
        GPS_latitude_2_string = GPS_sentence_string.substring(GPGGA_latitude_2_index1, 
        GPGGA_latitude_2_index2);
        GPS_latitude_NS_string = GPS_sentence_string.substring(GPGGA_latitude_NS_index1, 
        GPGGA_latitude_NS_index2);
    
        GPS_latitude_1 = GPS_latitude_1_string.toInt();      
        GPS_latitude_2 = GPS_latitude_2_string.toInt();      
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.println(GPS_latitude_NS_string);
          }
          
        // now parse longitude 
        
        GPS_longitude_1_string = GPS_sentence_string.substring(GPGGA_longitude_1_index1, 
        GPGGA_longitude_1_index2);
        GPS_longitude_2_string = GPS_sentence_string.substring(GPGGA_longitude_2_index1, 
        GPGGA_longitude_2_index2);
        GPS_longitude_EW_string = GPS_sentence_string.substring(GPGGA_longitude_EW_index1, 
        GPGGA_longitude_EW_index2);
    
        GPS_longitude_1 = GPS_longitude_1_string.toInt();      
        GPS_longitude_2 = GPS_longitude_2_string.toInt();      
    
        if(GPSECHO_GPS_query)
          {         
          Serial.print("Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.println(GPS_longitude_EW_string); 
          }
          
        // let's skip the "horizontal dilution" figure and go straight for the altitude now.
        // this begins two fields to the right of the num,ber of satellites so find this
        // by counting commas. use the indexOf function to find them.
        int comma_A_index = GPS_sentence_string.indexOf(",", GPGGA_satellites_index2 + 1);
        int comma_B_index = GPS_sentence_string.indexOf(",", comma_A_index + 1);
    
        GPS_altitude_string = GPS_sentence_string.substring(comma_A_index + 1, comma_B_index); 
        
        GPS_altitude = GPS_altitude_string.toFloat();
    
        if(GPSECHO_GPS_query)
          {
          Serial.print("Altitude (meters) = "); Serial.println(GPS_altitude);
          }

        // print a summary of the data and parsed results:
        if(GPSECHO_GPS_query)
          {
          Serial.print("GPS sentence: "); Serial.println(GPS_sentence_string);

          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
        
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.print(" "); Serial.print(GPS_latitude_NS_string);

          Serial.print("    Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.print(" "); Serial.println(GPS_longitude_EW_string); 

          Serial.print("Speed (knots) = "); Serial.print(GPS_speed_knots);
          Serial.print("     Direction (degrees) = "); Serial.println(GPS_direction);

          Serial.print("Number of satellites: "); Serial.print(GPS_satellites);
          Serial.print("       Altitude (meters): "); Serial.println(GPS_altitude);
              
          }
       
        // all done with this sentence, so return.
        return 0;
       
      }   // end of "if (data_OK && GPS_command.equals("$GPGGA"))" block
  
    // we'll fall through to here (instead of returning) when we've read a complete 
    // sentence, but it doesn't have navigational information (for example, an antenna 
    // status record).
    
    } 

  }

/////////////////////////////////////////////////////////////////////////

// also at the end reset the GPS to the usual configuration.

////////////////// That's it! ////////////////////////
