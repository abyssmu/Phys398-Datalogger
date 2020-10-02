#include "Includes.h"

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
  if (cmd == DELETEBME) return 3;
  if (cmd == READGPS) return 4;
  if (cmd == WRITEGPS) return 5;
  if (cmd == DELETEGPS) return 6;
  if (cmd == READALL) return 7;
  if (cmd == WRITEALL) return 8;
  if (cmd == DELETEALL) return 9;

  if (cmd == RECORD) return 11;

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
}

void CmdCenter::loopData(String mode)
{
  String data = ""; //collects all data at once and then writes to sd
  int timeDelay = 1000 / DATAPERSECOND; //time between each data collection
  
  if (mode == BMEFILE)
  {
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      data += bme.collectBME();
      delay(timeDelay);
    }

    writeSD(BMEFILE, data);
  }
  else if (mode == GPSFILE)
  {
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      //data += gps.collectGPS();
      delay(timeDelay);
    }

    writeSD(GPSFILE, data);
  }
  else if (mode == "all")
  {
    String dataGPS = "";
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      data += bme.collectBME();
      //dataGPS += gps.collectGPS();
      delay(timeDelay);
    }

    writeSD(BMEFILE, data);
    writeSD(GPSFILE, dataGPS);
  }
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
      loopData(BMEFILE);
      cmd = "Write BME";
      return true;

    case 3:
      sd.remove(BMEFILE);
      cmd = "Delete BME";
      return true;

    case 4:
      printSD(GPSFILE);
      cmd = "Read GPS";
      return true;

    case 5:
      loopData(GPSFILE);
      cmd = "Write GPS";
      return true;

    case 6:
      sd.remove(GPSFILE);
      cmd = "Delete GPS";
      return true;

    case 7:
      printSD(BMEFILE);
      printSD(GPSFILE);
      cmd = "Read ALL";
      return true;

    case 8:
      loopData("all");
      cmd = "Write ALL";
      return true;

    case 9:
      sd.remove(BMEFILE);
      sd.remove(GPSFILE);
      cmd = "Delete ALL";
      return true;

    case 11:
      audioLoop();
      cmd = "Record";
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
  File dir = sd.open("/");
  String fCaps = filename;
  fCaps.toUpperCase();
  int largest = 0;
  
  while(true)
  {
    File entry = dir.openNextFile();
    if(!entry) break;

    String n = String(entry.name());
    
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

    entry.close();
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
  
  filename += "-";

  if(num < 10)
  {
    filename += "00" + String(num) + ".txt";
  }
  else if(num < 100)
  {
    filename += "0" + String(num) + ".txt";
  }
  else
  {
    filename += String(num) + ".txt";
  }

  File dataFile = sd.open(filename, FILE_WRITE);

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

void audioSetup(void) {

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Print a message to the LCD.
  //             0123456789012345       0123456789012345
  LCD_message(F("Physics 398DLP  "), F("Record at 32kHz "));

  // delay a bit so I have time to see the display.
  delay(1000);
  
  // if we will blink the LED when there are errors, tell the Arduino that
  // the pin is to be an output.
  if (ERROR_LED_PIN >= 0) {pinMode(ERROR_LED_PIN, OUTPUT);}

  // fire up the serial monitor line
  Serial.begin(9600);

  // Read the first sample pin to force the ADC to initialize itself.
  analogRead(PIN_LIST[0]);

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    sd.initErrorPrint();
    fatalBlink();
  }

}

//////////////////////////////////////////////////////////////////////
/////////////////////////// loop function ////////////////////////////
//////////////////////////////////////////////////////////////////////

void audioLoop(void) {
  
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

  // now see if user would like to record more audio. hang in a while loop until
  // we detect a # from the keypad.
  /*
  char the_key = '?'; 
  char got_a_key;
  
  while(1) {
    
    // getKeys will load any keys pressed into key array.
    got_a_key = customKeypad.getKeys();

    if (got_a_key) {the_key = customKeypad.key[0].kchar;}

    if (the_key == '#') {break;}
        
  }
  */
  // land here when we got a keypad #. this'll cause us to reenter the
  // loop function and call logData again.

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

  String metaName = "";
  for (int i = 0; i < BASE_NAME_SIZE+2; i++) {
    metaName += binName[i];
  }
  metaName += ".txt";
  Serial.println(metaName);
  File metaFile = sd.open(metaName, FILE_WRITE);
  metaFile.print("Current time is:");
  now = rtc.now();
  metaFile.print(now.hour(), DEC);
  metaFile.print(':');
  if(now.minute() < 10)   metaFile.print(0);
  metaFile.print(now.minute(), DEC);
  metaFile.print(':');
  if(now.second() < 10)   metaFile.print(0);
  metaFile.print(now.second(), DEC);

  metaFile.print("   Date (dd/mm/yyyy): ");
  metaFile.print(now.day(), DEC); metaFile.print('/');
  if(int(now.month()) < 10) metaFile.print("0");
  metaFile.print(now.month(), DEC); metaFile.print("/");
  metaFile.println(now.year(), DEC);
  metaFile.close();

  
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

void gpsSetup () {

  // fire up the serial monitor
  Serial.begin(9600);
  while(!Serial){};
  
  Serial.println("Let's set the DS3231 real time clock from the GPS after acquiring satellites.");

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
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Now looking for ");
  lcd.setCursor(0, 1);
  lcd.print("GPS satellites  ");
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

    Serial.print("Day: "); Serial.println(GPS_day);
    Serial.print("Month: "); Serial.println(GPS_month);
    Serial.print("Year: "); Serial.println(GPS_year);
    Serial.print("Time: "); Serial.print(GPS_hour); Serial.print(":"); Serial.print(GPS_minutes); Serial.print(":"); Serial.println(GPS_seconds);
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

/////////////////////////////////////////////////////////////////////////

// also at the end reset the GPS to the usual configuration.

////////////////// That's it! ////////////////////////
