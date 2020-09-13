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

// Serial Peripheral Interface library
#include <SPI.h>

// SD card writing libraries
#include "SdFat.h"
#include "FreeStack.h"
#include "AnalogBinLogger.h"

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

//////////////////////////////////////////////////////////////////////
///////////////////////////    function ///////////////////////////
//////////////////////////////////////////////////////////////////////

void setup(void) {
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

void loop(void) {
  
  // tell the user what's happening.
  Serial.println(F("About to record microphone data. Hit keypad * to stop."));
  //             0123456789012345       0123456789012345
  LCD_message(F("Record at 32kHz "), F("Stop: keypad *  "));

  // record ADC data and write as binary to the SD.
  logData();


  
  // status message
  Serial.println(F("Hit keyboard # to record again.\n"));
  //             0123456789012345       0123456789012345
  LCD_message(F("Hit keypad # to "), F("record more     "));

  // now see if user would like to record more audio. hang in a while loop until
  // we detect a # from the keypad.

  char the_key = '?'; 
  char got_a_key;
  
  while(1) {
    
    // getKeys will load any keys pressed into key array.
    got_a_key = kpd.getKeys();

    if (got_a_key) {the_key = kpd.key[0].kchar;}

    if (the_key == '#') {break;}
        
  }
  
  // land here when we got a keypad #. this'll cause us to reenter the
  // loop function and call logData again.

}

//////////////////////////////////////////////////////////////////////
////////////////////////// LCD_message function //////////////////////
//////////////////////////////////////////////////////////////////////

void LCD_message(String line1, String line2)
{
  // write two lines (of 16 characters each, maximum) to the LCD display.
  // I assume an object named "lcd" has been created already, has been 
  // initialized in setup, and is global.

  // set the cursor to the beginning of the first line, clear the line, then write.
  lcd.setCursor(0, 0);
  lcd.print(F("                "));
  lcd.setCursor(0, 0);
  lcd.print(line1);

  // now do the next line.
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(line2);

  return;
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
///////////////////// fatalBlink function ////////////////////////////
//////////////////////////////////////////////////////////////////////



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

  Serial.println();

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
  
  // Get the address of the file on the SD.
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

      // Write block to SD.
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

    // kpd.getKeys fills kpd.key[ ] array with up to 10 active keys, if the user pushes
    // more than one key at the same time. But I am only going to look at the 
    // first key pressed. 
    
    // intialize character read to something not on the keypad.
    char the_key = '?'; 

    // Does the user want us to stop? Keypad * will signal this.
    bool got_a_key = kpd.getKeys();

    if (got_a_key) {the_key = kpd.key[0].kchar;}
    
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
