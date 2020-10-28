#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"
SdFat SD;

#define SD_CS_PIN SS
File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  unsigned long before = 0;
  unsigned long after = 0;
  long loopTimes = 1000;

/////////////////////////////////////////////////////////////////////////////

  Serial.println("Begin file all in loop test");

  before = micros();
  for (long i = 0; i < loopTimes; ++i)
  {
    myFile = SD.open("test1.txt", FILE_WRITE);
    myFile.println(i);
    myFile.close();
  }
  after = micros();

  Serial.println("After file in loop test");
  Serial.print("Time taken: "); Serial.println(after - before);
  Serial.println();

/////////////////////////////////////////////////////////////////////////////

  Serial.println("Begin file in loop test");

  before = micros();
  myFile = SD.open("test2.txt", FILE_WRITE);
  for (long i = 0; i < loopTimes; ++i)
  {
    myFile.println(i);
  }
  myFile.close();
  after = micros();

  Serial.println("After file in loop test");
  Serial.print("Time taken: "); Serial.println(after - before);
  Serial.println();

/////////////////////////////////////////////////////////////////////////////

  Serial.println("Begin file out loop test");

  String data = "";

  before = micros();
  for (long i = 0; i < loopTimes; ++i)
  {
    data += String(i) + "\n";
  }

  myFile = SD.open("test3.txt", FILE_WRITE);
  myFile.print(data);
  myFile.close();
  after = micros();

/////////////////////////////////////////////////////////////////////////////

  Serial.println("After file out loop test");
  Serial.print("Time taken: "); Serial.println(after - before);
  Serial.println();
}

void loop() {
  // nothing happens after setup
}