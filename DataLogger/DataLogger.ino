#include <LiquidCrystal.h>

#include "Commands.h"

BMEwrapper bme;
RTCwrapper rtc;
SDwrapper sd;

String currCommand = "";

const int rs = A8, en = A9, d4 = A10, d5 = A11, d6 = A12, d7 = A13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (!bme.init()) failedBoot("BME failed to start");
  if (!rtc.init()) failedBoot("RTC failed to start");
  if (!sd.init()) failedBoot("SD failed to start");
    
  lcd.begin(16, 2);
  lcd.print("1# writes to SD");
}

void loop() {
  rtc.printTime();
  bme.printBME();
  runCommand(currCommand, bme, rtc, sd);
  printLCD();
}

void failedBoot(String output)
{
  Serial.println(output);
  while (1);
}

void printLCD()
{
  lcd.setCursor(0, 1);
  lcd.print(currCommand);
}
