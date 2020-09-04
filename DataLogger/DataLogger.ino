#include <LiquidCrystal.h>
#include "Adafruit_Keypad.h"

#include "Commands.h"

//LCD variables
const int rs = A8, en = A9, d4 = A10, d5 = A11, d6 = A12, d7 = A13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Keypad definitions
#define KEYPAD_PID3844
#define R1    A3
#define R2    A4
#define R3    A5
#define R4    A6
#define C1    A0
#define C2    A1
#define C3    A2
#define C4    A7

//Must go after key definitions
#include "keypad_config.h"

//Keypad variable
Adafruit_Keypad keypad = Adafruit_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//Class variables
BMEwrapper bme;
RTCwrapper rtc;
SDwrapper sd;

//Command string to keep track of keypad codes
String currCommand = "";

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

  keypad.begin();
}

void loop() {
  checkKeypad();
  rtc.printTime();
  bme.printBME();
  runCommand(currCommand, bme, rtc, sd);
  printLCD();
}

void checkKeypad()
{
  keypad.tick();

  while(keypad.available()){
    keypadEvent e = keypad.read();
    Serial.print((char)e.bit.KEY);
    if(e.bit.EVENT == KEY_JUST_PRESSED) Serial.println(" pressed");
    else if(e.bit.EVENT == KEY_JUST_RELEASED) Serial.println(" released");
  }

  delay(10);
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
