#include <LiquidCrystal.h>

//Womack pins
//const int rs = A8, en = A9, d4 = A10, d5 = A11, d6 = A12, d7 = A13;

//Logger pins
const int rs = 12, en = 11, d4 = 36, d5 = 34, d6 = 32, d7 = 30;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// total number of characters in one line of the LCD display
#define LCDWIDTH 16

void printLCD(String cmd)
{
  if (cmd != "")
  {
    lcd.clear();
    lcd.print(cmd);
  }
}

void delayLCD(String s, int t)
{
  printLCD(s);
  delay(t);
  lcd.clear();
}

void loopcmds(String s)
{
  lcd.clear();
  printLCD(s);
  delay(2000);
}

void listcmd()
{
  loopcmds("1# Read BME");
  loopcmds("2# Write BME");
  loopcmds("4# Read GPS");
  loopcmds("5# Write GPS");

  loopcmds("7# Test Drift");
  loopcmds("8# Data Collect");

  loopcmds("51# Print BME");
  loopcmds("52# Print GPS");
  
  loopcmds("97# Reboot SD");
  loopcmds("98# List Cmds");
  loopcmds("*** Clear");
}

//////////////////////////////////////////////////////////////////////
///////////////////// fatalBlink function ////////////////////////////
//////////////////////////////////////////////////////////////////////

// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors. On an Arduino Mega 2560, the LED is connected to pin 13.
const int8_t ERROR_LED_PIN = 13;

void fatalBlink() {
  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
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
