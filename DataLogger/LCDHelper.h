#include <LiquidCrystal.h>

//Womack pins
//const int rs = A8, en = A9, d4 = A10, d5 = A11, d6 = A12, d7 = A13;

//Logger pins
const int rs = 12, en = 11, d4 = 36, d5 = 34, d6 = 32, d7 = 30;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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
  loopcmds("7# Read ALL");
  loopcmds("8# Write ALL");

  loopcmds("51# Print BME");
  loopcmds("52# Print GPS");
  
  loopcmds("97# Reboot SD");
  loopcmds("98# List Cmds");
  loopcmds("*** Clear");
}
