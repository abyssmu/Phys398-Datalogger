#include <LiquidCrystal.h>

const int rs = A8, en = A9, d4 = A10, d5 = A11, d6 = A12, d7 = A13;
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
  loopcmds("3# Delete BME");
  loopcmds("4# Read RTC");
  loopcmds("5# Read RTC");
  loopcmds("6# Read RTC");
  loopcmds("7# Read RTC");
  loopcmds("8# Read RTC");
  loopcmds("9# Read RTC");
  loopcmds("10# Read RTC");
  loopcmds("11# Read RTC");
  loopcmds("12# Read RTC");

  loopcmds("98# List Cmds");
  loopcmds("*** Clear");
}
