#include <LiquidCrystal.h>

#include "Commands.h"
#include "Errors.h"
#include "KeypadVar.h"
#include "LCDHelper.h"

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
  if (!gps.init())
  {
    lcd.clear();
    lcd.print("GPS error.");
    error(GPSerror);
  }
  if (!initSD())
  {
    lcd.clear();
    lcd.print("SD error.");
  }
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
      data += gps.collectGPS();
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
      dataGPS += gps.collectGPS();
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
      SD.remove(BMEFILE);
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
      SD.remove(GPSFILE);
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
      SD.remove(BMEFILE);
      SD.remove(GPSFILE);
      cmd = "Delete ALL";
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
      for(int i = 0; i < LOOPTIMES; ++i)
      {
        gps.printGPS();
        delay(1000);
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
  return SD.begin(chipSelect);
}

int findNextFile(String filename)
{
  File dir = SD.open("/");
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
  
  File dataFile = SD.open(filename);
  
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

  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}
