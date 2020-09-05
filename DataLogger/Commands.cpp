#include <LiquidCrystal.h>

#include "Commands.h"
#include "Errors.h"
#include "KeypadVar.h"
#include "LCDHelper.h"

bool CmdCenter::init()
{
  if (!bme.init()) error(BMEerror);
  if (!rtc.init()) error(RTCerror);
  if (!sd.init()) error(SDerror);

  lcd.begin(16, 2);
  lcd.print("98# List Cmds");
}

int CmdCenter::checkCmd()
{
  if (cmd == READBME) return 1;
  if (cmd == WRITEBME) return 2;
  if (cmd == DELETEBME) return 3;
  if (cmd == READRTC) return 4;
  if (cmd == WRITERTC) return 5;
  if (cmd == DELETERTC) return 6;
  if (cmd == READGPS) return 7;
  if (cmd == WRITEGPS) return 8;
  if (cmd == DELETEGPS) return 9;
  if (cmd == READALL) return 10;
  if (cmd == WRITEALL) return 11;
  if (cmd == DELETEALL) return 12;
  
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
  String data = "";
  
  if (mode == BMEFILE)
  {
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      data += bme.collectBME();
      delay(1000);
    }

    sd.writeSD(BMEFILE, data);
  }
  else if (mode == RTCFILE)
  {
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      data += rtc.collectRTC();
      delay(1000);
    }

    sd.writeSD(RTCFILE, data);
  }
  else if (mode == GPSFILE)
  {
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      data += bme.collectBME();
      delay(1000);
    }

    sd.writeSD(GPSFILE, data + "GPS");
  }
  else if (mode == "all")
  {
    String dataRTC = "";
    String dataGPS = "";
    for(int i = 0; i < LOOPTIMES; ++i)
    {
      data += bme.collectBME();
      dataRTC += rtc.collectRTC();
      dataGPS += bme.collectBME();
      delay(1000);
    }

    sd.writeSD(BMEFILE, data);
    sd.writeSD(RTCFILE, dataRTC);
    sd.writeSD(GPSFILE, dataGPS + "GPS");
  }
}

bool CmdCenter::runCmd()
{
  switch(checkCmd())
  {
    case 1:
      sd.printSD(BMEFILE);
      cmd = "Read BME";
      return true;

    case 2:
      loopData(BMEFILE);
      cmd = "Write BME";
      return true;

    case 3:
      sd.deleteSD(BMEFILE);
      cmd = "Delete BME";
      return true;
      
    case 4:
      sd.printSD(RTCFILE);
      cmd = "Read RTC";
      return true;

    case 5:
      loopData(RTCFILE);
      cmd = "Write RTC";
      return true;

    case 6:
      sd.deleteSD(RTCFILE);
      cmd = "Delete RTC";
      return true;

    case 7:
      sd.printSD(GPSFILE);
      cmd = "Read GPS";
      return true;

    case 8:
      loopData(GPSFILE);
      cmd = "Write GPS";
      return true;

    case 9:
      sd.deleteSD(GPSFILE);
      cmd = "Delete GPS";
      return true;

    case 10:
      sd.printSD(BMEFILE);
      sd.printSD(RTCFILE);
      sd.printSD(GPSFILE);
      cmd = "Read ALL";
      return true;

    case 11:
      loopData("all");
      cmd = "Write ALL";
      return true;

    case 12:
      sd.deleteSD(BMEFILE);
      sd.deleteSD(RTCFILE);
      sd.deleteSD(GPSFILE);
      cmd = "Delete ALL";
      return true;

    case 98:
      cmd = "List cmdands";
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
