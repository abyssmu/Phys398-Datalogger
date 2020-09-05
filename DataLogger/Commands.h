#include "BMEwrapper.h"
#include "RTCwrapper.h"
#include "SDwrapper.h"

#define READBME "1#"
#define WRITEBME "2#"
#define DELETEBME "3#"
#define READRTC "4#"
#define WRITERTC "5#"
#define DELETERTC "6#"
#define READGPS "7#"
#define WRITEGPS "8#"
#define DELETEGPS "9#"
#define READALL "10#"
#define WRITEALL "11#"
#define DELETEALL "12#"
#define LIST "98#"
#define CLEAR "***"

#define LOOPTIMES 10

#define BMEFILE "bmedata.txt"
#define RTCFILE "rtcdata.txt"
#define GPSFILE "gpsdata.txt"

class CmdCenter
{
  private:
    BMEwrapper bme;
    RTCwrapper rtc;
    SDwrapper sd;

    //cmdand string to keep track of keypad codes
    String cmd = "";
    //Boolean cmdand to see if data can be logged
    bool logData = false;

  public:
    CmdCenter() {}
    ~CmdCenter() {}

    bool init();
    int checkCmd();
    char checkKey();
    void getInput();
    void loopData(String filename);
    bool runCmd();
};
