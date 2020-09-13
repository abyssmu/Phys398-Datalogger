#include "BMEwrapper.h"
#include "GPSwrapper.h"
#include "RTCwrapper.h"
#include "SDwrapper.h"

#define DEFAULTTXT "98# List Cmds"

#define READBME "1#" //output bme680 file to serial window
#define WRITEBME "2#" //write bme680 to sd card using BMEFILE
#define DELETEBME "3#" //delete BMEFILE from sd card
#define READRTC "4#" //output rtc file to serial window
#define WRITERTC "5#" //write rtc to sd card in unix time using RTCFILE
#define DELETERTC "6#" //delete RTCFILE from sd card
#define READGPS "7#" //output gps file to serial window
#define WRITEGPS "8#" //write gps to sd card using GPSFILE
#define DELETEGPS "9#" //delete GPSFILE from sd card
#define READALL "10#" //output all files to serial window
#define WRITEALL "11#" //write everything to sd card using their respective files
#define DELETEALL "12#" //delete each file from sd card

#define PRINTBME "51#" //prints bme680 to serial window
#define PRINTRTC "52#" //prints rtc to serial window
#define PRINTGPS "53#" //prints gps to serial window

#define BOOTSD "97#" //reboot sd card when reinserted
#define LIST "98#" //list all commands on LCD
#define CLEAR "***" //clears LCD screen

#define LOOPTIMES 10 //number of times to loop through data in seconds
#define DATAPERSECOND 1 //number of times to gather data in one second

//names of the data files
#define BMEFILE "bmedata.txt"
#define RTCFILE "rtcdata.txt"
#define GPSFILE "gpsdata.txt"

//This is the class that controls everything. Honestly, it's terribly named, but I'm not great at naming things.
//It houses the class wrappers that control the modules on the breadboard.
class CmdCenter
{
  private:
    BMEwrapper bme;
    GPSwrapper gps;
    RTCwrapper rtc;
    SDwrapper sd;

    //cmdand string to keep track of keypad codes
    String cmd = "";
    //Boolean cmdand to see if data can be logged
    bool logData = false;

  public:
    CmdCenter() {}
    ~CmdCenter() {}

    bool init(); //initializes pins, classes, and modules
    int checkCmd(); //checks current command against list of commands
    char checkKey(); //obtains keys pressed by keypad
    void getInput(); //checks for any input from keypad and checks for commands
    void loopData(String filename); //loops through data collection LOOPTIMES
    bool runCmd(); //runs the commands obtained from checkCmd
};
