#include "BMEwrapper.h"
#include "GPSwrapper.h"

#include <SPI.h>

// SD card writing libraries
#include "SdFat.h"
#include "FreeStack.h"
#include "AnalogBinLogger.h"

#define DEFAULTTXT "98# List Cmds"

#define READBME "1#" //output bme680 file to serial window
#define WRITEBME "2#" //write bme680 to sd card using BMEFILE
#define DELETEBME "3#" //delete BMEFILE from sd card
#define READGPS "4#" //output gps file to serial window
#define WRITEGPS "5#" //write gps to sd card using GPSFILE
#define DELETEGPS "6#" //delete GPSFILE from sd card
#define READALL "7#" //output all files to serial window
#define WRITEALL "8#" //write everything to sd card using their respective files
#define DELETEALL "9#" //delete each file from sd card

#define RECORD "11#" //record audio

#define PRINTBME "51#" //prints bme680 to serial window
#define PRINTGPS "52#" //prints gps to serial window

#define BOOTSD "97#" //reboot sd card when reinserted
#define LIST "98#" //list all commands on LCD
#define CLEAR "***" //clears LCD screen

#define LOOPTIMES 2 //number of times to loop through data in seconds
#define DATAPERSECOND 1 //number of times to gather data in one second

//names of the data files
#define BMEFILE "bme"
#define GPSFILE "gps"

//variables for sd
const int chipSelect = 53;

//function declarations for sd
bool initSD();
int findNextFile(String filename);
void printSD(String filename);
void writeSD(String filename, String data);

void logData();
void audioSetup();
void audioLoop();
void adcInit(metadata_t* meta);
void adcStart();
void adcStop();
void checkOverrun();
void dumpData();

//This is the class that controls everything. Honestly, it's terribly named, but I'm not great at naming things.
//It houses the class wrappers that control the modules on the breadboard.
class CmdCenter
{
  private:
    BMEwrapper bme;
    GPSwrapper gps;

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
