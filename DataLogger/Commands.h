#include "BMEwrapper.h"

//This is the class that controls everything. Honestly, it's terribly named, but I'm not great at naming things.
//It houses the class wrappers that control the modules on the breadboard.
class CmdCenter
{
  private:
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
    bool runCmd(); //runs the commands obtained from checkCmd

    bool collectData(); //function to run full data collection routine
};
